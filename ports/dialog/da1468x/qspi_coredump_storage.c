//! @file
//!
//! Copyright (c) Memfault, Inc.
//! See License.txt for details
//!
//! A coredump storage implementation that uses the NVMS log partition for storing crash
//! information. The Dialog NVMS layer is required. See custom_config_qspi.h to enable both
//! the dg_configFLASH_ADAPTER and dg_configNVMS_ADAPTER features.
//!
//! The Dialog DA14683 USB dev board has the W25Q80EW QSPI Flash (8Bb/1MB). We need 64kB to store
//! all of RAM (64kB). The program sector size is 256B and the erase block sizes are: 4kB, 32kB,
//! 64kB, or the entire chip. The 4kB erase block is a "sector erase (0x20)" while the larger
//! blocks are "block erase (0x52)". The part can handle up to a 104MHz SCLK and has 100k
//! write-cycle durability.
//!
//! The details of the Flash chip are not important for this implementation since Dialog supplies
//! a non-volatile memory layer, ad_nvms that takes care of all the details.
//!
//! To use QSPI for coredump storage the user needs to:
//! 1. Add "#define MEMFAULT_PLATFORM_COREDUMP_STORAGE_USE_FLASH 1" to memfault_platform_config.h
//! 1. call ad_nvms_init() before using the functions in this API
//! 2. patch the Dialog SDK with the Memfault QSPI coredump storage patch,
//!    e.g. patch -p1 < /path/to/memfault_qspi_coredump_storage.patch
//! 3. call memfault_platform_coredump_storage_boot() from your memfault_platform_boot() implementation

#include "memfault/config.h"

#if MEMFAULT_PLATFORM_COREDUMP_STORAGE_USE_FLASH

#include "memfault/panics/coredump.h"

#include "memfault/core/math.h"
#include "memfault/core/debug_log.h"
#include "memfault/core/platform/core.h"
#include "memfault/panics/assert.h"
#include "memfault/ports/watchdog.h"
#include "memfault/util/crc16_ccitt.h"

#include "ad_nvms.h"
#include "hw_cpm.h"
#include "hw_qspi.h"
#include "partition_def.h"
#include "flash_partitions.h"
#include <ad_nvms_direct.h>

#include <string.h>

// Ensure Dialog NVMS setup as needed by this implementation.
#if dg_configNVMS_ADAPTER == 0
#error "Port requires dg_configNVMS_ADAPTER 1"
#endif

#if dg_configFLASH_ADAPTER == 0
#error "Port requires dg_configFLASH_ADAPTER 1"
#endif

// We default to the NVMS_LOG_PART if the user doesn't specify a partition.
#ifndef MEMFAULT_PLATFORM_COREDUMP_STORAGE_PARTITION
#define MEMFAULT_PLATFORM_COREDUMP_STORAGE_PARTITION NVMS_LOG_PART
#endif

// By default, the size used will match the size of the partition selected but we expose a
// configuration option so a user can truncate to the beginning of the partition
#ifndef MEMFAULT_PLATFORM_COREDUMP_STORAGE_MAX_SIZE_BYTES
#define MEMFAULT_PLATFORM_COREDUMP_STORAGE_MAX_SIZE_BYTES UINT32_MAX
#endif

// The NVMS partition will appear to us as a block of storage from "address" zero to
// partition_size - 1. For this reason we do not need to have explicit start and end addresses.
typedef struct nvms_partition_t {
  nvms_t handle; //! Opaque handle to the partition used for coredumps.
  size_t size;   //! The size in bytes of the partition we will use for coredumps.
} nvms_partition_t;

#define QSPI_COREDUMP_PART_INIT_MAGIC 0x45524f43

typedef struct {
  uint32_t magic;
  nvms_partition_t partition;
  uint32_t crc; //! Must be last element in the structure.
} sQspiCoredumpPartitionInfo;

static sQspiCoredumpPartitionInfo s_qspi_coredump_partition_info;

static struct partition_t static_mem_part = {.next = NULL,
                        .driver=NULL,
                        .driver_data=NULL,
                        .data = {.flags= 0,
                                .magic=234,
                                .reserved2 = {0,0,0,0,0,0,0,0},
                                .sector_count = HPY_MEMFAULT_PART_SIZE/FLASH_SECTOR_SIZE,
                                .start_sector = HPY_MEMFAULT_PART_START/FLASH_SECTOR_SIZE,
                                .type = HPY_MEMFAULT_PART,
                                .valid = 0xFF,
                                }};

//! Set-only flag to prevent PM deferred flash ops. Can't go into
//! sQspiCoredumpPartitionInfo because of CRC and changing values of flag.
static bool s_memfault_using_qspi;

static uint32_t prv_get_partition_info_crc(void) {
  return memfault_crc16_ccitt_compute(MEMFAULT_CRC16_CCITT_INITIAL_VALUE,
                                      &s_qspi_coredump_partition_info,
                                      offsetof(sQspiCoredumpPartitionInfo, crc));
}

static const bool prv_validate(void) {
  const uint32_t crc = prv_get_partition_info_crc();
  return crc == s_qspi_coredump_partition_info.crc;
}

// Error writing to flash - should never happen & likely detects a configuration error
// Call the reboot handler which will halt the device if a debugger is attached and then reboot
MEMFAULT_NO_OPT
static void prv_coredump_writer_assert_and_reboot(int error_code) {
  memfault_platform_halt_if_debugging();
  memfault_platform_reboot();
}

// There is a non-trivial bit of setup for the NVRAM on Dialog's QSPI. To use QSPI for
// coredump storage be sure to call this function early, e.g. from memfault_platform_boot().
// This is Dialog specific and not extern'd in a header file.
// NOTE: The user must call ad_nvms_init() before calling this function.
void memfault_platform_coredump_storage_boot(void) {

  // ad_nvms_open() should not fail if above check succeeds.
  s_qspi_coredump_partition_info.partition.handle = 0;
  const size_t partition_size = HPY_MEMFAULT_PART_SIZE;
  s_qspi_coredump_partition_info.partition.size = MEMFAULT_MIN(
      partition_size, MEMFAULT_PLATFORM_COREDUMP_STORAGE_MAX_SIZE_BYTES);

  s_qspi_coredump_partition_info.magic = QSPI_COREDUMP_PART_INIT_MAGIC;
  s_qspi_coredump_partition_info.crc = prv_get_partition_info_crc();
}

// This function allows sys_power_mgr.c:pm_register_qspi_operation() to
// determine if it safe to defer QSPI operations to a worker thread or not.
// This is Dialog specific and not extern'd in a header file.
bool memfault_platform_saving_coredump(void) {
  return s_memfault_using_qspi;
}

// We need to override the weak nop version of memfault_platform_coredump_save_begin()
// to check if it's safe to use the flash with interrupts disabled. We just need to call
// our checker function added to ad_flash.c in the Dialog SDK.
bool memfault_platform_coredump_save_begin(void) {

  // Unconditionally feed the watchdog. If it's not in use this is benign. If
  // it is in use this will give us another 2.6s to complete the coredump save.
  if (memfault_software_watchdog_feed() != 0) {
    return false;
  }

  // Signal to sys_power_mgr.c:pm_register_qspi_operation() that it should not
  // attempt to use deferred Flash ops at all from this point on.
  s_memfault_using_qspi = true;

  return true;
}

void memfault_platform_coredump_storage_get_info(sMfltCoredumpStorageInfo *info) {
  MEMFAULT_ASSERT(info);

  // we are about to perform a sequence of operations on coredump storage
  // sanity check that the memory holding the info is populated and not corrupted
  const bool validated = prv_validate();
  if (validated == false) {
    *info = (sMfltCoredumpStorageInfo) {0};
    return;
  }

  *info = (sMfltCoredumpStorageInfo) {
    .size = s_qspi_coredump_partition_info.partition.size,
    .sector_size = 0, // No longer used
  };
}

bool memfault_platform_coredump_storage_read(uint32_t offset, void *data,
                                             size_t data_len) {
  MEMFAULT_ASSERT(data && data_len);

  if ((offset + data_len) > HPY_MEMFAULT_PART_SIZE) {
    return false;
  }

  // TODO -check for real partition first
  int written = 0;
  bool output_bool = true;

  written = ad_nvms_direct_driver.read(&static_mem_part, offset, data, data_len);
  output_bool = (written == data_len);

  return output_bool;
}


bool memfault_platform_coredump_storage_write(uint32_t offset, const void *data,
                                              size_t data_len) {
  // Failing to write while in a fault handler due to bad args? Take the normal reboot path.
  if (data == NULL || data_len == 0) {
    prv_coredump_writer_assert_and_reboot(-1);
  }

  // It is NOT required that it be sector boundary aligned
//  if(0 != offset % FLASH_SECTOR_SIZE)
//  {
//      return false;
//  }

  // No support for truncation.
  if ((offset + data_len) > HPY_MEMFAULT_PART_SIZE) {
    return false;
  }

  // TODO -check for real partition first?
  int written = 0;
  bool output_bool = true;

  written = ad_nvms_direct_driver.write(&static_mem_part, offset, data, data_len);
  output_bool = (written == data_len);

  return output_bool;
}

bool memfault_platform_coredump_storage_erase(uint32_t offset, size_t erase_size) {
  if(0 != offset % FLASH_SECTOR_SIZE ||
     0 != erase_size % FLASH_SECTOR_SIZE
          )
  {
      return false;
  }

  if ((offset + erase_size) > HPY_MEMFAULT_PART_SIZE) {
    return false;
  }

  int written = 0;
  bool output_bool = true;

  output_bool = ad_nvms_direct_driver.erase(&static_mem_part, offset, erase_size);

  return output_bool;
}

void memfault_platform_coredump_storage_clear(void) {
  memfault_platform_coredump_storage_erase(0, HPY_MEMFAULT_PART_SIZE);
}

#endif /* MEMFAULT_PLATFORM_COREDUMP_STORAGE_USE_FLASH */
