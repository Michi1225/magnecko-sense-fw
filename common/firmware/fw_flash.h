#ifndef FW_FLASH_H
#define FW_FLASH_H
#include "fw_update.h"
#include "../s28hs512t/semper_flash.h"
typedef struct { SEMPER_Handle_t *flash; void (*idle)(void); } fw_flash_t;
/** @brief Build the bounded flash callbacks used by the portable updater. */
fw_io_t fw_flash_io(fw_flash_t *flash);
/** @brief Permit only the single FSBL sector; never parameters or application/metadata sectors. */
fw_io_t fw_flash_fsbl_io(fw_flash_t *flash);
/** @brief Permit only canonical parameters and their journal, with 4 KiB erases. */
fw_io_t fw_flash_parameters_io(fw_flash_t *flash);
/** @brief Permit only the two history sectors, with aligned single-ECC-unit writes. */
fw_io_t fw_flash_history_io(fw_flash_t *flash);
/** @brief Restore and verify legacy SPI access before resetting into the boot ROM. */
int fw_flash_prepare_reset(fw_flash_t *flash);
/** @brief Reload the independent watchdog if it is already running. */
void fw_watchdog_refresh(void);
/** @brief Start the independent watchdog with a nominal 32-second trial boot timeout. */
int fw_watchdog_start(void);
#endif
