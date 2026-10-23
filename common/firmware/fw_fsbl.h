#ifndef FW_FSBL_H
#define FW_FSBL_H
#include "fw_update.h"
#define FW_FSBL_BASE 0x00040000U
/* Signed header/padding plus the FSBL's 255 KiB code allocation. */
#define FW_FSBL_MAX 0x00040000U
#define FW_FSBL_RAM 0x34180000U
enum { FW_FSBL_IDLE, FW_FSBL_RECEIVING, FW_FSBL_READY, FW_FSBL_INSTALLED, FW_FSBL_RECOVERY };
typedef struct {
    fw_io_t io;
    struct fw_history *history;
    uint32_t state, received, error;
    fw_image_t active, incoming, backup_image;
    uint8_t staging[FW_FSBL_MAX], backup[FW_FSBL_MAX];
} fw_fsbl_t;
/** @brief Validate an aligned, unencrypted STM32 v2.3 FSBL and its SRAM vectors. */
int fw_fsbl_validate(const uint8_t *data,uint32_t size,fw_image_t *image);
/** @brief Read and validate the installed FSBL without changing any flash. */
int fw_fsbl_open(fw_fsbl_t *f,const fw_io_t *io);
/** @brief Receive a candidate in RAM; the installed boot sector is untouched. */
int fw_fsbl_begin(fw_fsbl_t *f,uint32_t size,const uint8_t sha256[32]);
int fw_fsbl_data(fw_fsbl_t *f,uint32_t offset,const uint8_t *data,uint32_t size,uint32_t crc);
int fw_fsbl_finish(fw_fsbl_t *f);
/** @brief Replace and read back FSBL, restoring its RAM backup on a write failure.
 * Power loss during this single ROM boot-sector rewrite requires SWD recovery.
 * The supplied IO permits only the FSBL sector; applications/parameters are excluded.
 */
int fw_fsbl_apply(fw_fsbl_t *f,const uint8_t expected_sha256[32]);
/** @brief Discard a RAM candidate; never discard the backup during failed recovery. */
int fw_fsbl_abort(fw_fsbl_t *f);
#endif
