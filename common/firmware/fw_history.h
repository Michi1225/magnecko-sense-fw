#ifndef FW_HISTORY_H
#define FW_HISTORY_H
#include "fw_update.h"

#define FW_HISTORY_BASE 0x01180000U
#define FW_HISTORY_SIZE (2U * FW_ERASE_SIZE)
#define FW_HISTORY_RECORD_SIZE 128U
#define FW_HISTORY_CAPACITY (FW_HISTORY_SIZE / FW_HISTORY_RECORD_SIZE)
#define FW_HISTORY_PAGE 3U
#define FW_HISTORY_FSBL 2U
enum {
    FW_HISTORY_ADOPTED=1, FW_HISTORY_UPLOAD_STARTED, FW_HISTORY_VERIFIED,
    FW_HISTORY_TRIAL, FW_HISTORY_CONFIRMED, FW_HISTORY_ROLLBACK,
    FW_HISTORY_FAILED, FW_HISTORY_ABORTED, FW_HISTORY_NO_IMAGE,
    FW_HISTORY_WARNINGS_CLEARED, FW_HISTORY_FSBL_APPLY,
    FW_HISTORY_FSBL_INSTALLED, FW_HISTORY_FSBL_RESTORED,
    FW_HISTORY_FSBL_RECOVERY, FW_HISTORY_PARAMETERS_APPLIED,
    FW_HISTORY_PARAMETERS_FAILED
};
typedef struct fw_history {
    fw_io_t io;
    uint32_t newest, sequence, next, last_error;
    uint32_t (*uptime_ms)(void); /* NULL in FSBL: timestamp unavailable. */
    bool ready, readable;
} fw_history_t;

/** @brief Scan the two-sector journal, ignoring uncommitted/corrupt records. No writes. */
int fw_history_open(fw_history_t *h,const fw_io_t *io,uint32_t (*uptime_ms)(void));
/** @brief Append a diagnostic event. Failures never change update/boot policy. */
int fw_history_record(fw_history_t *h,uint32_t event,uint32_t target,uint32_t result,
                      uint32_t generation,uint32_t flags,const fw_image_t *image);
/** @brief Read up to three raw records newest first; FW_NONE starts/ends traversal. */
int fw_history_page(fw_history_t *h,uint32_t cursor,uint8_t *records,uint32_t *count,uint32_t *next);
#endif
