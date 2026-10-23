#ifndef FW_PARAMETERS_H
#define FW_PARAMETERS_H
#include "fw_update.h"

#define FW_PARAMETERS_MAX 16384U
#define FW_PARAMETERS_SECTOR 4096U
#define FW_PARAMETERS_JOURNAL 0x4000U
#define FW_PARAMETERS_COMMIT 0x8000U
#define FW_PARAMETERS_END 0x9000U
#define FW_PARAMETERS_FSBL 2U
enum { FW_PARAMETERS_IDLE, FW_PARAMETERS_RECEIVING, FW_PARAMETERS_READY, FW_PARAMETERS_APPLIED };
typedef struct {
    fw_io_t io;
    uint32_t state, received, total, error, target, size, image_size;
    uint8_t image_sha256[32], json_sha256[32];
    uint8_t incoming[FW_PARAMETERS_MAX], previous[FW_PARAMETERS_MAX], merged[FW_PARAMETERS_MAX];
} fw_parameters_t;

/** @brief Load canonical parameters or the last completely committed journal. */
int fw_parameters_load(const fw_io_t *io,uint8_t *data,uint32_t *size,bool *journal);
/** @brief Complete an interrupted canonical write before reusing its journal. */
int fw_parameters_recover(const fw_io_t *io,uint8_t *scratch);
void fw_parameters_open(fw_parameters_t *p,const fw_io_t *io);
/** @brief Stage generated provenance bound to one image and physical slot. */
int fw_parameters_begin(fw_parameters_t *p,uint32_t size,uint32_t target,uint32_t image_size,
                        const uint8_t image_sha256[32],const uint8_t json_sha256[32]);
int fw_parameters_data(fw_parameters_t *p,uint32_t offset,const uint8_t *data,uint32_t size,uint32_t crc);
/** @brief Validate and merge only the selected image and generated provenance fields. */
int fw_parameters_finish(fw_parameters_t *p);
/** @brief Journal and verify parameters only after the bound image is installed. */
int fw_parameters_apply(fw_parameters_t *p,uint32_t target,const fw_image_t *installed);
void fw_parameters_abort(fw_parameters_t *p);
/** @brief Merge a strict JSON provenance envelope without replacing editable fields. */
int fw_parameters_merge(const uint8_t *previous,const uint8_t *envelope,uint32_t target,
                        uint32_t image_size,const uint8_t sha256[32],uint8_t *output,uint32_t *size);
#endif
