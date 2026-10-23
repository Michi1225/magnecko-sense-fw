#ifndef FW_UPDATE_H
#define FW_UPDATE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FW_SLOT_A 0x00100000U
#define FW_SLOT_B 0x00900000U
#define FW_SLOT_SIZE 0x00800000U
#define FW_MAX_IMAGE 0x00080000U
#define FW_META_A 0x01100000U
#define FW_META_B 0x01140000U
#define FW_ERASE_SIZE 0x00040000U
#define FW_NONE UINT32_MAX
#define FW_VECTOR_OFFSET 0x400U
#define FW_RAM_BASE 0x34000000U
#define FW_CHUNK_SIZE 512U
#define FW_RECORD_SIZE 124U

#define FW_UPLOADING       (1U << 0)
#define FW_UPLOADED        (1U << 1)
#define FW_VERIFIED        (1U << 2)
#define FW_CONFIRMED       (1U << 3)
#define FW_FALLBACK        (1U << 4)
#define FW_UPLOAD_FAILED   (1U << 5)
#define FW_NO_VALID_IMAGE  (1U << 6)
#define FW_METADATA_RESET  (1U << 7)
#define FW_UPLOAD_ABORTED  (1U << 8)
#define FW_WARNINGS (FW_FALLBACK | FW_UPLOAD_FAILED | FW_NO_VALID_IMAGE | FW_METADATA_RESET | FW_UPLOAD_ABORTED)

enum { FW_OK, FW_BAD_REQUEST, FW_BUSY, FW_IO_ERROR, FW_BAD_IMAGE,
       FW_BAD_HASH, FW_BAD_OFFSET, FW_NOT_READY, FW_TRIAL_FAILED };
typedef struct {
    void *context;
    int (*read)(void *, uint32_t, void *, uint32_t);
    int (*write)(void *, uint32_t, const void *, uint32_t);
    int (*erase)(void *, uint32_t);
} fw_io_t;
typedef struct { uint32_t size, version; uint8_t sha256[32]; } fw_image_t;
struct fw_history;
typedef struct {
    uint32_t generation, confirmed, pending, trial, flags, last_error;
    fw_image_t image[2];
} fw_metadata_t;
typedef struct {
    fw_io_t io;
    fw_metadata_t meta;
    int record;
    bool uploading;
    uint32_t target, received;
    fw_image_t incoming;
    struct fw_history *history; /* Optional diagnostic journal; never boot authority. */
} fw_update_t;

/** @brief Compute the reflected IEEE CRC-32 of a byte sequence. */
uint32_t fw_crc32(const void *data, size_t size);
/** @brief Compute the SHA-256 digest of a complete byte sequence. */
void fw_sha256(const void *data, size_t size, uint8_t digest[32]);
/** @brief Return the flash offset of application slot A or B. */
uint32_t fw_slot_address(uint32_t slot);
/** @brief Load the newest valid metadata copy or initialize an uncommitted update state. */
int fw_open(fw_update_t *fw, const fw_io_t *io);
/** @brief Commit metadata with single-program ECC units, preserving the previous valid copy. */
int fw_store(fw_update_t *fw, const fw_metadata_t *meta);
/** @brief Verify an image's header, vectors, checksum, and optional recorded SHA-256 digest. */
int fw_validate(fw_update_t *fw, uint32_t slot, fw_image_t *image, bool compare);
/** @brief Select a validated image, persist trial state, and roll back failed trials with a warning. */
int fw_boot_select(fw_update_t *fw, uint32_t *slot, bool *trial);
/** @brief Invalidate and erase the inactive slot before starting a bounded image upload. */
int fw_begin(fw_update_t *fw, uint32_t size, const uint8_t sha256[32], uint32_t version);
/** @brief Validate, program, and read back the next sequential upload chunk. */
int fw_data(fw_update_t *fw, uint32_t offset, const uint8_t *data, uint32_t size, uint32_t crc);
/** @brief Verify the complete uploaded image and commit it as pending for trial boot. */
int fw_finish(fw_update_t *fw);
/** @brief Cancel an unfinished upload and persist the aborted flag. */
int fw_abort(fw_update_t *fw);
/** @brief Promote a validated trial image to the confirmed boot slot. */
int fw_confirm(fw_update_t *fw, uint32_t slot);
/** @brief Clear sticky update warnings and the recorded error in persistent metadata. */
int fw_clear_warnings(fw_update_t *fw);
#endif
