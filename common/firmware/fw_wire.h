#ifndef FW_WIRE_H
#define FW_WIRE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define FW_WIRE_MAX 536U
#define FW_STATUS_SIZE 64U
#define FW_FSBL_STATUS_SIZE 156U
#define FW_PARAMETERS_STATUS_SIZE 176U
#define FW_HISTORY_REPLY_SIZE (40U + 3U * 128U)
#define FW_DFU_PORT 50003U
enum { FW_OP_STATUS=1, FW_OP_BEGIN, FW_OP_DATA, FW_OP_FINISH, FW_OP_ABORT, FW_OP_RESET, FW_OP_CLEAR };
enum { FW_OP_FSBL_STATUS=8, FW_OP_FSBL_BEGIN, FW_OP_FSBL_DATA, FW_OP_FSBL_FINISH,
       FW_OP_FSBL_APPLY, FW_OP_FSBL_ABORT };
enum { FW_OP_PARAMETERS_STATUS=14, FW_OP_PARAMETERS_BEGIN, FW_OP_PARAMETERS_DATA,
       FW_OP_PARAMETERS_FINISH, FW_OP_PARAMETERS_ABORT, FW_OP_PARAMETERS_APPLY };
enum { FW_OP_HISTORY=20 };
typedef struct { uint8_t frame[FW_WIRE_MAX]; uint32_t used; } fw_parser_t;
typedef void (*fw_frame_fn)(void *,const uint8_t *,uint32_t);
/** @brief Reassemble bounded DFUC and CTRL frames and dispatch complete requests. */
void fw_wire_feed(fw_parser_t *,const uint8_t *,size_t,fw_frame_fn,void *);
#endif
