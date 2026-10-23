#include "fw_fsbl.h"
#include "fw_history.h"
#include <string.h>
static void fsbl_log(fw_fsbl_t *f,uint32_t event,int result)
{ (void)fw_history_record(f->history,event,FW_HISTORY_FSBL,(uint32_t)result,0,0,&f->incoming); }
static uint32_t fsbl_u32(const uint8_t *p)
{ return p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static bool fsbl_size(uint32_t size)
{ return size>=FW_VECTOR_OFFSET+8U && size<=FW_FSBL_MAX && !(size&3U); }
int fw_fsbl_validate(const uint8_t *p,uint32_t size,fw_image_t *image)
{
    if(!fsbl_size(size) || fsbl_u32(p)!=0x324d5453U || fsbl_u32(p+0x68)!=0x20300U ||
       fsbl_u32(p+0x6c)!=size-0x240U)return FW_BAD_IMAGE;
    uint32_t sp=fsbl_u32(p+FW_VECTOR_OFFSET),reset=fsbl_u32(p+FW_VECTOR_OFFSET+4U),entry=reset&~1U;
    if(sp<=0x341C0000U || sp>0x34200000U || (sp&7U) || !(reset&1U) ||
       entry<FW_FSBL_RAM+FW_VECTOR_OFFSET || entry>=FW_FSBL_RAM+size ||
       fsbl_u32(p+0x70)!=reset)return FW_BAD_IMAGE;
    uint32_t sum=0;
    for(uint32_t i=0x240U;i<size;++i)sum+=p[i];
    if(sum!=fsbl_u32(p+0x64))return FW_BAD_IMAGE;
    *image=(fw_image_t){.size=size};fw_sha256(p,size,image->sha256);return FW_OK;
}
static int fsbl_read_active(fw_fsbl_t *f)
{
    uint8_t head[128];
    if(f->io.read(f->io.context,FW_FSBL_BASE,head,sizeof(head)))return FW_IO_ERROR;
    uint32_t payload=fsbl_u32(head+0x6c);
    if(payload>FW_FSBL_MAX-0x240U || !fsbl_size(payload+0x240U))return FW_BAD_IMAGE;
    uint32_t size=payload+0x240U;
    if(f->io.read(f->io.context,FW_FSBL_BASE,f->backup,size))return FW_IO_ERROR;
    int result=fw_fsbl_validate(f->backup,size,&f->backup_image);
    if(result==FW_OK)f->active=f->backup_image;
    return result;
}
int fw_fsbl_open(fw_fsbl_t *f,const fw_io_t *io)
{
    memset(f,0,offsetof(fw_fsbl_t,staging));f->io=*io;
    f->error=(uint32_t)fsbl_read_active(f);
    if(f->error)f->state=FW_FSBL_RECOVERY;
    return (int)f->error;
}
int fw_fsbl_begin(fw_fsbl_t *f,uint32_t size,const uint8_t sha256[32])
{
    if(f->state!=FW_FSBL_IDLE || !f->active.size)return FW_NOT_READY;
    if(!fsbl_size(size))return FW_BAD_IMAGE;
    f->incoming=(fw_image_t){.size=size};memcpy(f->incoming.sha256,sha256,32);
    f->received=0;f->error=FW_OK;f->state=FW_FSBL_RECEIVING;
    fsbl_log(f,FW_HISTORY_UPLOAD_STARTED,FW_OK);return FW_OK;
}
int fw_fsbl_data(fw_fsbl_t *f,uint32_t offset,const uint8_t *p,uint32_t size,uint32_t crc)
{
    if(f->state!=FW_FSBL_RECEIVING)return FW_NOT_READY;
    if(offset!=f->received || !size || size>FW_CHUNK_SIZE || (size&3U) ||
       size>f->incoming.size-f->received)return FW_BAD_OFFSET;
    if(fw_crc32(p,size)!=crc)return FW_BAD_HASH;
    memcpy(f->staging+offset,p,size);f->received+=size;return FW_OK;
}
int fw_fsbl_finish(fw_fsbl_t *f)
{
    if(f->state!=FW_FSBL_RECEIVING || f->received!=f->incoming.size)return FW_NOT_READY;
    fw_image_t image;
    int result=fw_fsbl_validate(f->staging,f->incoming.size,&image);
    if(result==FW_OK && memcmp(image.sha256,f->incoming.sha256,32))result=FW_BAD_HASH;
    f->error=(uint32_t)result;
    if(result==FW_OK)f->state=FW_FSBL_READY;
    fsbl_log(f,result?FW_HISTORY_FAILED:FW_HISTORY_VERIFIED,result);
    return result;
}
/* Program the first 16-byte ECC unit last, after verifying the rest. An
 * incomplete body has no valid STM32 magic/header. This is not power-fail A/B. */
static int fsbl_program(fw_fsbl_t *f,const uint8_t *data,uint32_t size)
{
    uint8_t check[FW_CHUNK_SIZE];
    if(f->io.erase(f->io.context,FW_FSBL_BASE))return FW_IO_ERROR;
    for(uint32_t pos=16;pos<size;){
        uint32_t n=size-pos;if(n>sizeof(check))n=sizeof(check);
        if(f->io.write(f->io.context,FW_FSBL_BASE+pos,data+pos,n) ||
           f->io.read(f->io.context,FW_FSBL_BASE+pos,check,n) || memcmp(check,data+pos,n))return FW_IO_ERROR;
        pos+=n;
    }
    if(f->io.write(f->io.context,FW_FSBL_BASE,data,16))return FW_IO_ERROR;
    for(uint32_t pos=0;pos<size;){
        uint32_t n=size-pos;if(n>sizeof(check))n=sizeof(check);
        if(f->io.read(f->io.context,FW_FSBL_BASE+pos,check,n) || memcmp(check,data+pos,n))return FW_IO_ERROR;
        pos+=n;
    }
    return FW_OK;
}
int fw_fsbl_apply(fw_fsbl_t *f,const uint8_t expected[32])
{
    if(memcmp(expected,f->incoming.sha256,32))return FW_BAD_HASH;
    if(f->state==FW_FSBL_INSTALLED)return FW_OK;
    if(f->state!=FW_FSBL_READY && f->state!=FW_FSBL_RECOVERY)return FW_NOT_READY;
    fw_image_t image;
    int result=fw_fsbl_validate(f->staging,f->incoming.size,&image);
    if(result || memcmp(image.sha256,expected,32))return result?result:FW_BAD_HASH;
    if(f->state==FW_FSBL_READY){
        result=fsbl_read_active(f);
        if(result)return result; /* No erase without a validated RAM backup. */
    }
    fsbl_log(f,FW_HISTORY_FSBL_APPLY,FW_OK);
    result=fsbl_program(f,f->staging,image.size);
    if(result==FW_OK){f->active=image;f->state=FW_FSBL_INSTALLED;f->error=FW_OK;fsbl_log(f,FW_HISTORY_FSBL_INSTALLED,FW_OK);return FW_OK;}
    f->error=FW_IO_ERROR;
    if(f->backup_image.size && fsbl_program(f,f->backup,f->backup_image.size)==FW_OK){
        f->active=f->backup_image;f->state=FW_FSBL_READY;
        fsbl_log(f,FW_HISTORY_FSBL_RESTORED,FW_IO_ERROR);
    } else {f->active=(fw_image_t){0};f->state=FW_FSBL_RECOVERY;fsbl_log(f,FW_HISTORY_FSBL_RECOVERY,FW_IO_ERROR);}
    return FW_IO_ERROR;
}
int fw_fsbl_abort(fw_fsbl_t *f)
{
    if(f->state==FW_FSBL_RECOVERY || f->state==FW_FSBL_INSTALLED)return FW_NOT_READY;
    if(f->state!=FW_FSBL_IDLE)fsbl_log(f,FW_HISTORY_ABORTED,FW_OK);
    f->state=FW_FSBL_IDLE;f->received=0;f->error=FW_OK;f->incoming=(fw_image_t){0};return FW_OK;
}
