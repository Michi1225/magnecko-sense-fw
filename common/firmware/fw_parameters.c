#include "fw_parameters.h"
#include <string.h>

#define PARAMS_MAGIC 0x31524A50U
#define PARAMS_COMMITTED 0x314B4F50U
static uint32_t params_u32(const uint8_t *p)
{ return p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static void params_put(uint8_t *p,uint32_t v)
{ for(unsigned i=0;i<4;++i)p[i]=(uint8_t)(v>>(8*i)); }

int fw_parameters_load(const fw_io_t *io,uint8_t *data,uint32_t *size,bool *journal)
{
    uint8_t header[16];*journal=false;
    if(io->read(io->context,FW_PARAMETERS_COMMIT,header,sizeof(header)))return FW_IO_ERROR;
    if(params_u32(header)==PARAMS_MAGIC && params_u32(header+12)==PARAMS_COMMITTED){
        uint32_t n=params_u32(header+4);
        if(n<3 || n>FW_PARAMETERS_MAX)return FW_BAD_IMAGE;
        if(io->read(io->context,FW_PARAMETERS_JOURNAL,data,n))return FW_IO_ERROR;
        if(fw_crc32(data,n)!=params_u32(header+8))return FW_BAD_HASH;
        if(data[n-1] || memchr(data,0,n-1))return FW_BAD_IMAGE;
        *journal=true;*size=n;return FW_OK;
    }
    if(io->read(io->context,0,data,FW_PARAMETERS_MAX))return FW_IO_ERROR;
    uint8_t *end=memchr(data,0,FW_PARAMETERS_MAX);
    if(!end || end==data)return FW_BAD_IMAGE;
    *size=(uint32_t)(end-data)+1U;return FW_OK;
}
/* Each physical 16-byte ECC unit is programmed only once per erase. */
static int params_write(const fw_io_t *io,uint32_t base,const uint8_t *data,uint32_t size)
{
    for(uint32_t off=0;off<FW_PARAMETERS_MAX;off+=FW_PARAMETERS_SECTOR)
        if(io->erase(io->context,base+off))return FW_IO_ERROR;
    for(uint32_t off=0;off<size;){
        uint8_t page[FW_CHUNK_SIZE],check[FW_CHUNK_SIZE];
        uint32_t n=size-off;if(n>sizeof(page))n=sizeof(page);
        uint32_t padded=(n+15U)&~15U;memset(page,255,padded);memcpy(page,data+off,n);
        if(io->write(io->context,base+off,page,padded) ||
           io->read(io->context,base+off,check,padded) || memcmp(page,check,padded))return FW_IO_ERROR;
        off+=n;
    }
    return FW_OK;
}
int fw_parameters_recover(const fw_io_t *io,uint8_t *scratch)
{
    uint32_t size;bool journal;
    int result=fw_parameters_load(io,scratch,&size,&journal);
    if(result || !journal)return result;
    result=params_write(io,0,scratch,size);if(result)return result;
    return io->erase(io->context,FW_PARAMETERS_COMMIT)?FW_IO_ERROR:FW_OK;
}
void fw_parameters_open(fw_parameters_t *p,const fw_io_t *io)
{
    memset(p,0,offsetof(fw_parameters_t,incoming));p->io=*io;
}
int fw_parameters_begin(fw_parameters_t *p,uint32_t size,uint32_t target,uint32_t image_size,
                        const uint8_t image_sha256[32],const uint8_t json_sha256[32])
{
    if(p->state==FW_PARAMETERS_RECEIVING || p->state==FW_PARAMETERS_READY)return FW_BUSY;
    if(!size || size>=FW_PARAMETERS_MAX || target>FW_PARAMETERS_FSBL || !image_size)return FW_BAD_REQUEST;
    p->total=size;p->target=target;p->image_size=image_size;p->received=0;p->size=0;p->error=FW_OK;
    memcpy(p->image_sha256,image_sha256,32);memcpy(p->json_sha256,json_sha256,32);
    p->state=FW_PARAMETERS_RECEIVING;return FW_OK;
}
int fw_parameters_data(fw_parameters_t *p,uint32_t offset,const uint8_t *data,uint32_t size,uint32_t crc)
{
    if(p->state!=FW_PARAMETERS_RECEIVING)return FW_NOT_READY;
    if(offset!=p->received || !size || size>FW_CHUNK_SIZE || size>p->total-p->received)return FW_BAD_OFFSET;
    if(fw_crc32(data,size)!=crc)return FW_BAD_HASH;
    memcpy(p->incoming+offset,data,size);p->received+=size;return FW_OK;
}
int fw_parameters_finish(fw_parameters_t *p)
{
    if(p->state!=FW_PARAMETERS_RECEIVING || p->received!=p->total)return FW_NOT_READY;
    uint8_t digest[32];fw_sha256(p->incoming,p->total,digest);
    int result=FW_OK;
    if(memcmp(digest,p->json_sha256,32))result=FW_BAD_HASH;
    else if(memchr(p->incoming,0,p->total))result=FW_BAD_IMAGE;
    else {
        p->incoming[p->total]=0;uint32_t size;bool journal;
        result=fw_parameters_load(&p->io,p->previous,&size,&journal);
        if(!result)result=fw_parameters_merge(p->previous,p->incoming,p->target,p->image_size,p->image_sha256,p->merged,&p->size);
    }
    p->error=(uint32_t)result;if(!result)p->state=FW_PARAMETERS_READY;
    return result;
}
int fw_parameters_apply(fw_parameters_t *p,uint32_t target,const fw_image_t *installed)
{
    if(p->state!=FW_PARAMETERS_READY && p->state!=FW_PARAMETERS_APPLIED)return FW_NOT_READY;
    if(target!=p->target || installed->size!=p->image_size || memcmp(installed->sha256,p->image_sha256,32))return FW_BAD_HASH;
    if(p->state==FW_PARAMETERS_APPLIED)return FW_OK;
    /* A previous interrupted commit must reach canonical storage before its
     * journal can be reused. A failed read never permits an erase. */
    int result=fw_parameters_recover(&p->io,p->previous);if(result)goto done;
    result=params_write(&p->io,FW_PARAMETERS_JOURNAL,p->merged,p->size);if(result)goto done;
    uint8_t header[16],check[16];
    params_put(header,PARAMS_MAGIC);params_put(header+4,p->size);
    params_put(header+8,fw_crc32(p->merged,p->size));params_put(header+12,PARAMS_COMMITTED);
    if(p->io.erase(p->io.context,FW_PARAMETERS_COMMIT) ||
       p->io.write(p->io.context,FW_PARAMETERS_COMMIT,header,sizeof(header)) ||
       p->io.read(p->io.context,FW_PARAMETERS_COMMIT,check,sizeof(check)) || memcmp(header,check,sizeof(header))){
        result=FW_IO_ERROR;goto done;
    }
    result=params_write(&p->io,0,p->merged,p->size);if(result)goto done;
    if(p->io.erase(p->io.context,FW_PARAMETERS_COMMIT)){result=FW_IO_ERROR;goto done;}
    p->state=FW_PARAMETERS_APPLIED;
done:
    p->error=(uint32_t)result;return result;
}
void fw_parameters_abort(fw_parameters_t *p)
{ p->state=FW_PARAMETERS_IDLE;p->received=p->total=p->error=p->size=0; }
