#include "fw_history.h"
#include <string.h>

static uint32_t history_get(const uint8_t *p)
{ return p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static void history_put(uint8_t *p,uint32_t value)
{ for(unsigned i=0;i<4;++i)p[i]=(uint8_t)(value>>(8*i)); }
static bool history_erased(const uint8_t *p)
{ for(unsigned i=0;i<FW_HISTORY_RECORD_SIZE;++i)if(p[i]!=0xFFU)return false;return true; }
static bool history_valid(const uint8_t *p)
{
    return history_get(p)==0x4853474DU && history_get(p+4)==1U &&
        history_get(p+8)==FW_HISTORY_RECORD_SIZE && history_get(p+124)==0x54494D43U &&
        history_get(p+120)==fw_crc32(p,120);
}
static uint32_t history_address(uint32_t index)
{ return FW_HISTORY_BASE+index*FW_HISTORY_RECORD_SIZE; }
static int history_error(fw_history_t *h)
{ h->ready=false;h->last_error=FW_IO_ERROR;return FW_IO_ERROR; }

int fw_history_open(fw_history_t *h,const fw_io_t *io,uint32_t (*uptime_ms)(void))
{
    memset(h,0,sizeof(*h));h->io=*io;h->uptime_ms=uptime_ms;h->newest=FW_NONE;
    /* Read full pages rather than issuing one XSPI transaction per record. */
    uint8_t block[512];
    for(uint32_t pos=0;pos<FW_HISTORY_SIZE;pos+=sizeof(block)){
        if(io->read(io->context,FW_HISTORY_BASE+pos,block,sizeof(block)))return history_error(h);
        for(uint32_t i=0;i<sizeof(block);i+=FW_HISTORY_RECORD_SIZE){
            uint8_t *r=block+i;uint32_t sequence=history_get(r+12);
            if(history_valid(r) && (h->newest==FW_NONE || (int32_t)(sequence-h->sequence)>0)){
                h->newest=(pos+i)/FW_HISTORY_RECORD_SIZE;h->sequence=sequence;
            }
        }
    }
    h->next=h->newest==FW_NONE?0:(h->newest+1U)%FW_HISTORY_CAPACITY;
    /* Reserve every visibly touched record after the last commit. Never reuse
     * a partially programmed ECC unit, even if its commit was interrupted.
     * At a sector boundary append erases first, also completing torn erases. */
    while(h->next%(FW_ERASE_SIZE/FW_HISTORY_RECORD_SIZE)){
        uint8_t r[FW_HISTORY_RECORD_SIZE];
        if(io->read(io->context,history_address(h->next),r,sizeof(r)))return history_error(h);
        if(history_erased(r))break;
        h->next=(h->next+1U)%FW_HISTORY_CAPACITY;
    }
    h->ready=h->readable=true;return FW_OK;
}

int fw_history_record(fw_history_t *h,uint32_t event,uint32_t target,uint32_t result,
                      uint32_t generation,uint32_t flags,const fw_image_t *image)
{
    if(!h)return FW_OK;
    /* After a failed append, do not retry until reopen has inspected the NOR. */
    if(!h->ready)return (int)h->last_error;
    uint32_t address=history_address(h->next);
    if(!(h->next%(FW_ERASE_SIZE/FW_HISTORY_RECORD_SIZE)) &&
       h->io.erase(h->io.context,address))return history_error(h);
    uint8_t r[FW_HISTORY_RECORD_SIZE]={0},check[FW_HISTORY_RECORD_SIZE];
    history_put(r,0x4853474DU);history_put(r+4,1);history_put(r+8,sizeof(r));
    history_put(r+12,h->sequence+1U);history_put(r+16,event);history_put(r+20,target);
    history_put(r+24,result);history_put(r+28,generation);history_put(r+32,flags);
    history_put(r+36,h->uptime_ms?h->uptime_ms():FW_NONE);
    if(image){history_put(r+40,image->size);history_put(r+44,image->version);memcpy(r+48,image->sha256,32);}
    history_put(r+120,fw_crc32(r,120));history_put(r+124,0x54494D43U);
    /* Prefix and final CRC/commit occupy disjoint 16-byte ECC units. */
    if(h->io.write(h->io.context,address,r,112) ||
       h->io.read(h->io.context,address,check,112) || memcmp(r,check,112) ||
       h->io.write(h->io.context,address+112,r+112,16) ||
       h->io.read(h->io.context,address,check,sizeof(check)) || memcmp(r,check,sizeof(r)))return history_error(h);
    h->newest=h->next;h->next=(h->next+1U)%FW_HISTORY_CAPACITY;++h->sequence;
    return FW_OK;
}

int fw_history_page(fw_history_t *h,uint32_t cursor,uint8_t *records,uint32_t *count,uint32_t *next)
{
    *count=0;*next=FW_NONE;
    if(cursor!=FW_NONE && cursor>=FW_HISTORY_CAPACITY)return FW_BAD_REQUEST;
    if(!h->readable)return (int)h->last_error;
    if(h->newest==FW_NONE)return FW_OK;
    uint32_t index=cursor==FW_NONE?h->newest:cursor;
    do {
        uint8_t r[FW_HISTORY_RECORD_SIZE];
        if(h->io.read(h->io.context,history_address(index),r,sizeof(r))){
            h->last_error=FW_IO_ERROR;*count=0;return FW_IO_ERROR;
        }
        /* A failed final readback may have left a committed newer record. Keep
         * this traversal ordered against the last known head until reopen. */
        if(history_valid(r) && (int32_t)(h->sequence-history_get(r+12))>=0){
            memcpy(records+(*count)*sizeof(r),r,sizeof(r));++*count;
        }
        index=(index+FW_HISTORY_CAPACITY-1U)%FW_HISTORY_CAPACITY;
        if(index==h->newest)return FW_OK;
    } while(*count<FW_HISTORY_PAGE);
    *next=index;return FW_OK;
}
