#include "fw_update.h"
#include "fw_history.h"
#include <string.h>

/* Portable update policy. All flash offsets and lengths are checked here;
 * neither the application nor an incoming packet supplies a physical address. */
/** @brief Decode a little-endian 32-bit image or metadata field. */
static uint32_t fw_get32(const uint8_t *p)
{ return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
/** @brief Encode a little-endian 32-bit image or metadata field. */
static void fw_put32(uint8_t *p, uint32_t n)
{ for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(n >> (8 * i)); }
/** @brief Compute the reflected IEEE CRC-32 of a byte sequence. */
uint32_t fw_crc32(const void *data, size_t size)
{
    const uint8_t *p = data;
    uint32_t c = UINT32_MAX;
    while (size--) {
        c ^= *p++;
        for (unsigned i = 0; i < 8; ++i) c = (c >> 1) ^ (0xEDB88320U & (0U - (c & 1U)));
    }
    return ~c;
}
typedef struct { uint32_t h[8]; uint64_t bytes; uint8_t block[64]; unsigned used; } fw_sha_t;
/** @brief Rotate a 32-bit word right for SHA-256 compression. */
static uint32_t fw_rotr(uint32_t x, unsigned n) { return (x >> n) | (x << (32 - n)); }
/** @brief Compress one 64-byte block into the SHA-256 state. */
static void fw_sha_block(fw_sha_t *s, const uint8_t *p)
{
    static const uint32_t k[64] = {
        0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
        0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
        0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
        0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
        0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
        0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
        0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
        0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
    uint32_t w[64];
    for (unsigned i = 0; i < 16; ++i)
        w[i] = ((uint32_t)p[4*i] << 24) | ((uint32_t)p[4*i+1] << 16) | ((uint32_t)p[4*i+2] << 8) | p[4*i+3];
    for (unsigned i = 16; i < 64; ++i) {
        uint32_t a=w[i-15], b=w[i-2];
        w[i]=w[i-16]+(fw_rotr(a,7)^fw_rotr(a,18)^(a>>3))+w[i-7]+(fw_rotr(b,17)^fw_rotr(b,19)^(b>>10));
    }
    uint32_t a=s->h[0],b=s->h[1],c=s->h[2],d=s->h[3],e=s->h[4],f=s->h[5],g=s->h[6],h=s->h[7];
    for (unsigned i=0;i<64;++i) {
        uint32_t t=h+(fw_rotr(e,6)^fw_rotr(e,11)^fw_rotr(e,25))+((e&f)^((~e)&g))+k[i]+w[i];
        uint32_t u=(fw_rotr(a,2)^fw_rotr(a,13)^fw_rotr(a,22))+((a&b)^(a&c)^(b&c));
        h=g;g=f;f=e;e=d+t;d=c;c=b;b=a;a=t+u;
    }
    s->h[0]+=a;s->h[1]+=b;s->h[2]+=c;s->h[3]+=d;s->h[4]+=e;s->h[5]+=f;s->h[6]+=g;s->h[7]+=h;
}
/** @brief Initialize a SHA-256 hash context. */
static void fw_sha_init(fw_sha_t *s)
{
    *s=(fw_sha_t){.h={0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19}};
}
/** @brief Accumulate bytes into the SHA-256 context and compress complete blocks. */
static void fw_sha_add(fw_sha_t *s,const uint8_t *p,size_t n)
{
    s->bytes+=n;
    while(n) {
        size_t take=64-s->used;if(take>n)take=n;
        memcpy(s->block+s->used,p,take);s->used+=(unsigned)take;p+=take;n-=take;
        if(s->used==64){fw_sha_block(s,s->block);s->used=0;}
    }
}
/** @brief Pad the SHA-256 message and emit its 32-byte digest. */
static void fw_sha_end(fw_sha_t *s,uint8_t out[32])
{
    uint64_t bits=s->bytes*8;
    uint8_t pad[128]={0x80};
    unsigned n=s->used<56 ? 56-s->used : 120-s->used;
    for(unsigned i=0;i<8;++i)pad[n+i]=(uint8_t)(bits>>(56-8*i));
    fw_sha_add(s,pad,n+8);
    for(unsigned i=0;i<32;++i)out[i]=(uint8_t)(s->h[i/4]>>(24-8*(i%4)));
}
/** @brief Compute the SHA-256 digest of a complete byte sequence. */
void fw_sha256(const void *data,size_t size,uint8_t digest[32])
{ fw_sha_t s;fw_sha_init(&s);fw_sha_add(&s,data,size);fw_sha_end(&s,digest); }
/** @brief Return the flash offset of application slot A or B. */
uint32_t fw_slot_address(uint32_t slot) { return slot==0 ? FW_SLOT_A : FW_SLOT_B; }

/** @brief Validate a committed metadata record and decode its slot descriptors. */
static bool fw_decode(const uint8_t r[FW_RECORD_SIZE],fw_metadata_t *m)
{
    if(fw_get32(r)!=0x42415746U || fw_get32(r+4)!=1U || fw_get32(r+8)!=FW_RECORD_SIZE ||
       fw_get32(r+116)!=fw_crc32(r,116) || fw_get32(r+120)!=0x54494d43U) return false;
    m->generation=fw_get32(r+12);m->confirmed=fw_get32(r+16);m->pending=fw_get32(r+20);
    m->trial=fw_get32(r+24);m->flags=fw_get32(r+28);m->last_error=fw_get32(r+32);
    if(m->confirmed>1 || (m->pending!=FW_NONE && m->pending>1) || m->pending==m->confirmed ||
       m->trial>1 || (m->pending==FW_NONE && m->trial))return false;
    for(unsigned i=0;i<2;++i){
        m->image[i].size=fw_get32(r+36+40*i);m->image[i].version=fw_get32(r+40+40*i);
        memcpy(m->image[i].sha256,r+44+40*i,32);
        if(m->image[i].size>FW_MAX_IMAGE)return false;
    }
    return true;
}
/** @brief Serialize update metadata with its CRC and final commit marker. */
static void fw_encode(uint8_t r[FW_RECORD_SIZE],const fw_metadata_t *m)
{
    memset(r,0,FW_RECORD_SIZE);fw_put32(r,0x42415746U);fw_put32(r+4,1);fw_put32(r+8,FW_RECORD_SIZE);
    fw_put32(r+12,m->generation);fw_put32(r+16,m->confirmed);fw_put32(r+20,m->pending);
    fw_put32(r+24,m->trial);fw_put32(r+28,m->flags);fw_put32(r+32,m->last_error);
    for(unsigned i=0;i<2;++i){fw_put32(r+36+40*i,m->image[i].size);fw_put32(r+40+40*i,m->image[i].version);
        memcpy(r+44+40*i,m->image[i].sha256,32);}
    fw_put32(r+116,fw_crc32(r,116));fw_put32(r+120,0x54494d43U);
}
/** @brief Load the newest valid metadata copy or initialize an uncommitted update state. */
int fw_open(fw_update_t *f,const fw_io_t *io)
{
    memset(f,0,sizeof(*f));f->io=*io;f->record=-1;f->meta.pending=FW_NONE;
    uint8_t r[FW_RECORD_SIZE];fw_metadata_t m;
    for(int i=0;i<2;++i){
        if(io->read(io->context,i?FW_META_B:FW_META_A,r,sizeof(r)))return FW_IO_ERROR;
        if(fw_decode(r,&m) && (f->record<0 || (int32_t)(m.generation-f->meta.generation)>0)){f->meta=m;f->record=i;}
    }
    return FW_OK;
}
/** @brief Commit replacement metadata while preserving the previous valid copy until completion. */
int fw_store(fw_update_t *f,const fw_metadata_t *next)
{
    fw_metadata_t m=*next;m.generation=f->meta.generation+1U;
    uint8_t r[FW_RECORD_SIZE],verify[FW_RECORD_SIZE];fw_encode(r,&m);
    int dest=f->record==0?1:0;uint32_t address=dest?FW_META_B:FW_META_A;
    /* SEMPER uses 16-byte ECC units. Writing bytes 0..119 followed by the
     * commit word at 120 programs unit 112..127 twice, which 2-bit ECC rejects.
     * Keep the existing 124-byte format: verify 0..111 first, then program the
     * last digest word, CRC and commit together in the untouched final unit.
     * The previous valid sector survives every erase/program/readback step. */
    const uint32_t tail=112U;
    if(f->io.erase(f->io.context,address) || f->io.write(f->io.context,address,r,tail) ||
       f->io.read(f->io.context,address,verify,tail) || memcmp(r,verify,tail) ||
       f->io.write(f->io.context,address+tail,r+tail,FW_RECORD_SIZE-tail) ||
       f->io.read(f->io.context,address,verify,sizeof(verify)) || memcmp(r,verify,sizeof(r)))return FW_IO_ERROR;
    f->meta=m;f->record=dest;return FW_OK;
}
static void fw_log(fw_update_t *f,uint32_t event,uint32_t target,int result,const fw_image_t *image)
{ (void)fw_history_record(f->history,event,target,(uint32_t)result,f->meta.generation,f->meta.flags,image); }
/** @brief Verify an image's header, vectors, checksum, and optional recorded SHA-256 digest. */
int fw_validate(fw_update_t *f,uint32_t slot,fw_image_t *image,bool compare)
{
    if(slot>1)return FW_BAD_REQUEST;
    uint32_t base=fw_slot_address(slot);uint8_t head[128],vectors[8],buffer[256];
    if(f->io.read(f->io.context,base,head,sizeof(head)))return FW_IO_ERROR;
    uint32_t payload=fw_get32(head+0x6c);
    if(fw_get32(head)!=0x324d5453U || fw_get32(head+0x68)!=0x00020300U ||
       payload<FW_VECTOR_OFFSET-0x240U+8 || payload>FW_MAX_IMAGE-0x240U)return FW_BAD_IMAGE;
    uint32_t size=0x240U+payload;
    if((size&3U) || (compare && image->size!=size))return FW_BAD_IMAGE;
    if(f->io.read(f->io.context,base+FW_VECTOR_OFFSET,vectors,8))return FW_IO_ERROR;
    uint32_t sp=fw_get32(vectors),reset=fw_get32(vectors+4),entry=reset&~1U;
    if(sp<=0x34080000U || sp>0x34200000U || (sp&7U) || !(reset&1U) ||
       entry<FW_RAM_BASE+FW_VECTOR_OFFSET || entry>=FW_RAM_BASE+size || fw_get32(head+0x70)!=reset)return FW_BAD_IMAGE;
    fw_sha_t sha;fw_sha_init(&sha);uint32_t sum=0;
    for(uint32_t pos=0;pos<size;){
        uint32_t n=size-pos;if(n>sizeof(buffer))n=sizeof(buffer);
        if(f->io.read(f->io.context,base+pos,buffer,n))return FW_IO_ERROR;
        fw_sha_add(&sha,buffer,n);
        for(uint32_t i=0;i<n;++i)if(pos+i>=0x240U)sum+=buffer[i];
        pos+=n;
    }
    uint8_t digest[32];fw_sha_end(&sha,digest);
    if(sum!=fw_get32(head+0x64))return FW_BAD_IMAGE;
    if(compare && memcmp(digest,image->sha256,32))return FW_BAD_HASH;
    image->size=size;memcpy(image->sha256,digest,32);return FW_OK;
}
/** @brief Verify a slot against its stored image size and digest. */
static int fw_slot_valid(fw_update_t *f,uint32_t slot)
{
    if(slot>1 || !f->meta.image[slot].size)return FW_BAD_IMAGE;
    fw_image_t image=f->meta.image[slot];return fw_validate(f,slot,&image,true);
}
/** @brief Select a validated image, persist trial state, and roll back failed trials with a warning. */
int fw_boot_select(fw_update_t *f,uint32_t *slot,bool *trial)
{
    *trial=false;
    fw_metadata_t m;
    if(f->record<0){
        m=f->meta;bool found=false;
        /* Migration: existing v2.3 slot A/B images have no update metadata yet.
         * Adopt only images that pass all header/vector/checksum checks. */
        for(unsigned i=0;i<2;++i)if(fw_validate(f,i,&m.image[i],false)==FW_OK){if(!found)m.confirmed=i;found=true;}
        if(!found){fw_log(f,FW_HISTORY_NO_IMAGE,FW_NONE,FW_BAD_IMAGE,NULL);return FW_BAD_IMAGE;}
        m.flags=FW_CONFIRMED|FW_METADATA_RESET;
        if(fw_store(f,&m))return FW_IO_ERROR;
        fw_log(f,FW_HISTORY_ADOPTED,m.confirmed,FW_OK,&m.image[m.confirmed]);
    }
    if(f->meta.flags&FW_UPLOADING){
        m=f->meta;
        m.flags=(m.flags&~FW_UPLOADING)|FW_UPLOAD_FAILED;m.last_error=FW_NOT_READY;
        if(fw_store(f,&m))return FW_IO_ERROR;
        fw_log(f,FW_HISTORY_FAILED,m.confirmed^1U,FW_NOT_READY,NULL);
    }
    if(f->meta.pending!=FW_NONE){
        m=f->meta;uint32_t candidate=m.pending;
        int result=m.trial?FW_TRIAL_FAILED:fw_slot_valid(f,candidate);
        if(result==FW_OK){
            m.trial=1;
            if(fw_store(f,&m))return FW_IO_ERROR;
            fw_log(f,FW_HISTORY_TRIAL,candidate,FW_OK,&m.image[candidate]);
            *slot=candidate;*trial=true;return FW_OK;
        }
        fw_image_t rejected=m.image[candidate];
        m.image[candidate].size=0;m.pending=FW_NONE;m.trial=0;
        m.flags=(m.flags & ~FW_CONFIRMED)|FW_FALLBACK;m.last_error=result;
        if(fw_store(f,&m))return FW_IO_ERROR;
        fw_log(f,FW_HISTORY_ROLLBACK,candidate,result,&rejected);
    }
    uint32_t selected=f->meta.confirmed;
    if(fw_slot_valid(f,selected)!=FW_OK){
        uint32_t other=selected^1U;
        if(fw_slot_valid(f,other)!=FW_OK){
            m=f->meta;m.flags|=FW_NO_VALID_IMAGE;m.last_error=FW_BAD_IMAGE;
            (void)fw_store(f,&m);fw_log(f,FW_HISTORY_NO_IMAGE,FW_NONE,FW_BAD_IMAGE,NULL);return FW_BAD_IMAGE;
        }
        fw_image_t rejected=f->meta.image[selected];
        m=f->meta;m.image[selected].size=0;m.confirmed=other;
        m.flags=(m.flags&~FW_CONFIRMED)|FW_FALLBACK;m.last_error=FW_BAD_IMAGE;
        if(fw_store(f,&m))return FW_IO_ERROR;
        fw_log(f,FW_HISTORY_ROLLBACK,selected,FW_BAD_IMAGE,&rejected);
        selected=other;
    }
    *slot=selected;return FW_OK;
}
/** @brief End an upload and persist its failure status. */
static int fw_failed(fw_update_t *f,int result)
{
    f->uploading=false;
    fw_metadata_t m=f->meta;m.flags=(m.flags&~FW_UPLOADING)|FW_UPLOAD_FAILED;m.last_error=(uint32_t)result;
    int stored=fw_store(f,&m);
    fw_log(f,FW_HISTORY_FAILED,f->target,result,&f->incoming);
    return stored==FW_OK?result:FW_IO_ERROR;
}
/** @brief Invalidate and erase the inactive slot before starting a bounded image upload. */
int fw_begin(fw_update_t *f,uint32_t size,const uint8_t sha[32],uint32_t version)
{
    if(f->uploading || f->meta.pending!=FW_NONE)return FW_BUSY;
    if(size<FW_VECTOR_OFFSET+8U || size>FW_MAX_IMAGE || (size&3U))return FW_BAD_REQUEST;
    if(f->record<0 || fw_slot_valid(f,f->meta.confirmed)!=FW_OK)return FW_NOT_READY;
    f->target=f->meta.confirmed^1U;f->received=0;
    f->incoming=(fw_image_t){.size=size,.version=version};memcpy(f->incoming.sha256,sha,32);
    fw_metadata_t m=f->meta;m.image[f->target]=(fw_image_t){0};
    m.flags=(m.flags&FW_WARNINGS)|FW_UPLOADING;m.last_error=FW_OK;
    /* Invalidate the inactive descriptor durably BEFORE touching that slot. */
    if(fw_store(f,&m))return FW_IO_ERROR;
    fw_log(f,FW_HISTORY_UPLOAD_STARTED,f->target,FW_OK,&f->incoming);
    for(uint32_t pos=0;pos<size;pos+=FW_ERASE_SIZE)
        if(f->io.erase(f->io.context,fw_slot_address(f->target)+pos))return fw_failed(f,FW_IO_ERROR);
    f->uploading=true;return FW_OK;
}
/** @brief Validate, program, and read back the next sequential upload chunk. */
int fw_data(fw_update_t *f,uint32_t offset,const uint8_t *data,uint32_t size,uint32_t crc)
{
    if(!f->uploading)return FW_NOT_READY;
    if(!size || size>FW_CHUNK_SIZE || (size&3U) || offset!=f->received || size>f->incoming.size-f->received)return FW_BAD_OFFSET;
    if(fw_crc32(data,size)!=crc)return FW_BAD_HASH;
    uint8_t verify[FW_CHUNK_SIZE];uint32_t address=fw_slot_address(f->target)+offset;
    if(f->io.write(f->io.context,address,data,size) || f->io.read(f->io.context,address,verify,size) ||
       memcmp(data,verify,size))return fw_failed(f,FW_IO_ERROR);
    f->received+=size;return FW_OK;
}
/** @brief Verify the complete uploaded image and commit it as pending for trial boot. */
int fw_finish(fw_update_t *f)
{
    if(!f->uploading || f->received!=f->incoming.size)return FW_NOT_READY;
    fw_image_t image=f->incoming;int result=fw_validate(f,f->target,&image,true);
    if(result)return fw_failed(f,result);
    fw_metadata_t m=f->meta;m.image[f->target]=image;m.pending=f->target;m.trial=0;
    m.flags=(m.flags&FW_WARNINGS)|FW_UPLOADED|FW_VERIFIED;m.last_error=FW_OK;
    if(fw_store(f,&m))return fw_failed(f,FW_IO_ERROR);
    f->uploading=false;fw_log(f,FW_HISTORY_VERIFIED,f->target,FW_OK,&image);return FW_OK;
}
/** @brief Cancel an unfinished upload and persist the aborted flag. */
int fw_abort(fw_update_t *f)
{
    if(f->meta.pending!=FW_NONE)return FW_BUSY;
    if(!(f->meta.flags&FW_UPLOADING))return FW_OK;
    fw_metadata_t m=f->meta;m.flags=(m.flags&~FW_UPLOADING)|FW_UPLOAD_ABORTED;m.last_error=FW_OK;
    int r=fw_store(f,&m);if(!r){f->uploading=false;fw_log(f,FW_HISTORY_ABORTED,f->target,FW_OK,&f->incoming);}return r;
}
/** @brief Promote a validated trial image to the confirmed boot slot. */
int fw_confirm(fw_update_t *f,uint32_t slot)
{
    if(f->meta.pending==FW_NONE)return slot==f->meta.confirmed?FW_OK:FW_BAD_REQUEST;
    if(slot!=f->meta.pending || !f->meta.trial)return FW_NOT_READY;
    if(fw_slot_valid(f,slot)!=FW_OK)return FW_BAD_IMAGE;
    fw_metadata_t m=f->meta;m.confirmed=slot;m.pending=FW_NONE;m.trial=0;m.flags|=FW_CONFIRMED;
    int result=fw_store(f,&m);
    if(!result)fw_log(f,FW_HISTORY_CONFIRMED,slot,FW_OK,&m.image[slot]);
    return result;
}
/** @brief Clear sticky update warnings and the recorded error in persistent metadata. */
int fw_clear_warnings(fw_update_t *f)
{
    fw_metadata_t m=f->meta;m.flags&=~FW_WARNINGS;m.last_error=FW_OK;
    int result=fw_store(f,&m);
    if(!result)fw_log(f,FW_HISTORY_WARNINGS_CLEARED,FW_NONE,FW_OK,NULL);
    return result;
}
