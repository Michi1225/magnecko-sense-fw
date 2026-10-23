#include "fw_parameters.h"
#include <string.h>

typedef struct { const uint8_t *start,*end; } params_json_span_t;
static void params_json_space(const uint8_t **p)
{ while(**p==' ' || **p=='\n' || **p=='\r' || **p=='\t')++*p; }
static int params_json_hex(uint8_t c)
{ return c>='0' && c<='9'?c-'0':c>='a' && c<='f'?c-'a'+10:c>='A' && c<='F'?c-'A'+10:-1; }
static bool params_json_string(const uint8_t **p)
{
    if(*(*p)++!='"')return false;
    while(**p && **p!='"'){
        uint8_t c=*(*p)++;
        if(c<32)return false;
        if(c=='\\'){
            c=*(*p)++;if(!c)return false;
            if(c=='u'){for(unsigned i=0;i<4;++i)if(params_json_hex(*(*p)++)<0)return false;}
            else if(!strchr("\"\\/bfnrt",c))return false;
        }
    }
    if(**p!='"')return false;
    ++*p;return true;
}
static bool params_json_value(const uint8_t **p,unsigned depth)
{
    params_json_space(p);if(depth>24)return false;
    uint8_t c=**p;
    if(c=='"')return params_json_string(p);
    if(c=='{' || c=='['){
        bool object=c=='{';uint8_t close=object?'}':']';++*p;params_json_space(p);
        if(**p==close){++*p;return true;}
        for(;;){
            if(object){if(!params_json_string(p))return false;params_json_space(p);if(*(*p)++!=':')return false;}
            if(!params_json_value(p,depth+1))return false;
            params_json_space(p);c=*(*p)++;
            if(c==close)return true;
            if(c!=',')return false;
            params_json_space(p);
        }
    }
    const char *literal=c=='t'?"true":c=='f'?"false":c=='n'?"null":NULL;
    if(literal){while(*literal)if(*(*p)++!=(uint8_t)*literal++)return false;return true;}
    if(c=='-')++*p;
    if(**p=='0')++*p;
    else {if(**p<'1' || **p>'9')return false;while(**p>='0' && **p<='9')++*p;}
    if(**p=='.'){++*p;if(**p<'0' || **p>'9')return false;while(**p>='0' && **p<='9')++*p;}
    if(**p=='e' || **p=='E'){
        ++*p;if(**p=='+' || **p=='-')++*p;
        if(**p<'0' || **p>'9')return false;
        while(**p>='0' && **p<='9')++*p;
    }
    return true;
}
/* Decode ASCII field names, including JSON unicode escapes, so escaped aliases
 * cannot hide a second build/flash key or overwrite an editable field. */
static bool params_json_key(params_json_span_t key,const char *name)
{
    const uint8_t *p=key.start+1;
    while(p<key.end-1){
        unsigned c=*p++;
        if(c=='\\'){
            c=*p++;
            if(c=='u'){c=0;for(unsigned i=0;i<4;++i)c=c*16U+(unsigned)params_json_hex(*p++);}
            else if(c=='b')c='\b';else if(c=='f')c='\f';else if(c=='n')c='\n';
            else if(c=='r')c='\r';else if(c=='t')c='\t';
        }
        if(!*name || c!=(unsigned char)*name++)return false;
    }
    return !*name;
}
/* The entire document is validated before these bounded spans are walked. */
static bool params_json_pair(const uint8_t **p,params_json_span_t *key,params_json_span_t *value)
{
    params_json_space(p);if(**p==',' || **p=='{'){++*p;params_json_space(p);}
    if(**p=='}')return false;
    key->start=*p;(void)params_json_string(p);key->end=*p;
    params_json_space(p);++*p;params_json_space(p);
    value->start=*p;(void)params_json_value(p,0);value->end=*p;
    return true;
}
static bool params_json_object(const uint8_t *text)
{
    const uint8_t *q=text;
    while(*q){
        uint32_t code=*q++,minimum;unsigned count;
        if(code<128)continue;
        if(code>=0xc2 && code<=0xdf){count=1;minimum=0x80;code&=0x1f;}
        else if(code>=0xe0 && code<=0xef){count=2;minimum=0x800;code&=0xf;}
        else if(code>=0xf0 && code<=0xf4){count=3;minimum=0x10000;code&=7;}
        else return false;
        while(count--){if((*q&0xc0)!=0x80)return false;code=(code<<6)|(*q++&0x3f);}
        if(code<minimum || code>0x10ffff || (code>=0xd800 && code<=0xdfff))return false;
    }
    const uint8_t *p=text;params_json_space(&p);if(*p!='{' || !params_json_value(&p,0))return false;
    params_json_space(&p);return !*p;
}
static int params_json_member(params_json_span_t object,const char *name,params_json_span_t *value)
{
    if(*object.start!='{')return -1;
    const uint8_t *p=object.start;params_json_span_t key,item;int found=0;
    while(params_json_pair(&p,&key,&item))if(params_json_key(key,name)){*value=item;++found;}
    return found>1?-1:found;
}
typedef struct { uint8_t *data;uint32_t size;bool ok; } params_json_output_t;
static void params_json_add(params_json_output_t *o,const void *p,size_t n)
{
    if(n>=FW_PARAMETERS_MAX-o->size){o->ok=false;return;}
    memcpy(o->data+o->size,p,n);o->size+=(uint32_t)n;
}
static void params_json_span(params_json_output_t *o,params_json_span_t span)
{ params_json_add(o,span.start,(size_t)(span.end-span.start)); }
static void params_json_field(params_json_output_t *o,const char *key,params_json_span_t value,bool *first)
{
    if(!*first)params_json_add(o,",",1);
    *first=false;
    params_json_add(o,"\"",1);params_json_add(o,key,strlen(key));params_json_add(o,"\":",2);params_json_span(o,value);
}
static int params_json_build(params_json_output_t *o,params_json_span_t build,const char *label,params_json_span_t record)
{
    params_json_span_t key,value,unused;bool first=true;
    if(params_json_member(build,label,&unused)<0)return FW_BAD_IMAGE;
    params_json_add(o,"{",1);const uint8_t *p=build.start;
    while(params_json_pair(&p,&key,&value)){
        if(params_json_key(key,label))continue;
        if(!first)params_json_add(o,",",1);
        first=false;params_json_span(o,key);params_json_add(o,":",1);params_json_span(o,value);
    }
    params_json_field(o,label,record,&first);params_json_add(o,"}",1);return FW_OK;
}
int fw_parameters_merge(const uint8_t *previous,const uint8_t *envelope,uint32_t target,
                        uint32_t image_size,const uint8_t sha256[32],uint8_t *output,uint32_t *size)
{
    if(target>FW_PARAMETERS_FSBL || !params_json_object(previous) || !params_json_object(envelope))return FW_BAD_IMAGE;
    const uint8_t *a=previous,*b=envelope;params_json_space(&a);params_json_space(&b);
    params_json_span_t root={a,a+strlen((const char *)a)},input={b,b+strlen((const char *)b)};
    params_json_span_t record,parameter_build,flash,hash,build,old,unused,repository;
    if(params_json_member(input,"record",&record)!=1 || *record.start!='{' ||
       params_json_member(input,"parameter_build",&parameter_build)!=1 || *parameter_build.start!='{' ||
       params_json_member(input,"flash",&flash)!=1 || *flash.start!='{' ||
       params_json_member(record,"image_sha256",&hash)!=1)return FW_BAD_IMAGE;
    char digest[67]="\"";static const char hex[]="0123456789abcdef";
    for(unsigned i=0;i<32;++i){digest[1+i*2]=hex[sha256[i]>>4];digest[2+i*2]=hex[sha256[i]&15];}
    digest[65]='"';digest[66]=0;
    if(hash.end-hash.start!=66 || memcmp(hash.start,digest,66))return FW_BAD_HASH;
    params_json_span_t length;
    if(params_json_member(record,"image_size_bytes",&length)!=1 || length.start==length.end)return FW_BAD_IMAGE;
    uint32_t bytes=0;
    for(const uint8_t *q=length.start;q<length.end;++q){
        if(*q<'0' || *q>'9' || bytes>(UINT32_MAX-(*q-'0'))/10U)return FW_BAD_IMAGE;
        bytes=bytes*10U+(*q-'0');
    }
    if(bytes!=image_size)return FW_BAD_IMAGE;
    int have_repository=params_json_member(input,"repository",&repository);
    if(have_repository<0 || (have_repository && *repository.start!='{'))return FW_BAD_IMAGE;
    static const uint8_t empty[]="{}";build=(params_json_span_t){empty,empty+2};
    int found=params_json_member(root,"build",&old);
    if(found<0)return FW_BAD_IMAGE;
    if(found)build=old;
    if(params_json_member(root,"parameter_build",&unused)<0 || params_json_member(root,"flash",&unused)<0 ||
       (have_repository && params_json_member(root,"repository",&unused)<0))return FW_BAD_IMAGE;
    params_json_output_t o={output,0,true};bool first=true;
    params_json_add(&o,"{",1);const uint8_t *p=root.start;params_json_span_t key,value;
    while(params_json_pair(&p,&key,&value)){
        if(params_json_key(key,"build") || params_json_key(key,"parameter_build") || params_json_key(key,"flash") ||
           (have_repository && params_json_key(key,"repository")))continue;
        if(!first)params_json_add(&o,",",1);
        first=false;params_json_span(&o,key);params_json_add(&o,":",1);params_json_span(&o,value);
    }
    if(!first)params_json_add(&o,",",1);
    params_json_add(&o,"\"build\":",8);
    const char *label=target==0?"application":target==1?"application_b":"fsbl";
    int result=params_json_build(&o,build,label,record);if(result)return result;
    first=false;params_json_field(&o,"parameter_build",parameter_build,&first);params_json_field(&o,"flash",flash,&first);
    if(have_repository)params_json_field(&o,"repository",repository,&first);
    params_json_add(&o,"}",1);if(!o.ok)return FW_BAD_OFFSET;
    output[o.size]=0;*size=o.size+1U;return FW_OK;
}
