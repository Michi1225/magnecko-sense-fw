#include "fw_flash.h"
#include "fw_fsbl.h"
#include "fw_parameters.h"
#include "fw_history.h"

/** @brief Wait for flash readiness with a timeout and an optional idle callback. */
static int fw_flash_wait(fw_flash_t *f)
{
    uint32_t start=HAL_GetTick();
    for(;;){
        SEMPER_Status_t status=SEMPER_GetStatus(f->flash);
        if(status==SEMPER_OK)return 0;
        if(status!=SEMPER_BUSY || HAL_GetTick()-start>4000U)return -1;
        if(f->idle)f->idle();
    }
}
/** @brief Check that a write fits entirely inside an allowed image or metadata range. */
static bool fw_flash_writable(uint32_t address,uint32_t size)
{
    const uint32_t base[]={FW_SLOT_A,FW_SLOT_B,FW_META_A,FW_META_B};
    const uint32_t limit[]={FW_MAX_IMAGE,FW_MAX_IMAGE,FW_RECORD_SIZE,FW_RECORD_SIZE};
    for(unsigned i=0;i<4;++i)
        if(address>=base[i] && address-base[i]<limit[i] && size<=limit[i]-(address-base[i]))return true;
    return false;
}
/** @brief Read flash bytes after checking device bounds and readiness. */
static int fw_flash_read(void *ctx,uint32_t address,void *data,uint32_t size)
{
    fw_flash_t *f=ctx;
    if(address>=0x04000000U || size>0x04000000U-address)return -1;
    if(fw_flash_wait(f))return -1;
    return SEMPER_Read(f->flash,address,data,size)==SEMPER_OK?0:-1;
}
/** @brief Program an allowed update range and wait for flash completion. */
static int fw_flash_write(void *ctx,uint32_t address,const void *data,uint32_t size)
{
    fw_flash_t *f=ctx;
    if(!size || !fw_flash_writable(address,size) || fw_flash_wait(f))return -1;
    if(SEMPER_Write(f->flash,address,data,size)!=SEMPER_OK)return -1;
    return fw_flash_wait(f);
}
/** @brief Erase an aligned update sector after checking the allowed flash ranges. */
static int fw_flash_erase(void *ctx,uint32_t address)
{
    fw_flash_t *f=ctx;
    if((address%FW_ERASE_SIZE) ||
       !((address>=FW_SLOT_A && address<FW_SLOT_A+FW_MAX_IMAGE) ||
         (address>=FW_SLOT_B && address<FW_SLOT_B+FW_MAX_IMAGE) ||
          address==FW_META_A || address==FW_META_B) || fw_flash_wait(f))return -1;
    if(SEMPER_Erase256K(f->flash,address)!=SEMPER_OK)return -1;
    return fw_flash_wait(f);
}
/** @brief Build the bounded flash callbacks used by the portable updater. */
fw_io_t fw_flash_io(fw_flash_t *f)
{ return (fw_io_t){f,fw_flash_read,fw_flash_write,fw_flash_erase}; }
static bool fw_flash_fsbl_range(uint32_t a,uint32_t n)
{ return n && a>=FW_FSBL_BASE && a-FW_FSBL_BASE<FW_FSBL_MAX && n<=FW_FSBL_MAX-(a-FW_FSBL_BASE); }
static int fw_flash_fsbl_read(void *ctx,uint32_t a,void *p,uint32_t n)
{ return fw_flash_fsbl_range(a,n)?fw_flash_read(ctx,a,p,n):-1; }
static int fw_flash_fsbl_write(void *ctx,uint32_t a,const void *p,uint32_t n)
{
    fw_flash_t *f=ctx;
    if(f->idle)f->idle(); /* Feed IWDG and service USB between bounded writes. */
    if(!fw_flash_fsbl_range(a,n) || fw_flash_wait(f) || SEMPER_Write(f->flash,a,p,n)!=SEMPER_OK)return -1;
    return fw_flash_wait(f);
}
static int fw_flash_fsbl_erase(void *ctx,uint32_t a)
{
    fw_flash_t *f=ctx;
    if(f->idle)f->idle();
    if(a!=FW_FSBL_BASE || fw_flash_wait(f) || SEMPER_Erase256K(f->flash,a)!=SEMPER_OK)return -1;
    return fw_flash_wait(f);
}
/** @brief Separate, narrowly bounded IO for explicitly requested bootloader installation. */
fw_io_t fw_flash_fsbl_io(fw_flash_t *f)
{ return (fw_io_t){f,fw_flash_fsbl_read,fw_flash_fsbl_write,fw_flash_fsbl_erase}; }
static bool fw_flash_parameters_range(uint32_t a,uint32_t n)
{ return n && a<FW_PARAMETERS_END && n<=FW_PARAMETERS_END-a; }
static int fw_flash_parameters_read(void *ctx,uint32_t a,void *p,uint32_t n)
{ return fw_flash_parameters_range(a,n)?fw_flash_read(ctx,a,p,n):-1; }
static int fw_flash_parameters_write(void *ctx,uint32_t a,const void *p,uint32_t n)
{
    fw_flash_t *f=ctx;
    if(!fw_flash_parameters_range(a,n) || (a&15U) || (n&15U))return -1;
    if(f->idle)f->idle();
    if(fw_flash_wait(f) || SEMPER_Write(f->flash,a,p,n)!=SEMPER_OK)return -1;
    return fw_flash_wait(f);
}
static int fw_flash_parameters_erase(void *ctx,uint32_t a)
{
    fw_flash_t *f=ctx;
    if(a%FW_PARAMETERS_SECTOR || !fw_flash_parameters_range(a,FW_PARAMETERS_SECTOR))return -1;
    if(f->idle)f->idle();
    if(fw_flash_wait(f) || SEMPER_Erase4K(f->flash,a)!=SEMPER_OK)return -1;
    return fw_flash_wait(f);
}
fw_io_t fw_flash_parameters_io(fw_flash_t *f)
{ return (fw_io_t){f,fw_flash_parameters_read,fw_flash_parameters_write,fw_flash_parameters_erase}; }
static bool fw_flash_history_range(uint32_t a,uint32_t n)
{ return n && a>=FW_HISTORY_BASE && a-FW_HISTORY_BASE<FW_HISTORY_SIZE && n<=FW_HISTORY_SIZE-(a-FW_HISTORY_BASE); }
static int fw_flash_history_read(void *ctx,uint32_t a,void *p,uint32_t n)
{ return fw_flash_history_range(a,n)?fw_flash_read(ctx,a,p,n):-1; }
static int fw_flash_history_write(void *ctx,uint32_t a,const void *p,uint32_t n)
{
    fw_flash_t *f=ctx;
    if(!fw_flash_history_range(a,n) || (a&15U) || (n&15U))return -1;
    if(f->idle)f->idle();
    if(fw_flash_wait(f) || SEMPER_Write(f->flash,a,p,n)!=SEMPER_OK)return -1;
    return fw_flash_wait(f);
}
static int fw_flash_history_erase(void *ctx,uint32_t a)
{
    fw_flash_t *f=ctx;
    if(a%FW_ERASE_SIZE || !fw_flash_history_range(a,FW_ERASE_SIZE))return -1;
    if(f->idle)f->idle();
    if(fw_flash_wait(f) || SEMPER_Erase256K(f->flash,a)!=SEMPER_OK)return -1;
    return fw_flash_wait(f);
}
fw_io_t fw_flash_history_io(fw_flash_t *f)
{ return (fw_io_t){f,fw_flash_history_read,fw_flash_history_write,fw_flash_history_erase}; }
/** @brief Restore the single-line SPI protocol expected by the boot ROM after a CPU reset. */
int fw_flash_prepare_reset(fw_flash_t *f)
{
    if(!f || !f->flash || f->flash->access!=SEMPER_ACCESS_INDIRECT || fw_flash_wait(f))return -1;
    /* Octal DDR normally runs at 200 MHz. Slow down before entering legacy SPI. */
    SEMPER_Handle_t *dev=f->flash;
    if(HAL_XSPI_SetClockPrescaler(dev->hxspi,3U)!=HAL_OK ||
       S28HS512T_ResetEnable(dev->hxspi,dev->interface,dev->transfer)!=S28HS512T_OK ||
       S28HS512T_ResetMemory(dev->hxspi,dev->interface,dev->transfer)!=S28HS512T_OK)return -1;
    /* Restore all volatile protocol/latency registers together. Switching only
     * CFR5 leaves the octal driver's register-read latency active in SPI mode. */
    HAL_Delay(S28HS512T_RESET_MAX_TIME);
    dev->interface=S28HS512T_SPI_MODE;dev->transfer=S28HS512T_STR_TRANSFER;
    dev->address_width=S28HS512T_3BYTES_SIZE;
    uint8_t id[3]={0};
    if(S28HS512T_ReadID(f->flash->hxspi,S28HS512T_SPI_MODE,S28HS512T_STR_TRANSFER,id)!=S28HS512T_OK ||
       id[0]==0U || id[0]==0xFFU)return -1;
    return 0;
}
/** @brief Reload the independent watchdog if it is already running. */
void fw_watchdog_refresh(void)
{
    if(IWDG->SR&IWDG_SR_ONF)IWDG->KR=0xAAAAU;
}
/** @brief Start the independent watchdog with a nominal 32-second trial boot timeout. */
int fw_watchdog_start(void)
{
    /* Independent clock; PR=6 divides LSI by 256. With nominal 32 kHz LSI,
     * a 4095 reload gives about 32 seconds for trial startup/confirmation. */
    LL_RCC_LSI_Enable();
    uint32_t start=HAL_GetTick();
    while(!LL_RCC_LSI_IsReady())if(HAL_GetTick()-start>1000U)return -1;
    IWDG->KR=0xCCCCU;IWDG->KR=0x5555U;
    IWDG->PR=6U;IWDG->RLR=4095U;IWDG->WINR=4095U;
    start=HAL_GetTick();
    while(IWDG->SR&(IWDG_SR_PVU|IWDG_SR_RVU|IWDG_SR_WVU))
        if(HAL_GetTick()-start>1000U)return -1;
    IWDG->KR=0xAAAAU;return 0;
}
