/* Included once from FSBL main.c's USER CODE section. Keeping the integration
 * here avoids edits to CubeMX-generated source lists and boot middleware. */
#include "fw_update.c"
#include "fw_history.c"
#include "fw_flash.c"
#include "fw_handoff.h"
#include "LEDController.h"
extern SEMPER_Handle_t Flash;
static fw_update_t fw_boot_state;
static fw_history_t fw_boot_history;
static bool fw_boot_led_ready;
/* Retained as debugger symbols when debug access is available. LED indications
 * also work with BOOT1=0 and do not depend on a connected debugger. */
static volatile uint32_t fw_boot_stage;
static volatile int fw_boot_result;

static void fw_boot_led_init(void)
{
    LEDController_Init(&led_controller);
    LEDController_SetAnimation(&led_controller, LED_ANIMATION_FSBL_STARTED);
    fw_boot_led_ready = true;
}

/** @brief Branch without executing compiler stack accesses after replacing MSP. */
__attribute__((naked, noreturn))
static void fw_boot_branch(uint32_t stack __attribute__((unused)),
                           uint32_t reset __attribute__((unused)))
{
    __asm volatile (
        "movs r2, #0\n"
        "msr msplim, r2\n"
        "msr psplim, r2\n"
        "msr basepri, r2\n"
        "msr faultmask, r2\n"
        "msr control, r2\n"
        "isb\n"
        "msr msp, r0\n"
        "mvn lr, r2\n"
        "dsb\n"
        "isb\n"
        "cpsie i\n"
        "bx r1\n");
}

/** @brief Select and verify a boot slot, load its image into SRAM, and jump to the application. */
static int fw_boot_run(void)
{
    fw_flash_t flash={&Flash,fw_watchdog_refresh};
    fw_io_t io=fw_flash_io(&flash);
    fw_boot_stage=3U; /* Read A/B metadata. */
    LEDController_SetAnimation(&led_controller, LED_ANIMATION_FSBL_LOADING);
    int result=fw_open(&fw_boot_state,&io);
    if(result)return result;
    fw_io_t history_io=fw_flash_history_io(&flash);
    (void)fw_history_open(&fw_boot_history,&history_io,NULL);
    fw_boot_state.history=&fw_boot_history;
    uint32_t slot;bool trial;
    fw_boot_stage=4U; /* Validate and select a slot. */
    result=fw_boot_select(&fw_boot_state,&slot,&trial);
    if(result)return result;
    if(trial && fw_watchdog_start())return FW_IO_ERROR;
    uint32_t size=fw_boot_state.meta.image[slot].size;
    uint8_t *destination=(uint8_t *)FW_RAM_BASE;
    fw_boot_stage=5U; /* Copy the selected image. */
    /* The application is LRUN: load its signed header/padding plus payload,
     * leaving the vector table at 0x34000400. */
    for(uint32_t offset=0;offset<size;){
        uint32_t count=size-offset;if(count>512U)count=512U;
        if(io.read(io.context,fw_slot_address(slot)+offset,destination+offset,count))return FW_IO_ERROR;
        fw_watchdog_refresh();offset+=count;
    }
    fw_boot_stage=6U; /* Verify the SRAM copy. */
    uint8_t digest[32];fw_sha256(destination,size,digest);
    if(memcmp(digest,fw_boot_state.meta.image[slot].sha256,32))return FW_BAD_HASH;
    fw_watchdog_refresh();
    fw_boot_stage=7U;
    LEDController_SetAnimation(&led_controller,LED_ANIMATION_FSBL_HANDOFF);
    const uint32_t *vectors=(const uint32_t *)(FW_RAM_BASE+FW_VECTOR_OFFSET);
    uint32_t stack=vectors[0],reset=vectors[1];
    fw_prepare_handoff(FW_RAM_BASE+FW_VECTOR_OFFSET);
    fw_boot_branch(stack,reset);
}
