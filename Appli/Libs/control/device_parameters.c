#include "device_parameters.h"
#include "semper_flash.h"
#include "xspi.h"
#include "../../../common/firmware/fw_parameters.h"
#include "tx_api.h"
#include <string.h>

/* Same 16-byte framing as VL9F/IMUF; PARM payload is UTF-8 JSON. */
static uint8_t packet[16U + DEVICE_PARAMETERS_MAX_JSON];
static uint32_t packet_size;

/** @brief Wrap parameter JSON in a PARM telemetry packet. */
static void set_payload(const uint8_t *json, uint32_t length)
{
    const uint16_t version = 1U, reserved = 0U;
    const uint32_t sequence = 0U;
    memcpy(packet, "PARM", 4U);
    memcpy(packet + 4U, &sequence, 4U);
    memcpy(packet + 8U, &length, 4U);
    memcpy(packet + 12U, &version, 2U);
    memcpy(packet + 14U, &reserved, 2U);
    if (json != packet + 16U) memcpy(packet + 16U, json, length);
    packet_size = 16U + length;
}

static int parameters_startup_read(void *context,uint32_t address,void *data,uint32_t size)
{ return SEMPER_Read(context,address,data,size)==SEMPER_OK?0:-1; }
/** @brief Read device parameters from flash and prepare JSON telemetry or an error payload. */
void device_parameters_init(void)
{
    SEMPER_Handle_t flash = {0};
    static const uint8_t read_error[] = "{\"parameter_error\":\"flash read failed\"}";
    static const uint8_t missing[] = "{\"parameter_error\":\"parameters missing or too large\"}";
    /* Read offset zero indirectly; no dependency on FSBL mapping being left on. */
    uint32_t length;bool journal;
    fw_io_t io={&flash,parameters_startup_read,NULL,NULL};
    if (SEMPER_Init(&flash, &hxspi1) != SEMPER_OK)
    {
        set_payload(read_error, sizeof(read_error) - 1U);
        return;
    }
    int result=fw_parameters_load(&io,packet+16U,&length,&journal);
    if(result!=FW_OK){
        if(result==FW_IO_ERROR)set_payload(read_error,sizeof(read_error)-1U);
        else set_payload(missing,sizeof(missing)-1U);
        return;
    }
    for (uint32_t i = 0U; i < DEVICE_PARAMETERS_MAX_JSON; ++i)
    {
        if (packet[16U + i] == 0U)
        {
            if (i != 0U && packet[16U] == '{')
                set_payload(packet + 16U, i);
            else
                set_payload(missing, sizeof(missing) - 1U);
            return;
        }
    }
    set_payload(missing, sizeof(missing) - 1U);
}

void device_parameters_publish(const uint8_t *json,uint32_t length)
{
    if(!length || length>=DEVICE_PARAMETERS_MAX_JSON)return;
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    set_payload(json,length);
    TX_RESTORE
}
uint32_t device_parameters_copy_packet(uint8_t *destination,uint32_t capacity)
{
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    uint32_t size=packet_size<=capacity?packet_size:0U;
    if(size)memcpy(destination,packet,size);
    TX_RESTORE
    return size;
}
