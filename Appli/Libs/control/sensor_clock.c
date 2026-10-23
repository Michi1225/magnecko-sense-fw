#include "sensor_clock.h"
#include "main.h"

static volatile uint32_t running;
static uint32_t previous;
static uint64_t elapsed;

void sensor_clock_init(void)
{
    uint32_t frequency = HAL_RCCEx_GetTIMGFreq();
    if (frequency < 1000000U || frequency % 1000000U) Error_Handler();
    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_TIM2,
                                         RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM2_FORCE_RESET();
    __HAL_RCC_TIM2_RELEASE_RESET();
    TIM2->PSC = frequency / 1000000U - 1U;
    TIM2->ARR = UINT32_MAX;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    previous = 0;
    elapsed = 0;
    TIM2->CR1 = TIM_CR1_CEN;
    running = 1;
}

uint64_t sensor_clock_now_us(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (running) {
        uint32_t now = TIM2->CNT;
        elapsed += (uint32_t)(now - previous);
        previous = now;
    }
    uint64_t result = elapsed;
    __set_PRIMASK(mask);
    return result;
}
