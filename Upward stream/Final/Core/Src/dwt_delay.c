#include "dwt_delay.h"

void DWT_Init(void) {
    // Обновляем глобальную переменную SystemCoreClock
    SystemCoreClockUpdate();

    // Включаем DWT (для Cortex-M4 этого достаточно)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void DWT_Delay_us(uint32_t us) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - startTick) < delayTicks);
}
