/* mbed Microcontroller Library
 * Copyright (c) 2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "cmsis.h"
#include "device.h"

#if DEVICE_LOWPOWERTIMER

#include "lp_ticker_api.h"

static TIM_HandleTypeDef TimMasterHandle;
static uint8_t lp_ticker_inited = 0;

static lp_handler(void)
{
    __HAL_TIM_DISABLE_IT();
}

void lp_ticker_init(void) {
    if (!lp_ticker_inited) {
        us_ticker_inited = 1;
        TimMasterHandle.Instance = TIM6;
        __TIM6_CLK_ENABLE();
        __TIM6_FORCE_RESET();
        __TIM6_RELEASE_RESET();

        // Update the SystemCoreClock variable
        SystemCoreClockUpdate();

        // Configure time base
        TimMasterHandle.Instance = TIM_MST;
        TimMasterHandle.Init.Period            = 0xFFFFFFFF;
        TimMasterHandle.Init.Prescaler         = (uint32_t)(SystemCoreClock / 1000) - 1; // 1 ms tick
        TimMasterHandle.Init.ClockDivision     = 0;
        TimMasterHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
        TimMasterHandle.Init.RepetitionCounter = 0;
        HAL_TIM_OC_Init(&TimMasterHandle);

        NVIC_SetVector(TIM_MST_IRQ, (uint32_t)lp_handler);
        NVIC_EnableIRQ(TIM_MST_IRQ);

        HAL_TIM_OC_Start(&TimMasterHandle, TIM_CHANNEL_1);
        // __HAL_TIM_ENABLE_IT(&TimMasterHandle, TIM_IT_CC2);
    }
}

uint32_t lp_ticker_read() {
    if (!lp_ticker_inited) {
        lp_ticker_init();
    }
    return TIM6->CNT;
}

void lp_ticker_set_interrupt(uint32_t now, uint32_t time) {
    // Set new output compare value
    __HAL_TIM_SetCompare(&TimMasterHandle, TIM_CHANNEL_1, time);
    // Enable IT
    __HAL_TIM_ENABLE_IT(&TimMasterHandle, TIM_IT_CC1);
}

#endif
