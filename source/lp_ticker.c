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
#include "sleep_api.h"
#include "uvisor-lib/uvisor-lib.h"

static TIM_HandleTypeDef TimMasterHandle;
static uint8_t lp_ticker_inited = 0;
static volatile uint32_t overflows = 0;

static void lp_handler(void)
{
    __HAL_TIM_CLEAR_IT(&TimMasterHandle, TIM_IT_CC1);
    overflows++;
}

void lp_ticker_init(void) {
    if (!lp_ticker_inited) {
        lp_ticker_inited = 1;
        __TIM2_CLK_ENABLE();
        __TIM2_FORCE_RESET();
        __TIM2_RELEASE_RESET();

        // Update the SystemCoreClock variable
        SystemCoreClockUpdate();

        // Configure time base
        TimMasterHandle.Instance = TIM2;
        TimMasterHandle.Init.Period            = 0xFFFFFFFF;
        TimMasterHandle.Init.Prescaler         = (uint32_t)(SystemCoreClock / 1000000000) - 1; // 1 ms tick
        TimMasterHandle.Init.ClockDivision     = 0;
        TimMasterHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
        TimMasterHandle.Init.RepetitionCounter = 0;
        HAL_TIM_OC_Init(&TimMasterHandle);

        vIRQ_SetVector(TIM2_IRQn, (uint32_t)lp_handler);
        vIRQ_EnableIRQ(TIM2_IRQn);

        HAL_TIM_OC_Start(&TimMasterHandle, TIM_CHANNEL_1);
    }
}

uint32_t lp_ticker_read() {
    if (!lp_ticker_inited) {
        lp_ticker_init();
    }
    return TIM2->CNT;
}

uint32_t lp_ticker_get_overflows_counter(void) {
    return overflows;
}

uint32_t lp_ticker_get_compare_match(void) {
    return TIM2->CCMR1;
}

void lp_ticker_set_interrupt(uint32_t now, uint32_t time) {
    (void)now;
    // Set new output compare value
    __HAL_TIM_SetCompare(&TimMasterHandle, TIM_CHANNEL_1, time);
    // Enable IT
    __HAL_TIM_ENABLE_IT(&TimMasterHandle, TIM_IT_CC1);
}

void lp_ticker_sleep_until(uint32_t now, uint32_t time)
{
    lp_ticker_set_interrupt(now, time);
    sleep_t sleep_obj;
    mbed_enter_sleep(&sleep_obj);
    mbed_exit_sleep(&sleep_obj);
}

#endif
