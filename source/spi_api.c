/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2014, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */
#include "uvisor-lib/uvisor-lib.h"
#include "mbed_assert.h"
#include "spi_api.h"

#if DEVICE_SPI

#include <math.h>
#include <stdbool.h>
#include "cmsis.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "target_config.h"

static SPI_HandleTypeDef SpiHandle[MODULE_SIZE_SPI];
static const IRQn_Type SpiIRQs[MODULE_SIZE_SPI] = {
    SPI1_IRQn,
    SPI2_IRQn,
    SPI3_IRQn,
#if MODULE_SIZE_SPI > 3
    SPI4_IRQn,
#endif
#if MODULE_SIZE_SPI > 4
    SPI5_IRQn,
#endif
#if MODULE_SIZE_SPI > 5
    SPI6_IRQn
#endif
};

static void init_spi(spi_t *obj)
{
    SPI_HandleTypeDef *handle = &SpiHandle[obj->spi.module];

    __HAL_SPI_DISABLE(handle);

    HAL_SPI_Init(handle);

    __HAL_SPI_ENABLE(handle);
}

void spi_init(spi_t *obj, PinName mosi, PinName miso, PinName sclk)
{
    // Determine the SPI to use
    SPIName spi_mosi = (SPIName)pinmap_peripheral(mosi, PinMap_SPI_MOSI);
    SPIName spi_miso = (SPIName)pinmap_peripheral(miso, PinMap_SPI_MISO);
    SPIName spi_sclk = (SPIName)pinmap_peripheral(sclk, PinMap_SPI_SCLK);

    SPIName spi_data = (SPIName)pinmap_merge(spi_mosi, spi_miso);

    SPIName instance = (SPIName)pinmap_merge(spi_data, spi_sclk);
    MBED_ASSERT(instance != (SPIName)NC);

    // Enable SPI clock and set the right module number
    switch(instance)
    {
        case SPI_1:
            __SPI1_CLK_ENABLE();
            obj->spi.module = 0;
            break;

        case SPI_2:
            __SPI2_CLK_ENABLE();
            obj->spi.module = 1;
            break;

        case SPI_3:
            __SPI3_CLK_ENABLE();
            obj->spi.module = 2;
            break;

#if MODULE_SIZE_SPI > 3
        case SPI_4:
            __SPI4_CLK_ENABLE();
            obj->spi.module = 3;
            break;
#endif
#if MODULE_SIZE_SPI > 4
        case SPI_5:
            __SPI5_CLK_ENABLE();
            obj->spi.module = 4;
            break;
#endif
#if MODULE_SIZE_SPI > 5
        case SPI_6:
            __SPI6_CLK_ENABLE();
            obj->spi.module = 5;
            break;
#endif
        default:
            break;
    }

    // Configure the SPI pins
    pinmap_pinout(mosi, PinMap_SPI_MOSI);
    pinmap_pinout(miso, PinMap_SPI_MISO);
    pinmap_pinout(sclk, PinMap_SPI_SCLK);

    obj->spi.pin_miso = miso;
    obj->spi.pin_mosi = mosi;
    obj->spi.pin_sclk = sclk;

    // initialize the handle for this master!
    SPI_HandleTypeDef *handle = &SpiHandle[obj->spi.module];

    handle->Instance               = (SPI_TypeDef *)(instance);
    handle->Init.Mode              = SPI_MODE_MASTER;
    handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    handle->Init.Direction         = SPI_DIRECTION_2LINES;
    handle->Init.CLKPhase          = SPI_PHASE_1EDGE;
    handle->Init.CLKPolarity       = SPI_POLARITY_LOW;
    handle->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
    handle->Init.CRCPolynomial     = 7;
    handle->Init.DataSize          = SPI_DATASIZE_8BIT;
    handle->Init.FirstBit          = SPI_FIRSTBIT_MSB;
    handle->Init.NSS               = SPI_NSS_SOFT;
    handle->Init.TIMode            = SPI_TIMODE_DISABLED;

    init_spi(obj);
}

void spi_free(spi_t *obj)
{
    // Reset SPI and disable clock
    switch(obj->spi.module)
    {
        case 0:
            __SPI1_FORCE_RESET();
            __SPI1_RELEASE_RESET();
            __SPI1_CLK_DISABLE();
            break;

        case 1:
            __SPI2_FORCE_RESET();
            __SPI2_RELEASE_RESET();
            __SPI2_CLK_DISABLE();
            break;

        case 2:
            __SPI3_FORCE_RESET();
            __SPI3_RELEASE_RESET();
            __SPI3_CLK_DISABLE();
            break;

#if MODULE_SIZE_SPI > 3
        case 3:
            __SPI4_FORCE_RESET();
            __SPI4_RELEASE_RESET();
            __SPI4_CLK_DISABLE();
            break;
#endif
#if MODULE_SIZE_SPI > 4
        case 4:
            __SPI5_FORCE_RESET();
            __SPI5_RELEASE_RESET();
            __SPI5_CLK_DISABLE();
            break;
#endif
#if MODULE_SIZE_SPI > 5
        case 5:
            __SPI6_FORCE_RESET();
            __SPI6_RELEASE_RESET();
            __SPI6_CLK_DISABLE();
            break;
#endif
        default:
            break;
    }

    // Configure GPIOs
    pin_function(obj->spi.pin_miso, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));
    pin_function(obj->spi.pin_mosi, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));
    pin_function(obj->spi.pin_sclk, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));
}

void spi_format(spi_t *obj, int bits, int mode, spi_bitorder_t order)
{
    SPI_HandleTypeDef *handle = &SpiHandle[obj->spi.module];

    // Save new values
    if (bits == 16) {
        handle->Init.DataSize = SPI_DATASIZE_16BIT;
    } else {
        handle->Init.DataSize = SPI_DATASIZE_8BIT;
    }

    switch (mode) {
        case 0:
            handle->Init.CLKPolarity = SPI_POLARITY_LOW;
            handle->Init.CLKPhase = SPI_PHASE_1EDGE;
            break;
        case 1:
            handle->Init.CLKPolarity = SPI_POLARITY_LOW;
            handle->Init.CLKPhase = SPI_PHASE_2EDGE;
            break;
        case 2:
            handle->Init.CLKPolarity = SPI_POLARITY_HIGH;
            handle->Init.CLKPhase = SPI_PHASE_1EDGE;
            break;
        default:
            handle->Init.CLKPolarity = SPI_POLARITY_HIGH;
            handle->Init.CLKPhase = SPI_PHASE_2EDGE;
            break;
    }

    if (order == SPI_MSB) {
        handle->Init.FirstBit = SPI_FIRSTBIT_MSB;
    } else {
        handle->Init.FirstBit = SPI_FIRSTBIT_LSB;
    }

    init_spi(obj);
}

void spi_frequency(spi_t *obj, int hz)
{
    SPI_HandleTypeDef *handle = &SpiHandle[obj->spi.module];

#if defined(TARGET_STM32F401RE) || defined(TARGET_STM32F401VC) || defined(TARGET_STM32F407VG)
    // Note: The frequencies are obtained with SPI1 clock = 84 MHz (APB2 clock)
    if (hz < 600000) {
        handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; // 330 kHz
    } else if ((hz >= 600000) && (hz < 1000000)) {
        handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; // 656 kHz
    } else if ((hz >= 1000000) && (hz < 2000000)) {
        handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; // 1.3 MHz
    } else if ((hz >= 2000000) && (hz < 5000000)) {
        handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; // 2.6 MHz
    } else if ((hz >= 5000000) && (hz < 10000000)) {
        handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // 5.25 MHz
    } else if ((hz >= 10000000) && (hz < 21000000)) {
        handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // 10.5 MHz
    } else if ((hz >= 21000000) && (hz < 42000000)) {
        handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; // 21 MHz
    } else { // >= 42000000
        handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; // 42 MHz
    }
#elif defined(TARGET_STM32F405RG)
    // Note: The frequencies are obtained with SPI1 clock = 48 MHz (APB2 clock)
    if (obj->spi.module == 0) {
        if (hz < 375000) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; // 187.5 kHz
        } else if ((hz >= 375000) && (hz < 750000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; // 375 kHz
        } else if ((hz >= 750000) && (hz < 1500000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; // 0.75 MHz
        } else if ((hz >= 1500000) && (hz < 3000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; // 1.5 MHz
        } else if ((hz >= 3000000) && (hz < 6000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // 3 MHz
        } else if ((hz >= 6000000) && (hz < 12000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // 6 MHz
        } else if ((hz >= 12000000) && (hz < 24000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; // 12 MHz
        } else { // >= 24000000
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; // 24 MHz
        }
    // Note: The frequencies are obtained with SPI2/3 clock = 48 MHz (APB1 clock)
    } else if ((obj->spi.module == 1) || (obj->spi.module == 2)) {
        if (hz < 375000) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; // 187.5 kHz
        } else if ((hz >= 375000) && (hz < 750000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; // 375 kHz
        } else if ((hz >= 750000) && (hz < 1500000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; // 0.75 MHz
        } else if ((hz >= 1500000) && (hz < 3000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; // 1.5 MHz
        } else if ((hz >= 3000000) && (hz < 6000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // 3 MHz
        } else if ((hz >= 6000000) && (hz < 12000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // 6 MHz
        } else if ((hz >= 12000000) && (hz < 24000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; // 12 MHz
        } else { // >= 24000000
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; // 24 MHz
        }
    }
#elif defined(TARGET_STM32F411RE) || defined(TARGET_STM32F429ZI) || defined(TARGET_STM32F439ZI)
    // Values depend of PCLK2: 100 MHz
    if ((obj->spi.module == 0) || (obj->spi.module == 3) || (obj->spi.module == 4)) {
        if (hz < 700000) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; // 391 kHz
        } else if ((hz >= 700000) && (hz < 1000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; // 781 kHz
        } else if ((hz >= 1000000) && (hz < 3000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;  // 1.56 MHz
        } else if ((hz >= 3000000) && (hz < 6000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;  // 3.13 MHz
        } else if ((hz >= 6000000) && (hz < 12000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  // 6.25 MHz
        } else if ((hz >= 12000000) && (hz < 25000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;   // 12.5 MHz
        } else if ((hz >= 25000000) && (hz < 50000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;   // 25 MHz
        } else { // >= 50000000
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;   // 50 MHz
        }
    }

    // Values depend of PCLK1: 50 MHz
    if ((obj->spi.module == 1) || (obj->spi.module == 2)) {
        if (hz < 400000) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; // 195 kHz
        } else if ((hz >= 400000) && (hz < 700000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; // 391 kHz
        } else if ((hz >= 700000) && (hz < 1000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;  // 781 MHz
        } else if ((hz >= 1000000) && (hz < 3000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;  // 1.56 MHz
        } else if ((hz >= 3000000) && (hz < 6000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  // 3.13 MHz
        } else if ((hz >= 6000000) && (hz < 12000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;   // 6.25 MHz
        } else if ((hz >= 12000000) && (hz < 25000000)) {
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;   // 12.5 MHz
        } else { // >= 25000000
            handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;   // 25 MHz
        }
    }
#endif
    init_spi(obj);
}

uint8_t spi_get_module(spi_t *obj)
{
    return obj->spi.module;
}

static inline int ssp_readable(spi_t *obj)
{
    int status;
    SPI_HandleTypeDef *handle = &SpiHandle[obj->spi.module];
    // Check if data is received
    status = ((__HAL_SPI_GET_FLAG(handle, SPI_FLAG_RXNE) != RESET) ? 1 : 0);
    return status;
}

static inline int ssp_writeable(spi_t *obj)
{
    int status;
    SPI_HandleTypeDef *handle = &SpiHandle[obj->spi.module];
    // Check if data is transmitted
    status = ((__HAL_SPI_GET_FLAG(handle, SPI_FLAG_TXE) != RESET) ? 1 : 0);
    return status;
}

static inline void ssp_write(spi_t *obj, int value)
{
    SPI_TypeDef *spi = (SPI_TypeDef *)SpiHandle[obj->spi.module].Instance;
    while (!ssp_writeable(obj));
    spi->DR = (uint16_t)value;
}

static inline int ssp_read(spi_t *obj)
{
    SPI_TypeDef *spi = (SPI_TypeDef *)SpiHandle[obj->spi.module].Instance;
    while (!ssp_readable(obj));
    return (int)spi->DR;
}

static inline int ssp_busy(spi_t *obj)
{
    int status;
    SPI_HandleTypeDef *handle = &SpiHandle[obj->spi.module];
    status = ((__HAL_SPI_GET_FLAG(handle, SPI_FLAG_BSY) != RESET) ? 1 : 0);
    return status;
}

int spi_master_write(spi_t *obj, int value)
{
    ssp_write(obj, value);
    return ssp_read(obj);
}

int spi_busy(spi_t *obj)
{
    return ssp_busy(obj);
}

#endif
