/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2015, STMicroelectronics
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
#include "mbed-drivers/mbed_assert.h"
#include "spi_api.h"

#if DEVICE_SPI

#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "cmsis.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "target_config.h"

#define DEBUG_STDIO 0

#ifndef DEBUG_STDIO
#   define DEBUG_STDIO 0
#endif

#if DEBUG_STDIO
#   include <stdio.h>
#   define DEBUG_PRINTF(...) do { printf(__VA_ARGS__); } while(0)
#else
#   define DEBUG_PRINTF(...) {}
#endif

typedef enum {
    SPI_TRANSFER_TYPE_NONE = 0,
    SPI_TRANSFER_TYPE_TX = 1,
    SPI_TRANSFER_TYPE_RX = 2,
    SPI_TRANSFER_TYPE_TXRX = 3,
} transfer_type_t;


// Allow for backward compatibility
#ifndef MODULES_SIZE_SPI
#define MODULES_SIZE_SPI MODULE_SIZE_SPI
#endif

static SPI_HandleTypeDef SpiHandle[MODULES_SIZE_SPI];
static const IRQn_Type SpiIRQs[MODULES_SIZE_SPI] = {
    SPI1_IRQn,
    SPI2_IRQn,
    SPI3_IRQn,
#if MODULES_SIZE_SPI > 3
    SPI4_IRQn,
#endif
#if MODULES_SIZE_SPI > 4
    SPI5_IRQn,
#endif
#if MODULES_SIZE_SPI > 5
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
    switch(instance) {
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
#if MODULES_SIZE_SPI > 3
        case SPI_4:
            __SPI4_CLK_ENABLE();
            obj->spi.module = 3;
            break;
#endif
#if MODULES_SIZE_SPI > 4
        case SPI_5:
            __SPI5_CLK_ENABLE();
            obj->spi.module = 4;
            break;
#endif
#if MODULES_SIZE_SPI > 5
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

    DEBUG_PRINTF("SPI%u: Init\n", obj->spi.module+1);

    init_spi(obj);
}

void spi_free(spi_t *obj)
{
    // Reset SPI and disable clock
    switch(obj->spi.module) {
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
#if MODULES_SIZE_SPI > 3
        case 3:
            __SPI4_FORCE_RESET();
            __SPI4_RELEASE_RESET();
            __SPI4_CLK_DISABLE();
            break;
#endif
#if MODULES_SIZE_SPI > 4
        case 4:
            __SPI5_FORCE_RESET();
            __SPI5_RELEASE_RESET();
            __SPI5_CLK_DISABLE();
            break;
#endif
#if MODULES_SIZE_SPI > 5
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

    DEBUG_PRINTF("SPI%u: Free\n", obj->spi.module+1);
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

    DEBUG_PRINTF("SPI%u: Format: %u, %u, %u\n", obj->spi.module+1, bits, mode, order);

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

    DEBUG_PRINTF("SPI%u: Frequency: %u, 0x%x\n", obj->spi.module+1, hz, (unsigned int)handle->Init.BaudRatePrescaler);

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

/// @returns the number of bytes transferred, or `0` if nothing transferred
static int spi_master_start_asynch_transfer(spi_t *obj, transfer_type_t transfer_type, void *tx, void *rx, size_t length)
{
    if (transfer_type != SPI_TRANSFER_TYPE_TX) DEBUG_PRINTF("SPI%u: Start: %u, %u\n", obj->spi.module+1, transfer_type, length);
    SPI_HandleTypeDef *handle = &SpiHandle[obj->spi.module];
    obj->spi.transfer_type = transfer_type;

    bool is16bit = (handle->Init.DataSize == SPI_DATASIZE_16BIT);
    // the HAL expects number of transfers instead of number of bytes
    // so for 16 bit transfer width the count needs to be halved
    size_t words;
    if (is16bit) words = length / 2;
    else         words = length;

    // enable the interrupt
    IRQn_Type irq_n = SpiIRQs[obj->spi.module];
    vIRQ_EnableIRQ(irq_n);

    // enable the right hal transfer
    static uint16_t sink;
    int rc = 0;
    switch(transfer_type) {
        case SPI_TRANSFER_TYPE_TXRX:
            rc = HAL_SPI_TransmitReceive_IT(handle, (uint8_t*)tx, (uint8_t*)rx, words);
            break;
        case SPI_TRANSFER_TYPE_TX:
            // TODO: we do not use `HAL_SPI_Transmit_IT`, since it has some unknown bug
            // and makes the HAL keep some state and then that fails successive transfers
            // rc = HAL_SPI_Transmit_IT(handle, (uint8_t*)tx, words);
            rc = HAL_SPI_TransmitReceive_IT(handle, (uint8_t*)tx, (uint8_t*)&sink, 1);
            length = is16bit ? 2 : 1;
            break;
        case SPI_TRANSFER_TYPE_RX:
            // the receive function also "transmits" the receive buffer so in order
            // to guarantee that 0xff is on the line, we explicitly memset it here
            memset(rx, SPI_FILL_WORD, length);
            rc = HAL_SPI_Receive_IT(handle, (uint8_t*)rx, words);
            break;
        default:
            length = 0;
    }

    if (rc) {
        DEBUG_PRINTF("SPI%u: RC=%u\n", obj->spi.module+1, rc);
        length = 0;
    }

    return length;
}

// asynchronous API
void spi_master_transfer(spi_t *obj, void *tx, size_t tx_length, void *rx, size_t rx_length, uint32_t handler, uint32_t event, DMAUsage hint)
{
    // TODO: DMA usage is currently ignored
    (void) hint;

    // check which use-case we have
    bool use_tx = (tx != NULL && tx_length > 0);
    bool use_rx = (rx != NULL && rx_length > 0);

    // don't do anything, if the buffers aren't valid
    if (!use_tx && !use_rx)
        return;

    SPI_HandleTypeDef *handle = &SpiHandle[obj->spi.module];

    bool is16bit = (handle->Init.DataSize == SPI_DATASIZE_16BIT);

    // copy the buffers to the SPI object
    obj->tx_buff.buffer = tx;
    obj->tx_buff.length = tx_length;
    obj->tx_buff.pos = 0;
    obj->tx_buff.width = is16bit ? 16 : 8;

    obj->rx_buff.buffer = rx;
    obj->rx_buff.length = rx_length;
    obj->rx_buff.pos = 0;
    obj->rx_buff.width = obj->tx_buff.width;

    obj->spi.event = event;

    DEBUG_PRINTF("SPI%u: Transfer: %u, %u\n", obj->spi.module+1, tx_length, rx_length);

    // register the thunking handler
    IRQn_Type irq_n = SpiIRQs[obj->spi.module];
    vIRQ_SetVector(irq_n, handler);

    // enable the right hal transfer
    if (use_tx && use_rx) {
        // transfer with the min(tx, rx), then later either transmit _or_ receive the remainder
        size_t size = (tx_length < rx_length)? tx_length : rx_length;
        spi_master_start_asynch_transfer(obj, SPI_TRANSFER_TYPE_TXRX, tx, rx, size);
    } else if (use_tx) {
        spi_master_start_asynch_transfer(obj, SPI_TRANSFER_TYPE_TX, tx, NULL, tx_length);
    } else if (use_rx) {
        spi_master_start_asynch_transfer(obj, SPI_TRANSFER_TYPE_RX, NULL, rx, rx_length);
    }
}

uint32_t spi_irq_handler_asynch(spi_t *obj)
{
    // use the right instance
    SPI_HandleTypeDef *handle = &SpiHandle[obj->spi.module];
    int event = 0;

    // call the CubeF4 handler, this will update the handle
    HAL_SPI_IRQHandler(handle);

    if (HAL_SPI_GetState(handle) == HAL_SPI_STATE_READY) {
        // adjust buffer positions
        size_t tx_size = (handle->TxXferSize - handle->TxXferCount);
        size_t rx_size = (handle->RxXferSize - handle->RxXferCount);
        // 16 bit transfers need to be doubled to get bytes
        if (handle->Init.DataSize == SPI_DATASIZE_16BIT) {
            tx_size *= 2;
            rx_size *= 2;
        }
        // adjust buffer positions
        if (obj->spi.transfer_type != SPI_TRANSFER_TYPE_RX) {
            obj->tx_buff.pos += tx_size;
        }
        if (obj->spi.transfer_type != SPI_TRANSFER_TYPE_TX) {
            obj->rx_buff.pos += rx_size;
        }

        if (handle->TxXferCount > 0) DEBUG_PRINTF("SPI%u: TxXferCount: %u\n", obj->spi.module+1, handle->TxXferCount);
        if (handle->RxXferCount > 0) DEBUG_PRINTF("SPI%u: RxXferCount: %u\n", obj->spi.module+1, handle->RxXferCount);

        int error = HAL_SPI_GetError(handle);
        if(error != HAL_SPI_ERROR_NONE) {
            // something went wrong and the transfer has definitely completed
            event = SPI_EVENT_ERROR | SPI_EVENT_INTERNAL_TRANSFER_COMPLETE;

            if (error & HAL_SPI_ERROR_OVR) {
                // buffer overrun
                event |= SPI_EVENT_RX_OVERFLOW;
            }
        } else {
            // figure out if we need to transfer more data:
            if (obj->tx_buff.pos < obj->tx_buff.length) {
                // DEBUG_PRINTF("t%u ", obj->tx_buff.pos);
                // we need to transfer more data
                spi_master_start_asynch_transfer(obj, SPI_TRANSFER_TYPE_TX,
                    obj->tx_buff.buffer + obj->tx_buff.pos,     // offset the initial buffer by the position
                    NULL,                                       // there is no receive buffer
                    obj->tx_buff.length - obj->tx_buff.pos);    // transfer the remaining bytes only
            } else if (obj->rx_buff.pos < obj->rx_buff.length) {
                // DEBUG_PRINTF("r%u ", obj->rx_buff.pos);
                // we need to receive more data
                spi_master_start_asynch_transfer(obj, SPI_TRANSFER_TYPE_RX,
                    NULL,                                       // there is no transmit buffer
                    obj->rx_buff.buffer + obj->rx_buff.pos,     // offset the initial buffer by the position
                    obj->rx_buff.length - obj->rx_buff.pos);    // transfer one byte at a time, until we received everything
            } else {
                // everything is ok, nothing else needs to be transferred
                event = SPI_EVENT_COMPLETE | SPI_EVENT_INTERNAL_TRANSFER_COMPLETE;
                DEBUG_PRINTF("SPI%u: Done: %u, %u\n", obj->spi.module+1, obj->tx_buff.pos, obj->rx_buff.pos);
            }
        }
    }

    if (event) DEBUG_PRINTF("SPI%u: Event: 0x%x\n", obj->spi.module+1, event);

    return (event & (obj->spi.event | SPI_EVENT_INTERNAL_TRANSFER_COMPLETE));
}

uint8_t spi_active(spi_t *obj)
{
    SPI_HandleTypeDef *handle = &SpiHandle[obj->spi.module];
    HAL_SPI_StateTypeDef state = HAL_SPI_GetState(handle);

    switch(state) {
        case HAL_SPI_STATE_RESET:
        case HAL_SPI_STATE_READY:
        case HAL_SPI_STATE_ERROR:
            return 0;
        default:
            return -1;
    }
}

void spi_abort_asynch(spi_t *obj)
{
    // diable interrupt
    vIRQ_DisableIRQ(SpiIRQs[obj->spi.module]);

    // clean-up
    SPI_HandleTypeDef *handle = &SpiHandle[obj->spi.module];
    __HAL_SPI_DISABLE(handle);
    HAL_SPI_DeInit(handle);
    HAL_SPI_Init(handle);
    __HAL_SPI_ENABLE(handle);
}

#endif
