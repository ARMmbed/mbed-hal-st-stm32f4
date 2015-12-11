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
#include "mbed-drivers/mbed_assert.h"
#include "serial_api.h"

#if DEVICE_SERIAL

#include "cmsis.h"
#include <stdbool.h>
#include "pinmap.h"
#include <string.h>
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

#ifndef YOTTA_CFG_MBED_OS_STDIO_DEFAULT_BAUD
#   define YOTTA_CFG_MBED_OS_STDIO_DEFAULT_BAUD 9600
#endif

#define UART_NUM (8)

static UART_HandleTypeDef UartHandle[UART_NUM];
static const IRQn_Type UartIRQs[UART_NUM] = {
    USART1_IRQn,
    USART2_IRQn,
#if defined(USART3_BASE)
    USART3_IRQn,
#else
    0,
#endif
#if defined(UART4_BASE)
    UART4_IRQn,
#else
    0,
#endif
#if defined(UART5_BASE)
    UART5_IRQn,
#else
    0,
#endif
    USART6_IRQn,
#if defined(UART7_BASE)
    UART7_IRQn,
#else
    0,
#endif
#if defined(UART8_BASE)
    UART8_IRQn,
#else
    0,
#endif
};

static uint32_t serial_irq_ids[UART_NUM] = {0, 0, 0, 0, 0, 0, 0, 0};
static uart_irq_handler irq_handlers[UART_NUM] = {0, 0, 0, 0, 0, 0, 0, 0};

int stdio_uart_inited = 0;
serial_t stdio_uart;

void serial_init(serial_t *obj, PinName tx, PinName rx)
{
    // Determine the UART to use (UART_1, UART_2, ...)
    UARTName uart_tx = (UARTName)pinmap_peripheral(tx, PinMap_UART_TX);
    UARTName uart_rx = (UARTName)pinmap_peripheral(rx, PinMap_UART_RX);

    // Get the peripheral name (UART_1, UART_2, ...) from the pin and assign it to the object
    UARTName instance = (UARTName)pinmap_merge(uart_tx, uart_rx);
    MBED_ASSERT(instance != (UARTName)NC);

    // Enable USART clock
    switch (instance) {
        case UART_1:
            __USART1_CLK_ENABLE();
            obj->serial.module = 0;
            break;
        case UART_2:
            __USART2_CLK_ENABLE();
            obj->serial.module = 1;
            break;
#if defined(USART3_BASE)
        case UART_3:
            __USART3_CLK_ENABLE();
            obj->serial.module = 2;
            break;
#endif
#if defined(UART4_BASE)
        case UART_4:
            __UART4_CLK_ENABLE();
            obj->serial.module = 3;
            break;
#endif
#if defined(UART5_BASE)
        case UART_5:
            __UART5_CLK_ENABLE();
            obj->serial.module = 4;
            break;
#endif
        case UART_6:
            __USART6_CLK_ENABLE();
            obj->serial.module = 5;
            break;
#if defined(UART7_BASE)
        case UART_7:
            __UART7_CLK_ENABLE();
            obj->serial.module = 6;
            break;
#endif
#if defined(UART8_BASE)
        case UART_8:
            __UART8_CLK_ENABLE();
            obj->serial.module = 7;
            break;
#endif
    }

    // Configure the UART pins
    pinmap_pinout(tx, PinMap_UART_TX);
    pinmap_pinout(rx, PinMap_UART_RX);
    if (tx != NC) pin_mode(tx, PullUp);
    if (rx != NC) pin_mode(rx, PullUp);
    obj->serial.pin_tx = tx;
    obj->serial.pin_rx = rx;

    // initialize the handle for this master!
    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];

    handle->Instance            = (USART_TypeDef *)instance;
    handle->Init.BaudRate       = 9600;
    handle->Init.WordLength     = UART_WORDLENGTH_8B;
    handle->Init.StopBits       = UART_STOPBITS_1;
    handle->Init.Parity         = UART_PARITY_NONE;
    if (rx == NC) {
        handle->Init.Mode = UART_MODE_TX;
    } else if (tx == NC) {
        handle->Init.Mode = UART_MODE_RX;
    } else {
        handle->Init.Mode = UART_MODE_TX_RX;
    }
    handle->Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    handle->Init.OverSampling   = UART_OVERSAMPLING_16;
    handle->TxXferCount         = 0;
    handle->RxXferCount         = 0;

    if (tx == STDIO_UART_TX && rx == STDIO_UART_RX) {
        handle->Init.BaudRate   = YOTTA_CFG_MBED_OS_STDIO_DEFAULT_BAUD;
    }

    HAL_UART_Init(handle);

    // For stdio management
    if (tx == STDIO_UART_TX && rx == STDIO_UART_RX) {
        stdio_uart_inited = 1;
        memcpy(&stdio_uart, obj, sizeof(serial_t));
    }

    // DEBUG_PRINTF("UART%u: Init\n", obj->serial.module+1);
}

void serial_free(serial_t *obj)
{
    // Reset UART and disable clock
    switch (obj->serial.module) {
        case 0:
            __USART1_FORCE_RESET();
            __USART1_RELEASE_RESET();
            __USART1_CLK_DISABLE();
            break;
        case 1:
            __USART2_FORCE_RESET();
            __USART2_RELEASE_RESET();
            __USART2_CLK_DISABLE();
            break;
#if defined(USART3_BASE)
        case 2:
            __USART3_FORCE_RESET();
            __USART3_RELEASE_RESET();
            __USART3_CLK_DISABLE();
            break;
#endif
#if defined(UART4_BASE)
        case 3:
            __UART4_FORCE_RESET();
            __UART4_RELEASE_RESET();
            __UART4_CLK_DISABLE();
            break;
#endif
#if defined(UART5_BASE)
        case 4:
            __UART5_FORCE_RESET();
            __UART5_RELEASE_RESET();
            __UART5_CLK_DISABLE();
            break;
#endif
        case 5:
            __USART6_FORCE_RESET();
            __USART6_RELEASE_RESET();
            __USART6_CLK_DISABLE();
            break;
#if defined(UART7_BASE)
        case 6:
            __UART7_FORCE_RESET();
            __UART7_RELEASE_RESET();
            __UART7_CLK_DISABLE();
            break;
#endif
#if defined(UART8_BASE)
        case 7:
            __UART8_FORCE_RESET();
            __UART8_RELEASE_RESET();
            __UART8_CLK_DISABLE();
            break;
#endif
    }
    // Configure GPIOs
    pin_function(obj->serial.pin_tx, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));
    pin_function(obj->serial.pin_rx, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));

    DEBUG_PRINTF("UART%u: Free\n", obj->serial.module+1);
}

void serial_baud(serial_t *obj, int baudrate)
{
    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];
    handle->Init.BaudRate = baudrate;

    HAL_UART_Init(handle);

    DEBUG_PRINTF("UART%u: Baudrate: %u\n", obj->serial.module+1, baudrate);
}

void serial_format(serial_t *obj, int data_bits, SerialParity parity, int stop_bits)
{
    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];

    if (data_bits > 8) {
        handle->Init.WordLength = UART_WORDLENGTH_9B;
    } else {
        handle->Init.WordLength = UART_WORDLENGTH_8B;
    }

    switch (parity) {
        case ParityOdd:
            handle->Init.Parity = UART_PARITY_ODD;
            break;
        case ParityEven:
            handle->Init.Parity = UART_PARITY_EVEN;
            break;
        default: // ParityNone
        case ParityForced0: // unsupported!
        case ParityForced1: // unsupported!
            handle->Init.Parity = UART_PARITY_NONE;
            break;
    }

    if (stop_bits == 2) {
        handle->Init.StopBits = UART_STOPBITS_2;
    } else {
        handle->Init.StopBits = UART_STOPBITS_1;
    }

    HAL_UART_Init(handle);

    DEBUG_PRINTF("UART%u: Format: %u, %u, %u\n", obj->serial.module+1, data_bits, parity, stop_bits);
}

/******************************************************************************
 * INTERRUPTS HANDLING
 ******************************************************************************/

static void uart_irq(uint8_t id)
{
    UART_HandleTypeDef *handle = &UartHandle[id];

    if (serial_irq_ids[id] != 0) {
        if (__HAL_UART_GET_FLAG(handle, UART_FLAG_TC) != RESET) {
            irq_handlers[id](serial_irq_ids[id], TxIrq);
            __HAL_UART_CLEAR_FLAG(handle, UART_FLAG_TC);
        }
        if (__HAL_UART_GET_FLAG(handle, UART_FLAG_RXNE) != RESET) {
            irq_handlers[id](serial_irq_ids[id], RxIrq);
            __HAL_UART_CLEAR_FLAG(handle, UART_FLAG_RXNE);
        }
    }
}

static void uart1_irq(void)
{
    uart_irq(0);
}

static void uart2_irq(void)
{
    uart_irq(1);
}

#if defined(USART3_BASE)
static void uart3_irq(void)
{
    uart_irq(2);
}
#endif

#if defined(UART4_BASE)
static void uart4_irq(void)
{
    uart_irq(3);
}
#endif

#if defined(UART5_BASE)
static void uart5_irq(void)
{
    uart_irq(4);
}
#endif

static void uart6_irq(void)
{
    uart_irq(5);
}

#if defined(UART7_BASE)
static void uart7_irq(void)
{
    uart_irq(6);
}
#endif

#if defined(UART8_BASE)
static void uart8_irq(void)
{
    uart_irq(7);
}
#endif

static const uint32_t uart_irq_vectors[UART_NUM] = {
    (uint32_t)&uart1_irq,
    (uint32_t)&uart2_irq,
#if defined(USART3_BASE)
    (uint32_t)&uart3_irq,
#else
    0,
#endif
#if defined(UART4_BASE)
    (uint32_t)&uart4_irq,
#else
    0,
#endif
#if defined(UART5_BASE)
    (uint32_t)&uart5_irq,
#else
    0,
#endif
    (uint32_t)&uart6_irq,
#if defined(UART7_BASE)
    (uint32_t)&uart7_irq,
#else
    0,
#endif
#if defined(UART8_BASE)
    (uint32_t)&uart8_irq
#else
    0,
#endif
};

void serial_irq_handler(serial_t *obj, uart_irq_handler handler, uint32_t id)
{
    irq_handlers[obj->serial.module] = handler;
    serial_irq_ids[obj->serial.module] = id;
}

void serial_irq_set(serial_t *obj, SerialIrq irq, uint32_t enable)
{
    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];
    IRQn_Type irq_n = UartIRQs[obj->serial.module];
    uint32_t vector = uart_irq_vectors[obj->serial.module];

    if (!irq_n || !vector)
        return;

    if (enable) {

        if (irq == RxIrq) {
            __HAL_UART_ENABLE_IT(handle, UART_IT_RXNE);
        } else { // TxIrq
            __HAL_UART_ENABLE_IT(handle, UART_IT_TC);
        }

        vIRQ_SetVector(irq_n, vector);
        vIRQ_EnableIRQ(irq_n);

    } else { // disable

        int all_disabled = 0;

        if (irq == RxIrq) {
            __HAL_UART_DISABLE_IT(handle, UART_IT_RXNE);
            // Check if TxIrq is disabled too
            if ((handle->Instance->CR1 & USART_CR1_TXEIE) == 0) all_disabled = 1;
        } else { // TxIrq
            __HAL_UART_DISABLE_IT(handle, UART_IT_TXE);
            // Check if RxIrq is disabled too
            if ((handle->Instance->CR1 & USART_CR1_RXNEIE) == 0) all_disabled = 1;
        }

        if (all_disabled) vIRQ_DisableIRQ(irq_n);

    }
}

/******************************************************************************
 * READ/WRITE
 ******************************************************************************/

int serial_getc(serial_t *obj)
{
    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];
    while (!serial_readable(obj));
    return (int)(handle->Instance->DR & 0x1FF);
}

void serial_putc(serial_t *obj, int c)
{
    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];
    while (!serial_writable(obj));
    handle->Instance->DR = (uint32_t)(c & 0x1FF);
}

int serial_readable(serial_t *obj)
{
    int status;
    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];
    // Check if data is received
    status = ((__HAL_UART_GET_FLAG(handle, UART_FLAG_RXNE) != RESET) ? 1 : 0);
    return status;
}

int serial_writable(serial_t *obj)
{
    int status;
    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];
    // Check if data is transmitted
    status = ((__HAL_UART_GET_FLAG(handle, UART_FLAG_TXE) != RESET) ? 1 : 0);
    return status;
}

void serial_clear(serial_t *obj)
{
    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];
    __HAL_UART_CLEAR_FLAG(handle, UART_FLAG_TXE);
    __HAL_UART_CLEAR_FLAG(handle, UART_FLAG_RXNE);
}

void serial_pinout_tx(PinName tx)
{
    pinmap_pinout(tx, PinMap_UART_TX);
}

void serial_break_set(serial_t *obj)
{
    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];
    HAL_LIN_SendBreak(handle);
}

void serial_break_clear(serial_t *obj)
{
    (void)obj;
}

int serial_tx_asynch(serial_t *obj, void *tx, size_t tx_length, uint8_t tx_width, uint32_t handler, uint32_t event, DMAUsage hint)
{
    // TODO: DMA usage is currently ignored
    (void) hint;

    bool use_tx = (tx != NULL && tx_length > 0);
    IRQn_Type irq_n = UartIRQs[obj->serial.module];

    if (!use_tx || !irq_n)
        return 0;

    obj->tx_buff.buffer = tx;
    obj->tx_buff.length = tx_length;
    obj->tx_buff.pos    = 0;
    obj->tx_buff.width  = tx_width;

    obj->serial.event   = (obj->serial.event & ~SERIAL_EVENT_TX_MASK) | (event & SERIAL_EVENT_TX_MASK);

    // register the thunking handler
    vIRQ_SetVector(irq_n, handler);
    vIRQ_EnableIRQ(irq_n);

    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];
    // HAL_StatusTypeDef rc = HAL_UART_Transmit_IT(handle, tx, tx_length);

    // manually implemented HAL_UART_Transmit_IT for tighter control of what it does
    handle->pTxBuffPtr = tx;
    handle->TxXferSize = tx_length;
    handle->TxXferCount = tx_length;

    if(handle->State == HAL_UART_STATE_BUSY_RX) {
        handle->State = HAL_UART_STATE_BUSY_TX_RX;
    } else {
        handle->State = HAL_UART_STATE_BUSY_TX;
    }

    // if the TX register is empty, directly input the first transmit byte
    if (__HAL_UART_GET_FLAG(handle, UART_FLAG_TXE)) {
        handle->Instance->DR = *handle->pTxBuffPtr++;
        handle->TxXferCount--;
    }
    // chose either the tx reg empty or if last byte wait directly for tx complete
    if (handle->TxXferCount != 0) {
        handle->Instance->CR1 |= USART_CR1_TXEIE;
    } else {
        handle->Instance->CR1 |= USART_CR1_TCIE;
    }

    DEBUG_PRINTF("UART%u: Tx: 0=(%u, %u) %x\n", obj->serial.module+1, tx_length, tx_width, HAL_UART_GetState(handle));

    return tx_length;
}

void serial_rx_asynch(serial_t *obj, void *rx, size_t rx_length, uint8_t rx_width, uint32_t handler, uint32_t event, uint8_t char_match, DMAUsage hint)
{
    // TODO: DMA usage is currently ignored
    (void) hint;

    bool use_rx = (rx != NULL && rx_length > 0);
    IRQn_Type irq_n = UartIRQs[obj->serial.module];

    if (!use_rx || !irq_n)
        return;

    obj->rx_buff.buffer = rx;
    obj->rx_buff.length = rx_length;
    obj->rx_buff.pos    = 0;
    obj->rx_buff.width  = rx_width;

    obj->serial.event      = (obj->serial.event & ~SERIAL_EVENT_RX_MASK) | (event & SERIAL_EVENT_RX_MASK);
    obj->serial.char_match = char_match;

    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];
    // register the thunking handler
    vIRQ_SetVector(irq_n, handler);
    vIRQ_EnableIRQ(irq_n);

    // HAL_StatusTypeDef rc = HAL_UART_Receive_IT(handle, rx, rx_length);

    handle->pRxBuffPtr = rx;
    handle->RxXferSize = rx_length;
    handle->RxXferCount = rx_length;

    if(handle->State == HAL_UART_STATE_BUSY_TX) {
        handle->State = HAL_UART_STATE_BUSY_TX_RX;
    } else {
        handle->State = HAL_UART_STATE_BUSY_RX;
    }

    __HAL_UART_CLEAR_PEFLAG(handle);
    handle->Instance->CR1 |= USART_CR1_RXNEIE | USART_CR1_PEIE;
    handle->Instance->CR3 |= USART_CR3_EIE;

    DEBUG_PRINTF("UART%u: Rx: 0=(%u, %u, %u) %x\n", obj->serial.module+1, rx_length, rx_width, char_match, HAL_UART_GetState(handle));
}

int serial_irq_handler_asynch(serial_t *obj)
{
    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];

    int status = handle->Instance->SR;
    int data = handle->Instance->DR;
    int event = 0;

    if (status & USART_SR_PE) {
        event |= SERIAL_EVENT_RX_PARITY_ERROR;
    }
    if (status & (USART_SR_NE | USART_SR_FE)) {
        event |= SERIAL_EVENT_RX_FRAMING_ERROR;
    }
    if (status & USART_SR_ORE) {
        event |= SERIAL_EVENT_RX_OVERRUN_ERROR;
    }

    if ((status & USART_SR_TC) && (handle->State & 0x10) && !handle->TxXferCount) {
        // transmission is finally complete
        handle->Instance->CR1 &= ~USART_CR1_TCIE;
        // set event tx complete
        event |= SERIAL_EVENT_TX_COMPLETE;
        // update handle state
        if(handle->State == HAL_UART_STATE_BUSY_TX_RX) {
            handle->State = HAL_UART_STATE_BUSY_RX;
        } else {
            handle->State = HAL_UART_STATE_READY;
        }
    }
    else if ((status & USART_SR_TXE) && handle->TxXferCount) {
        // chose either the tx reg empty or if last byte wait directly for tx complete
        if (--handle->TxXferCount == 0) {
            handle->Instance->CR1 &= ~USART_CR1_TXEIE;
            handle->Instance->CR1 |= USART_CR1_TCIE;
        }
        // copy new data into transmit register
        handle->Instance->DR = (uint8_t)*handle->pTxBuffPtr++;
        obj->tx_buff.pos++;
    }

    if ((status & USART_SR_RXNE) && handle->RxXferCount) {
        // something arrived in the receive buffer
        // copy into buffer
        *handle->pRxBuffPtr++ = (uint8_t)data;
        obj->rx_buff.pos++;
        // check for character match only if enabled though!
        if ((obj->serial.char_match != SERIAL_RESERVED_CHAR_MATCH) && ((uint8_t)data == obj->serial.char_match)) {
            event |= SERIAL_EVENT_RX_CHARACTER_MATCH;
        }
        if (--handle->RxXferCount == 0) {
            // last receive byte, disable all rx interrupts
            handle->Instance->CR1 &= ~(USART_CR1_RXNEIE | USART_CR1_PEIE);
            handle->Instance->CR3 &= ~USART_CR3_EIE;
            // set event rx complete
            event |= SERIAL_EVENT_RX_COMPLETE;
            // update handle state
            if(handle->State == HAL_UART_STATE_BUSY_TX_RX) {
                handle->State = HAL_UART_STATE_BUSY_TX;
            } else {
                handle->State = HAL_UART_STATE_READY;
            }
        }
    }

    return (event & obj->serial.event);
}

void serial_rx_abort_asynch(serial_t *obj)
{
    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];
    // stop interrupts
    handle->Instance->CR1 &= ~(USART_CR1_RXNEIE | USART_CR1_PEIE);
    handle->Instance->CR3 &= ~USART_CR3_EIE;
    // clear flags
    __HAL_UART_CLEAR_PEFLAG(handle);
    // reset states
    handle->RxXferCount = 0;
    // update handle state
    if(handle->State == HAL_UART_STATE_BUSY_TX_RX) {
        handle->State = HAL_UART_STATE_BUSY_TX;
    } else {
        handle->State = HAL_UART_STATE_READY;
    }
}

void serial_tx_abort_asynch(serial_t *obj)
{
    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];
    // stop interrupts
    handle->Instance->CR1 &= ~(USART_CR1_TCIE | USART_CR1_TXEIE);
    // clear flags
    __HAL_UART_CLEAR_PEFLAG(handle);
    // reset states
    handle->TxXferCount = 0;
    // update handle state
    if(handle->State == HAL_UART_STATE_BUSY_TX_RX) {
        handle->State = HAL_UART_STATE_BUSY_RX;
    } else {
        handle->State = HAL_UART_STATE_READY;
    }
}

uint8_t serial_tx_active(serial_t *obj)
{
    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];
    HAL_UART_StateTypeDef state = HAL_UART_GetState(handle);

    switch(state) {
        case HAL_UART_STATE_RESET:
        case HAL_UART_STATE_READY:
        case HAL_UART_STATE_ERROR:
        case HAL_UART_STATE_TIMEOUT:
        case HAL_UART_STATE_BUSY_RX:
            return 0;
        default:
            return 1;
    }
}

uint8_t serial_rx_active(serial_t *obj)
{
    UART_HandleTypeDef *handle = &UartHandle[obj->serial.module];
    HAL_UART_StateTypeDef state = HAL_UART_GetState(handle);

    switch(state) {
        case HAL_UART_STATE_RESET:
        case HAL_UART_STATE_READY:
        case HAL_UART_STATE_ERROR:
        case HAL_UART_STATE_TIMEOUT:
        case HAL_UART_STATE_BUSY_TX:
            return 0;
        default:
            return 1;
    }
}


#endif
