/**
******************************************************************************
* @file    MicoDriverUart.c 
* @author  William Xu
* @version V1.0.0
* @date    05-May-2014
* @brief   This file provide UART driver functions.
******************************************************************************
*
*  The MIT License
*  Copyright (c) 2014 MXCHIP Inc.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy 
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights 
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is furnished
*  to do so, subject to the following conditions:
*
*  The above copyright notice and this permission notice shall be included in
*  all copies or substantial portions of the Software.
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR 
*  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
******************************************************************************
*/ 


#include "MICORTOS.h"
#include "MICOPlatform.h"

#include "platform.h"
#include "platform_common_config.h"

#include "gpio_irq.h"

#include "chip.h"
#include "board.h"
#include "string.h"

/******************************************************
*                    Constants
******************************************************/
/* Transmit and receive ring buffers */
STATIC RINGBUFF_T txring, rxring;

/******************************************************
*                   Enumerations
******************************************************/

/******************************************************
*                 Type Definitions
******************************************************/
/* Ring buffer size */
#define UART_RB_SIZE 128

#define LPC_USART       LPC_USART0
#define LPC_IRQNUM      UART0_IRQn
#define LPC_UARTHNDLR   UART0_IRQHandler

/* Transmit and receive buffers */
static uint8_t rxbuff[UART_RB_SIZE], txbuff[UART_RB_SIZE];

/******************************************************
*                    Structures
******************************************************/

typedef struct
{
  uint32_t            rx_size;
  ring_buffer_t*      rx_buffer;
#ifndef NO_MICO_RTOS
  mico_semaphore_t    rx_complete;
  mico_semaphore_t    tx_complete;
#else
  volatile bool       rx_complete;
  volatile bool       tx_complete;
#endif
  mico_semaphore_t    sem_wakeup;
  OSStatus            tx_dma_result;
  OSStatus            rx_dma_result;
} uart_interface_t;

/******************************************************
*               Variables Definitions
******************************************************/

static uart_interface_t uart_interfaces[NUMBER_OF_UART_INTERFACES];

#ifndef NO_MICO_RTOS
static mico_uart_t current_uart;
#endif

/******************************************************
*               Function Declarations
******************************************************/

static OSStatus internal_uart_init ( mico_uart_t uart, const mico_uart_config_t* config, ring_buffer_t* optional_rx_buffer );
static OSStatus platform_uart_receive_bytes( mico_uart_t uart, void* data, uint32_t size, uint32_t timeout );


/* Interrupt service functions - called from interrupt vector table */
#ifndef NO_MICO_RTOS
static void thread_wakeup(void *arg);
static void RX_PIN_WAKEUP_handler(void *arg);
#endif

void LPC_UARTHNDLR(void)
{
  /* Want to handle any errors? Do it here. */

  /* Use default ring buffer handler. Override this with your own
     code if you need more capability. */
  Chip_UART_IRQRBHandler(LPC_USART, &rxring, &txring);
}


/******************************************************
*               Function Definitions
******************************************************/

OSStatus MicoUartInitialize( mico_uart_t uart, const mico_uart_config_t* config, ring_buffer_t* optional_rx_buffer )
{
  // #ifndef MICO_DISABLE_STDIO
  //     if (uart == STDIO_UART)
  //     {
  //         return kGeneralErr;
  //     }
  // #endif
  
  return internal_uart_init(uart, config, optional_rx_buffer);
}


OSStatus MicoStdioUartInitialize( const mico_uart_config_t* config, ring_buffer_t* optional_rx_buffer )
{
  return internal_uart_init(STDIO_UART, config, optional_rx_buffer);
}


OSStatus internal_uart_init( mico_uart_t uart, const mico_uart_config_t* config, ring_buffer_t* optional_rx_buffer )
{
#ifndef NO_MICO_RTOS
  mico_rtos_init_semaphore(&uart_interfaces[uart].tx_complete, 1);
  mico_rtos_init_semaphore(&uart_interfaces[uart].rx_complete, 1);
#else
  uart_interfaces[uart].tx_complete = false;
  uart_interfaces[uart].rx_complete = false;
#endif
  
  MicoMcuPowerSaveConfig(false);
  
  /* Enable GPIO peripheral clocks for TX and RX pins */
  Chip_Clock_SetAsyncSysconClockDiv(1);	/* divided by 1 */

  /* Configure USART TX Pin */
  /* Configure USART RX Pin */
  Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 0, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
  Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 1, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
  
#ifndef NO_MICO_RTOS
  if(config->flags & UART_WAKEUP_ENABLE){
    current_uart = uart;
    mico_rtos_init_semaphore( &uart_interfaces[uart].sem_wakeup, 1 );
    mico_rtos_create_thread(NULL, MICO_APPLICATION_PRIORITY, "UART_WAKEUP", thread_wakeup, 0x100, &current_uart);
  }
#endif
  /* Enable UART peripheral clock */

  
  /**************************************************************************
  * Initialise LPC54000 UART0 registers
  * NOTE:
  * - Both transmitter and receiver are disabled until usart_enable_transmitter/receiver is called.
  * - Only 1 and 2 stop bits are implemented at the moment.
  **************************************************************************/
  /* Setup UART */
  /* Initialise USART peripheral */
  Chip_UART_Init(LPC_USART0);
  Chip_UART_ConfigData(LPC_USART0, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
  Chip_UART_SetBaud(LPC_USART0, 115200);
 
//  /* Setup ring buffer */
//  if (optional_rx_buffer != NULL)
//  {
//    /* Note that the ring_buffer should've been initialised first */
//    uart_interfaces[uart].rx_buffer = optional_rx_buffer;
//    uart_interfaces[uart].rx_size   = 0;
//    platform_uart_receive_bytes( uart, optional_rx_buffer->buffer, optional_rx_buffer->size, 0 );
//  }
//  else
//  {
//    /* Not using ring buffer. Configure RX DMA interrupt on Cortex-M3 */
//    nvic_init_structure.NVIC_IRQChannel                   = uart_mapping[uart].rx_dma_irq;
//    nvic_init_structure.NVIC_IRQChannelPreemptionPriority = (uint8_t) 0x5;
//    nvic_init_structure.NVIC_IRQChannelSubPriority        = 0x8;
//    nvic_init_structure.NVIC_IRQChannelCmd                = ENABLE;
//    NVIC_Init( &nvic_init_structure );    /* Enable TC (transfer complete) and TE (transfer error) interrupts on source */
//  }
 
  Chip_UART_Enable(LPC_USART);
  Chip_UART_TXEnable(LPC_USART);
  
  /* Before using the ring buffers, initialize them using the ring
     buffer init function */
  RingBuffer_Init(&rxring, rxbuff, 1, UART_RB_SIZE);
  RingBuffer_Init(&txring, txbuff, 1, UART_RB_SIZE);  
  
  /* Enable receive data and line status interrupt */
  Chip_UART_IntEnable(LPC_USART, UART_INTEN_RXRDY);
  Chip_UART_IntDisable(LPC_USART, UART_INTEN_TXRDY);	/* May not be needed */  
    
  /* preemption = 1, sub-priority = 1 */
  NVIC_EnableIRQ(LPC_IRQNUM);
  
  MicoMcuPowerSaveConfig(true);
  
  return kNoErr;
}

OSStatus MicoUartFinalize( mico_uart_t uart )
{
  MicoMcuPowerSaveConfig(false);
  
  /* Disable USART */
  /* Deinitialise USART */
  Chip_UART_DeInit(LPC_USART0);

  /* Deinitialise DMA streams */

  /* Disable TC (transfer complete) interrupt at the source */

  /* Disable transmit DMA interrupt at Cortex-M3 */

  
  /* Disable DMA peripheral clocks */

  /* Disable UART interrupt vector on Cortex-M3 */

  /* Disable registers clocks */

#ifndef NO_MICO_RTOS
  mico_rtos_deinit_semaphore(&uart_interfaces[uart].rx_complete);
  mico_rtos_deinit_semaphore(&uart_interfaces[uart].tx_complete);
#endif
  
  MicoMcuPowerSaveConfig(true);
  
  return kNoErr;
}

OSStatus MicoUartSend( mico_uart_t uart, const void* data, uint32_t size )
{
//  /* Reset DMA transmission result. The result is assigned in interrupt handler */
//  uart_interfaces[uart].tx_dma_result = kGeneralErr;
//  
  MicoMcuPowerSaveConfig(false);  
//  
//  uart_mapping[uart].tx_dma_stream->CR  &= ~(uint32_t) DMA_SxCR_CIRC;
//  uart_mapping[uart].tx_dma_stream->NDTR = size;
//  uart_mapping[uart].tx_dma_stream->M0AR = (uint32_t)data;
//  
//  USART_DMACmd( uart_mapping[uart].usart, USART_DMAReq_Tx, ENABLE );
//  USART_ClearFlag( uart_mapping[uart].usart, USART_FLAG_TC );
//  DMA_Cmd( uart_mapping[uart].tx_dma_stream, ENABLE );
//  
//#ifndef NO_MICO_RTOS
//  mico_rtos_get_semaphore( &uart_interfaces[ uart ].tx_complete, MICO_NEVER_TIMEOUT );
//#else 
//  while(uart_interfaces[ uart ].tx_complete == false);
//  uart_interfaces[ uart ].tx_complete = false;
//#endif
//  
  Chip_UART_SendRB(LPC_USART0, &txring, data, size);
  MicoMcuPowerSaveConfig(true);

  return kNoErr;
}

OSStatus MicoUartRecv( mico_uart_t uart, void* data, uint32_t size, uint32_t timeout )
{
//  if (uart_interfaces[uart].rx_buffer != NULL)
//  {
    while (size != 0)
    {
      uint32_t transfer_size = MIN( UART_RB_SIZE / 2, size); // MIN(uart_interfaces[uart].rx_buffer->size / 2, size);
      
      /* Check if ring buffer already contains the required amount of data. */
      if ( transfer_size > RingBuffer_GetCount(&rxring) ) //ring_buffer_used_space( uart_interfaces[uart].rx_buffer ) )
      {
        /* Set rx_size and wait in rx_complete semaphore until data reaches rx_size or timeout occurs */
        uart_interfaces[uart].rx_size = transfer_size;
        
#ifndef NO_MICO_RTOS
        if ( mico_rtos_get_semaphore( &uart_interfaces[uart].rx_complete, timeout) != kNoErr )
        {
          uart_interfaces[uart].rx_size = 0;
          return kTimeoutErr;
        }
#else
        uart_interfaces[uart].rx_complete = false;
        int delay_start = mico_get_time_no_os();
        while(uart_interfaces[uart].rx_complete == false){
          if(mico_get_time_no_os() >= delay_start + timeout && timeout != MICO_NEVER_TIMEOUT){
            uart_interfaces[uart].rx_size = 0;
            return kTimeoutErr;
          }
        }
#endif
        
        /* Reset rx_size to prevent semaphore being set while nothing waits for the data */
        uart_interfaces[uart].rx_size = 0;
      }
     
      size -= transfer_size;
      
      // Grab data from the buffer
      do
      {
        uint8_t* available_data;
        uint32_t bytes_available;
        bytes_available = Chip_UART_ReadRB(LPC_USART, &rxring, &available_data, UART_RB_SIZE);
        //ring_buffer_get_data( uart_interfaces[uart].rx_buffer, &available_data, &bytes_available );
        bytes_available = MIN( bytes_available, transfer_size );
        memcpy( data, available_data, bytes_available );
        transfer_size -= bytes_available;
        data = ( (uint8_t*) data + bytes_available );
        //ring_buffer_consume( uart_interfaces[uart].rx_buffer, bytes_available );
      } while ( transfer_size != 0 );
    }
    
    if ( size != 0 )
    {
      return kGeneralErr;
    }
    else
    {
      return kNoErr;
    }
//  }
//  else
//  {
//    return platform_uart_receive_bytes( uart, data, size, timeout );
//  }
  
  return kNoErr;
}


static OSStatus platform_uart_receive_bytes( mico_uart_t uart, void* data, uint32_t size, uint32_t timeout )
{
//  if ( uart_interfaces[uart].rx_buffer != NULL )
//  {
//    uart_mapping[uart].rx_dma_stream->CR |= DMA_SxCR_CIRC;
//    
//    // Enabled individual byte interrupts so progress can be updated
//    USART_ITConfig( uart_mapping[uart].usart, USART_IT_RXNE, ENABLE );
//  }
//  else
//  {
//    uart_mapping[uart].rx_dma_stream->CR &= ~(uint32_t) DMA_SxCR_CIRC;
//  }
//  
//  /* Reset DMA transmission result. The result is assigned in interrupt handler */
//  uart_interfaces[uart].rx_dma_result = kGeneralErr;
//  
//  uart_mapping[uart].rx_dma_stream->NDTR = size;
//  uart_mapping[uart].rx_dma_stream->M0AR = (uint32_t)data;
//  uart_mapping[uart].rx_dma_stream->CR  |= DMA_SxCR_EN;
//  
//  
//  if ( timeout > 0 )
//  {
//#ifndef NO_MICO_RTOS
//    mico_rtos_get_semaphore( &uart_interfaces[uart].rx_complete, timeout );
//#else
//    uart_interfaces[uart].rx_complete = false;
//    int delay_start = mico_get_time_no_os();
//    while(uart_interfaces[uart].rx_complete == false){
//      if(mico_get_time_no_os() >= delay_start + timeout && timeout != MICO_NEVER_TIMEOUT){
//        break;
//      }
//    }    
//#endif
//    return uart_interfaces[uart].rx_dma_result;
//  }
//  
  
  return kNoErr;
}


uint32_t MicoUartGetLengthInBuffer( mico_uart_t uart )
{
  uint32_t temp;
  temp = RingBuffer_GetCount(&rxring);
  return temp; //ring_buffer_used_space( uart_interfaces[uart].rx_buffer );
}

#ifndef NO_MICO_RTOS
static void thread_wakeup(void *arg)
{
  mico_uart_t uart = *(mico_uart_t *)arg;
  
  while(1){
//     if(mico_rtos_get_semaphore(&uart_interfaces[ uart ].sem_wakeup, 1000) != kNoErr){
//      gpio_irq_enable(uart_mapping[uart].pin_rx->bank, uart_mapping[uart].pin_rx->number, IRQ_TRIGGER_FALLING_EDGE, RX_PIN_WAKEUP_handler, &uart);
//      MicoMcuPowerSaveConfig(true);
//    }   
  }
}
#endif

/******************************************************
*            Interrupt Service Routines
******************************************************/
#ifndef NO_MICO_RTOS
void RX_PIN_WAKEUP_handler(void *arg)
{
  (void)arg;
  mico_uart_t uart = *(mico_uart_t *)arg;
  
//  RCC_AHB1PeriphClockCmd(uart_mapping[ uart ].pin_rx->peripheral_clock, ENABLE);
//  uart_mapping[ uart ].usart_peripheral_clock_func ( uart_mapping[uart].usart_peripheral_clock,  ENABLE );
//  uart_mapping[uart].rx_dma_peripheral_clock_func  ( uart_mapping[uart].rx_dma_peripheral_clock, ENABLE );
//  
//  gpio_irq_disable(uart_mapping[uart].pin_rx->bank, uart_mapping[uart].pin_rx->number); 
  
  MicoMcuPowerSaveConfig(false);
  mico_rtos_set_semaphore(&uart_interfaces[uart].sem_wakeup);
}
#endif

// end file






