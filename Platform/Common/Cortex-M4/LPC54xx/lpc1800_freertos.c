/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
/* lwIP includes. */
#include <stdio.h>
#include "MicoRtos.h"
//#include "a_types.h"
//#include "FREERTOS.h"
//#include "sys_arch.h"
#include "board.h"
#include "string.h"
#include "stdio.h"

#define ATH_SPI_DMA             1 
///* Number of memory operations to use for application */
//#define NUMBER_TRANSFER_OPS     50000

/* Size of the source and destination buffers in 32-bit words.
   Allowable values  = 126, 256, 512, or 1024 */
//#define SIZE_BUFFERS            (512)

/* Source and destination buffers */
//uint16_t src[SIZE_BUFFERS*2], dst[SIZE_BUFFERS*2];

/* DMA completion flag */
volatile uint8_t bDMASPITXDoneFlag = false;
volatile uint8_t bDMASPIRXDoneFlag = false;

DMA_CHDESC_T dmaSPITxDesc, dmaSPIRxDesc;

//#define SEMAPHORE 1
#define SEMDELAY 100
//#define SPIDEBUG 1
static mico_semaphore_t spi_transfer_finished_semaphore;

extern void spi_irq_handler( );

/**
 * @brief	DMA Interrupt Handler
 * @return	None
 */
/*****************************************************************************
 * Public functiion
*****************************************************************************/
/**
 * @brief	DMA Interrupt Handler
 * @return	None
 */
extern void uart0_rx_dma_handler(void);
extern void uart0_tx_dma_handler(void);

extern void spi_cs_Set(uint8_t status);
void DMA_IRQHandler(void)
{
//  /* Rrror interrupt on channel 0? */
//  if ((Chip_DMA_GetIntStatus(LPC_DMA) & DMA_INTSTAT_ACTIVEERRINT) != 0) {
//    /* This shouldn't happen for this simple DMA example, so set the LED
//    to indicate an error occurred. This is the correct method to clear
//    an abort. */
//    Chip_DMA_DisableChannel(LPC_DMA, DMAREQ_SPI0_TX);
//    while ((Chip_DMA_GetBusyChannels(LPC_DMA) & (1 << DMAREQ_SPI0_TX)) != 0) {}
//    Chip_DMA_AbortChannel(LPC_DMA, DMAREQ_SPI0_TX);
//    Chip_DMA_ClearErrorIntChannel(LPC_DMA, DMAREQ_SPI0_TX);
//    Chip_DMA_EnableChannel(LPC_DMA, DMAREQ_SPI0_TX);
//    
//    Chip_DMA_DisableChannel(LPC_DMA, DMAREQ_SPI0_RX);
//    while ((Chip_DMA_GetBusyChannels(LPC_DMA) & (1 << DMAREQ_SPI0_RX)) != 0) {}
//    Chip_DMA_AbortChannel(LPC_DMA, DMAREQ_SPI0_RX);
//    Chip_DMA_ClearErrorIntChannel(LPC_DMA, DMAREQ_SPI0_RX);
//    Chip_DMA_EnableChannel(LPC_DMA, DMAREQ_SPI0_RX);
//  }
//  
  /* Clear DMA interrupt for the channel */
  if ( (Chip_DMA_GetIntStatus(LPC_DMA) & DMA_INTSTAT_ACTIVEINT) != 0 ) {
    if ( Chip_DMA_GetActiveIntAChannels(LPC_DMA) & (1 << DMAREQ_SPI0_TX) ) {
      Chip_DMA_ClearActiveIntAChannel(LPC_DMA, DMAREQ_SPI0_TX);
      bDMASPITXDoneFlag = true;
    }
    if ( Chip_DMA_GetActiveIntAChannels(LPC_DMA) & (1 << DMAREQ_SPI0_RX) ) {
      Chip_DMA_ClearActiveIntAChannel(LPC_DMA, DMAREQ_SPI0_RX);
      bDMASPIRXDoneFlag = true;
  //    spi_cs_Set(1);
      mico_rtos_set_semaphore( &spi_transfer_finished_semaphore );
    }
//    if((bDMASPITXDoneFlag == true) && (bDMASPIRXDoneFlag == true)) {
// //     printf("DMA once\r\n");
//      spi_cs_Set(1);
//      mico_rtos_set_semaphore( &spi_transfer_finished_semaphore );
//    }
//    uart0_rx_dma_handler();
//    uart0_tx_dma_handler();
/*;
    if (status & (1UL << DMAREQ_UART0_RX)) {
      Chip_DMA_ClearActiveIntAChannel(LPC_DMA, DMAREQ_UART0_RX);
  //    g_dma_m_rx_done = true;
    }
    if (status & (1UL << DMAREQ_UART0_TX)) {
      Chip_DMA_ClearActiveIntAChannel(LPC_DMA, DMAREQ_UART0_TX);
 //     g_dma_m_tx_done = true;
    }
    if (status & (1UL << DMAREQ_UART1_RX)) {
      Chip_DMA_ClearActiveIntAChannel(LPC_DMA, DMAREQ_UART1_RX);
 //     g_dma_n_rx_done = true;
    }
    if (status & (1UL << DMAREQ_UART2_RX)) {
      Chip_DMA_ClearActiveIntAChannel(LPC_DMA, DMAREQ_UART2_RX);
//      g_dma_n_tx_done = true;
    }
*/
  }
}

uint8_t SPI_DMA_Init(void)
{
  uint8_t ret = false;
  bool    err = false;
  uint32_t i = 0;
  bDMASPITXDoneFlag = bDMASPIRXDoneFlag = false;
/* DMA initialization - enable DMA clocking and reset DMA if needed */
  Chip_DMA_Init(LPC_DMA);
	/* Enable DMA controller and use driver provided DMA table for current descriptors */
  Chip_DMA_Enable(LPC_DMA);
  Chip_DMA_SetSRAMBase(LPC_DMA, DMA_ADDR(Chip_DMA_Table));

#define SPI_FRAME_LENGTH		8       
  Chip_SPI_SetControlInfo(LPC_SPI0, SPI_FRAME_LENGTH, SPI_TXDATCTL_SSELN(SLAVE0));
  Chip_DMA_SetValidChannel(LPC_DMA, DMAREQ_SPI0_TX);
  
  Chip_SPI_SetControlInfo(LPC_SPI0, SPI_FRAME_LENGTH, SPI_TXDATCTL_SSELN(SLAVE0));
  Chip_DMA_SetValidChannel(LPC_DMA, DMAREQ_SPI0_RX);
  LPC_SYSCON->AHBMATPRIO |= 0x03 << 8;
  /* Enable DMA interrupt */
  NVIC_SetPriority(DMA_IRQn, 0);
  NVIC_EnableIRQ(DMA_IRQn);
  return ret;
}

uint8_t SPI_DMA_DeInit(void)
{
  uint8_t ret = false;
  bool    err = false;
  
  bDMASPITXDoneFlag = bDMASPIRXDoneFlag = false;
  
/* Setup channel 0 for the following configuration:
	   - High channel priority
	   - Interrupt A fires on descriptor completion */
  Chip_DMA_DisableChannel(LPC_DMA, DMAREQ_SPI0_TX);
  Chip_DMA_DisableIntChannel(LPC_DMA, DMAREQ_SPI0_TX);
  
/* Setup channel 0 for the following configuration:
	   - High channel priority
	   - Interrupt A fires on descriptor completion */
  Chip_DMA_DisableChannel(LPC_DMA, DMAREQ_SPI0_RX);
  Chip_DMA_DisableIntChannel(LPC_DMA, DMAREQ_SPI0_RX); 
  
/* Enable DMA controller and use driver provided DMA table for current descriptors */
  Chip_DMA_Disable(LPC_DMA);
/* DMA initialization - enable DMA clocking and reset DMA if needed */
  Chip_DMA_DeInit(LPC_DMA);

  /* Enable DMA interrupt */
  NVIC_DisableIRQ(DMA_IRQn);  

  ret = true;
  return ret;
}

uint8_t ADC_Init(void)
{
  return true;
}

uint8_t ADC_DeInit(void)
{
	/* Setup sequencer A for all 12 ADC channels, EOS interrupt */
#if defined(BOARD_NXP_LPCXPRESSO_54000)
  /* All pins to inactive, neither pull-up nor pull-down. */
  Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 29, IOCON_MODE_INACT | IOCON_FUNC0 | IOCON_ANALOG_EN);
  Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 30, IOCON_MODE_INACT | IOCON_FUNC0 | IOCON_ANALOG_EN);
  Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 31, IOCON_MODE_INACT | IOCON_FUNC0 | IOCON_ANALOG_EN);
  Chip_IOCON_PinMuxSet(LPC_IOCON, 1,  0, IOCON_MODE_INACT | IOCON_FUNC0 | IOCON_ANALOG_EN);
  Chip_IOCON_PinMuxSet(LPC_IOCON, 1,  1, IOCON_MODE_INACT | IOCON_FUNC0 | IOCON_ANALOG_EN);
  Chip_IOCON_PinMuxSet(LPC_IOCON, 1,  2, IOCON_MODE_INACT | IOCON_FUNC0 | IOCON_ANALOG_EN);
  Chip_IOCON_PinMuxSet(LPC_IOCON, 1,  3, IOCON_MODE_INACT | IOCON_FUNC0 | IOCON_ANALOG_EN);
  Chip_IOCON_PinMuxSet(LPC_IOCON, 1,  4, IOCON_MODE_INACT | IOCON_FUNC0 | IOCON_ANALOG_EN);
  Chip_IOCON_PinMuxSet(LPC_IOCON, 1,  5, IOCON_MODE_INACT | IOCON_FUNC0 | IOCON_ANALOG_EN);
  Chip_IOCON_PinMuxSet(LPC_IOCON, 1,  6, IOCON_MODE_INACT | IOCON_FUNC0 | IOCON_ANALOG_EN);
  Chip_IOCON_PinMuxSet(LPC_IOCON, 1,  7, IOCON_MODE_INACT | IOCON_FUNC0 | IOCON_ANALOG_EN);
  Chip_IOCON_PinMuxSet(LPC_IOCON, 1,  8, IOCON_MODE_INACT | IOCON_FUNC0 | IOCON_ANALOG_EN);

  Chip_ADC_DisableInt(LPC_ADC, (ADC_INTEN_SEQA_ENABLE | ADC_INTEN_OVRRUN_ENABLE));
  Chip_ADC_DisableSequencer(LPC_ADC, ADC_SEQA_IDX);
/* Enable ADC NVIC interrupt */
  NVIC_DisableIRQ(ADC_SEQA_IRQn);
  NVIC_DisableIRQ(ADC_SEQB_IRQn);
/* Clear all pending interrupts */
  Chip_ADC_ClearFlags(LPC_ADC, Chip_ADC_GetFlags(LPC_ADC));
  
  Chip_ADC_DeInit(LPC_ADC);
#else
#warning "No ADC setup for this example"
#endif
  return true;
}

void QCA_SYS_DELAY(uint32_t time)
{
  vTaskDelay(time);
}

#define GPIO_GPOPWD_PORT	1  // 0
#define GPIO_GPOPWD_PIN		17 // 29

#define GPIO_GPOFET_PORT	0
#define GPIO_GPOFET_PIN		25

#define GPIO_SSPSEL_PORT	0
#define GPIO_SSPSEL_PIN		14

#define GPIO_SSPCLK_PORT	0   // 1
#define GPIO_SSPCLK_PIN		11  // 3

#define GPIO_SSPMISO_PORT       0   // 1
#define GPIO_SSPMISO_PIN	13  // 4

#define GPIO_SSPMOSI_PORT	0
#define GPIO_SSPMOSI_PIN	12

#define GPIO_SPIINT_PORT	0
#define GPIO_SPIINT_PIN		30
#define GPIO_SPIINT_INDEX	PININTSELECT0	/* PININT index used for GPIO mapping */

#define GPIO_STATUS_PORT        1
#define GPIO_STATUS_PIN         9

#define GPIO_BOOT_PORT          1
#define GPIO_BOOT_PIN           10

#define GPIO_STANBY_PORT        1
#define GPIO_STANBY_PIN         11

#define GPIO_EASYLINK_PORT      0
#define GPIO_EASYLINK_PIN       31
#define GPIO_EASYLINK_INDEX	PININTSELECT1	/* PININT index used for GPIO mapping */

//#define A_VOID void 
//extern A_VOID Custom_HW_InterruptHandler(A_VOID *pointer); // Magicoe
void PIN_INT0_IRQHandler(void)
{
  Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH(GPIO_SPIINT_INDEX)); 
  spi_irq_handler();
}
static uint32_t _default_start_time = 0;
static mico_timer_t _button_EL_timer;
#define RestoreDefault_TimeOut          3000  /**< Restore default and start easylink after 
                                                   press down EasyLink button for 3 seconds. */
void PIN_INT1_IRQHandler(void)
{
 int interval = -1;
  Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH(GPIO_EASYLINK_INDEX)); 
  printf("Easylink\r\n");
 
  
  if ( Chip_GPIO_ReadPortBit(LPC_GPIO, GPIO_EASYLINK_PORT, GPIO_EASYLINK_PIN) == 0 ) {
    _default_start_time = mico_get_time()+1;
    mico_start_timer(&_button_EL_timer);
  } else {
    interval = mico_get_time() + 1 - _default_start_time;
    if ( (_default_start_time != 0) && interval > 50 && interval < RestoreDefault_TimeOut){
      /* EasyLink button clicked once */
      PlatformEasyLinkButtonClickedCallback();
    }
    mico_stop_timer(&_button_EL_timer);
    _default_start_time = 0;
  }
}


uint8_t STATUS_GPIO_Init(void)
{
  Chip_GPIO_SetPinState(LPC_GPIO, GPIO_STATUS_PORT, GPIO_STATUS_PIN, 0);
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_STATUS_PORT, GPIO_STATUS_PIN, (IOCON_FUNC0 | IOCON_GPIO_MODE | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));
  Chip_GPIO_SetPinDIRInput(LPC_GPIO, GPIO_STATUS_PORT, GPIO_STATUS_PIN);
  return true;
}

uint8_t BOOT_GPIO_Init(void)
{
  Chip_GPIO_SetPinState(LPC_GPIO, GPIO_STANBY_PORT, GPIO_BOOT_PIN, 0);
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_STANBY_PORT, GPIO_BOOT_PIN, (IOCON_FUNC0 | IOCON_GPIO_MODE | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));
  Chip_GPIO_SetPinDIRInput(LPC_GPIO, GPIO_STANBY_PORT, GPIO_BOOT_PIN);
  return true;
}

uint8_t STANDBY_GPIO_Init(void)
{
  Chip_GPIO_SetPinState(LPC_GPIO, GPIO_BOOT_PORT, GPIO_STANBY_PIN, 0);
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_BOOT_PORT, GPIO_STANBY_PIN, (IOCON_FUNC0 | IOCON_GPIO_MODE | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));
  Chip_GPIO_SetPinDIRInput(LPC_GPIO, GPIO_BOOT_PORT, GPIO_STANBY_PIN);
  return true;
}

uint8_t EASYLINK_GPIO_Init(void)
{
  Chip_GPIO_SetPinState(LPC_GPIO, GPIO_EASYLINK_PORT, GPIO_EASYLINK_PIN, 0);
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_EASYLINK_PORT, GPIO_EASYLINK_PIN, (IOCON_FUNC0 | IOCON_GPIO_MODE | IOCON_DIGITAL_EN));
  Chip_GPIO_SetPinDIRInput(LPC_GPIO, GPIO_EASYLINK_PORT, GPIO_EASYLINK_PIN);
//  
//  Chip_PININT_Init(LPC_PININT);
//  /* Configure pin interrupt selection for the GPIO pin in Input Mux Block */
//  Chip_INMUX_PININT_Config(LPC_INMUX, GPIO_EASYLINK_INDEX, GPIO_EASYLINK_PORT, GPIO_EASYLINK_PIN);
//
//  /* Configure channel interrupt as edge sensitive and falling edge interrupt */
//  Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH(GPIO_EASYLINK_INDEX));
//  Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH(GPIO_EASYLINK_INDEX));
//  Chip_PININT_EnableIntLow(LPC_PININT, PININTCH(GPIO_EASYLINK_INDEX));
//  
////  NVIC_SetPriority(PININT0_IRQn, 1);
//  /* Enable interrupt in the NVIC */
//  NVIC_ClearPendingIRQ(PININT1_IRQn);
//  NVIC_EnableIRQ(PININT1_IRQn);	

return true;
}

uint8_t RESET_GPIO_Init(void)
{
  Chip_GPIO_SetPinState(LPC_GPIO, GPIO_GPOPWD_PORT, GPIO_GPOPWD_PIN, 0);
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_GPOPWD_PORT, GPIO_GPOPWD_PIN, (IOCON_FUNC0 | IOCON_GPIO_MODE | IOCON_DIGITAL_EN));
  Chip_GPIO_SetPinDIROutput(LPC_GPIO, GPIO_GPOPWD_PORT, GPIO_GPOPWD_PIN);
  return true;
}

uint8_t RESET_GPIO_DeInit(void)
{
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_GPOPWD_PORT, GPIO_GPOPWD_PIN, (IOCON_FUNC0 | IOCON_GPIO_MODE | IOCON_DIGITAL_EN));
  Chip_GPIO_SetPinDIRInput(LPC_GPIO, GPIO_GPOPWD_PORT, GPIO_GPOPWD_PIN);
  return true;
}

void RESET_GPIO_Set(uint8_t status)
{
  if(status == 0) Chip_GPIO_SetPinState(LPC_GPIO, GPIO_GPOPWD_PORT, GPIO_GPOPWD_PIN, 0);
  else            Chip_GPIO_SetPinState(LPC_GPIO, GPIO_GPOPWD_PORT, GPIO_GPOPWD_PIN, 1);
}

uint8_t QCAFET_GPIO_Init(void)
{
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_GPOFET_PORT, GPIO_GPOFET_PIN, (IOCON_FUNC0 | IOCON_GPIO_MODE | IOCON_DIGITAL_EN));
  Chip_GPIO_SetPinDIROutput(LPC_GPIO, GPIO_GPOFET_PORT, GPIO_GPOFET_PIN);
  Chip_GPIO_SetPinState(LPC_GPIO, GPIO_GPOFET_PORT, GPIO_GPOFET_PIN, 0);
  return true;
}

void QCAFET_GPIO_Set(uint8_t status)
{
  if(status == 0) Chip_GPIO_SetPinState(LPC_GPIO, GPIO_GPOFET_PORT, GPIO_GPOFET_PIN, 0);
  else            Chip_GPIO_SetPinState(LPC_GPIO, GPIO_GPOFET_PORT, GPIO_GPOFET_PIN, 1);
}

void QCALED_GPIO_Set(uint8_t status)
{
  if(status == 0) Chip_GPIO_SetPinState(LPC_GPIO, 1, 5, 0);
  else            Chip_GPIO_SetPinState(LPC_GPIO, 1, 5, 1);
}

void spi_cs_init(void)
{
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_SSPSEL_PORT, GPIO_SSPSEL_PIN, IOCON_MODE_REPEATER);
  Chip_GPIO_SetPinDIROutput(LPC_GPIO, GPIO_SSPSEL_PORT, GPIO_SSPSEL_PIN);
  Chip_GPIO_SetPinState(LPC_GPIO, GPIO_SSPSEL_PORT, GPIO_SSPSEL_PIN, 1);
}
void spi_cs_Set(uint8_t status)
{
  if(status == 0) Chip_GPIO_SetPinState(LPC_GPIO, GPIO_SSPSEL_PORT, GPIO_SSPSEL_PIN, 0);
  else            Chip_GPIO_SetPinState(LPC_GPIO, GPIO_SSPSEL_PORT, GPIO_SSPSEL_PIN, 1);
}

uint8_t SPI_GPIO_Init(void)
{
#if (defined(BOARD_NXP_LPCXPRESSO_54000))
  spi_cs_init();
//  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_SSPSEL_PORT,  GPIO_SSPSEL_PIN,  (IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* SSEL */
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_SSPCLK_PORT,  GPIO_SSPCLK_PIN,  (IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN)); //(IOCON_FUNC5 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* SCK */
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_SSPMISO_PORT, GPIO_SSPMISO_PIN, (IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN)); // (IOCON_FUNC5 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* MISO */
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_SSPMOSI_PORT, GPIO_SSPMOSI_PIN, (IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* MOSI */
#else
	/* Configure your own SPI pin muxing here if needed */
#warning "No SPI pin muxing defined"
#endif
	
  return true;
}

uint8_t SPI_GPIO_DeInit(void)
{
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_SSPSEL_PORT,  GPIO_SSPSEL_PIN,  (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* SSEL */
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_SSPCLK_PORT,  GPIO_SSPCLK_PIN,  (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* SCK */
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_SSPMISO_PORT, GPIO_SSPMISO_PIN, (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* MISO */
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_SSPMOSI_PORT, GPIO_SSPMOSI_PIN, (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* MOSI */	
  return true;
}

static volatile bool mEnd, sEnd;

/* SPI master select assertion callback function */
static void SPIMasterAssert(SPIM_XFER_T *pMasterXfer)
{
	/* Indicates tha master just asserted the slave select
	   signal */
}

/* SPI master send data callback function */
static void SPIMasterSendData(SPIM_XFER_T *pMasterXfer)
{
	/* This callback is called when the master needs more data to
	   send. The pMasterXfer->pTXData buffer pointer and transfer size
	   (in items) in pMasterXfer->txCount should be updated. */

	/* If this function sets the pMasterXfer->terminate flag to true,
	   this function won't be called again and the transfer will
	   terminate when the current transmit buffer is complete. */

	/* This example sets up the entire transfer structure without
	   using this callback. */
}

/* SPI master receive data callback function */
static void SPIMasterRecvData(SPIM_XFER_T *pMasterXfer)
{
	/* This callback is called when the master needs another receive
	   buffer. The pMasterXfer->pRXData buffer pointer and transfer size
	   (in items) in pMasterXfer->rxCount shoudl be updated. */

	/* This example sets up the entire transfer structure without
	   using this callback. */
}

/* SPI master select de-assertion callback function */
static void SPIMasterDeAssert(SPIM_XFER_T *pMasterXfer)
{
  /* Indicates tha master just deasserted the slave select
     signal */
}

/* SPI master transfer done callback */
static void SPIMasterDone(SPIM_XFER_T *pMasterXfer)
{
  /* Indicates tha transfer is complete */
  mEnd = true;
}

/* SPI master driver callbacks */
static const SPIM_CALLBACKS_T spiMasterCallbacks = {
  &SPIMasterAssert,
  &SPIMasterSendData,
  &SPIMasterRecvData,
  &SPIMasterDeAssert,
  &SPIMasterDone
};


SPIM_XFER_T spiMasterXfer;

void SPI0_IRQHandler(void)
{
  uint32_t ints = Chip_SPI_GetPendingInts(LPC_SPI0);

  /* Handle SPI slave interrupts only */
  if ((ints &  (SPI_INTENSET_RXDYEN | SPI_INTENSET_RXOVEN |
                SPI_INTENSET_TXUREN | SPI_INTENSET_SSAEN | SPI_INTENSET_SSDEN)) != 0) {
  /* SPI slave handler */
    Chip_SPIM_XferHandler(LPC_SPI0, &spiMasterXfer);
  }
}

extern void SPI_IRQ_ENABLE(void);
extern void SPI_IRQ_DISABLE(void);
extern uint8_t SPI_Int_Init(void);
uint8_t SPI_Init(void)
{
  SPI_CFGSETUP_T spiSetup;
  SPIM_DELAY_CONFIG_T masterDelay;

  SPI_GPIO_Init();
  Chip_SPI_Init(LPC_SPI0);
  
  spiSetup.master = 1;
  spiSetup.lsbFirst = 0;
  spiSetup.mode = SPI_CLOCK_MODE3;
  Chip_SPI_ConfigureSPI(LPC_SPI0, &spiSetup);
  /* Setup master controller SSEL0 for active low select */
  Chip_SPI_SetCSPolLow(LPC_SPI0, 0);
  /* Setup master clock rate, slave clock doesn't need to be setup */
  Chip_SPIM_SetClockRate(LPC_SPI0, 20000000);

  /* Setup master delay (all chip selects) */
  masterDelay.PreDelay			= 0x01;
  masterDelay.PostDelay			= 0x01;
  masterDelay.FrameDelay		= 0x01;
  masterDelay.TransferDelay             = 0x01;
  Chip_SPIM_DelayConfig(LPC_SPI0, &masterDelay);

  /* Setup master transfer callbacks in the transfer descriptor */
  spiMasterXfer.pCB = &spiMasterCallbacks;
  Chip_SPI_SetXferSize(LPC_SPI0, 8);

	/* Setup master trasnfer options - 16 data bits per transfer, EOT, EOF */
  spiMasterXfer.options =
		SPI_TXCTL_FLEN(8) |		/* This must be enabled as a minimum, use 16 data bits */
		// SPI_TXCTL_EOT |			/* Enable this to assert and deassert SSEL for each individual byte/word, current slave functions for this example do not support this */
		// SPI_TXCTL_EOF |			/* Insert a delay between bytes/words as defined by frame delay time */
		// SPI_TXCTL_RXIGNORE |		/* Enable this to ignore incoming data, or set spiMasterXfer.pRXData16 to NULL to ignore RX data  */
		0;

  /* Transfer will terminate after current buffer is sent. If terminate is not set, the buffers
    must be setup by the callbacks		*/
  spiMasterXfer.terminate = true;

  /* Use SPI select 0 */
  spiMasterXfer.sselNum = 0;
  
#ifndef ATH_SPI_DMA
  Chip_SPI_Enable(LPC_SPI0);
  /* For the SPI controller configured in master mode, enable SPI master interrupts
     for interrupt service. Do not enable SPI_INTENSET_TXDYEN. */
  Chip_SPI_EnableInts(LPC_SPI0, (SPI_INTENSET_RXDYEN |
                      SPI_INTENSET_RXOVEN | SPI_INTENSET_TXUREN | SPI_INTENSET_SSAEN |
                      SPI_INTENSET_SSDEN));	
  Chip_SPI_FlushFifos(LPC_SPI0);	
  NVIC_EnableIRQ(SPI0_IRQn);
#else
//  Chip_SPI_Int_Cmd(LPC_SPI0, SPI_INTENSET_TXDYEN | SPI_INTENSET_RXDYEN | SPI_INTENSET_RXOVEN | SPI_INTENSET_TXUREN, ENABLE);
  mico_rtos_init_semaphore(&spi_transfer_finished_semaphore, 1);
  Chip_SPI_Enable(LPC_SPI0);
  SPI_DMA_Init();
#ifdef SEMAPHORE 
  mico_rtos_get_semaphore( &spi_transfer_finished_semaphore, SEMDELAY );
#endif
#endif
  SPI_Int_Init();
//  while((bDMASPITXDoneFlag!=true)&&(bDMASPIRXDoneFlag!= true));
//  bDMASPITXDoneFlag = bDMASPIRXDoneFlag = false;
  return true;
}

uint8_t SPI_DeInit(void)
{
#ifndef ATH_SPI_DMA
  SPI_IRQ_DISABLE();
  NVIC_ClearPendingIRQ(SPI0_IRQn);
  NVIC_DisableIRQ(SPI0_IRQn);
#else
  SPI_IRQ_DISABLE();
  
  SPI_DMA_DeInit();
  Chip_SPI_Disable(LPC_SPI0);
  SPI_GPIO_DeInit();
  Chip_SPI_DeInit(LPC_SPI0);
#endif
  return true;
}


uint8_t SPI_Int_Init(void)
{
  Chip_PININT_Init(LPC_PININT);
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_SPIINT_PORT, GPIO_SPIINT_PIN, (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN  | IOCON_GPIO_MODE));
  Chip_GPIO_SetPinDIRInput(LPC_GPIO, GPIO_SPIINT_PORT, GPIO_SPIINT_PIN);

  /* Configure pin interrupt selection for the GPIO pin in Input Mux Block */
  Chip_INMUX_PININT_Config(LPC_INMUX, GPIO_SPIINT_INDEX, GPIO_SPIINT_PORT, GPIO_SPIINT_PIN);

  /* Configure channel interrupt as edge sensitive and falling edge interrupt */
  Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH(GPIO_SPIINT_INDEX));
  Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH(GPIO_SPIINT_INDEX));
//  Chip_PININT_EnableIntLow(LPC_PININT, PININTCH(GPIO_SPIINT_INDEX));
  Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH(GPIO_SPIINT_INDEX));
  
//  NVIC_SetPriority(PININT0_IRQn, 1);
  /* Enable interrupt in the NVIC */
  NVIC_ClearPendingIRQ(PININT0_IRQn);
  NVIC_EnableIRQ(PININT0_IRQn);	
  
  return true;
}

uint8_t SPI_Int_DeInit(void)
{
  NVIC_DisableIRQ(PININT0_IRQn);	
  NVIC_ClearPendingIRQ(PININT0_IRQn);
  Chip_SPI_DeInit(LPC_SPI0);
  return true;
}

void SPI_Endian_Set(uint32_t ENDIAN)
{

}

void SPI_DATA_START(void)
{
  /* Setup master trasnfer options - 16 data bits per transfer, EOT, EOF */
  spiMasterXfer.options =
          SPI_TXCTL_FLEN(8) |		/* This must be enabled as a minimum, use 16 data bits */
          // SPI_TXCTL_EOT |			/* Enable this to assert and deassert SSEL for each individual byte/word, current slave functions for this example do not support this */
          // SPI_TXCTL_EOF |			/* Insert a delay between bytes/words as defined by frame delay time */
          // SPI_TXCTL_RXIGNORE |		/* Enable this to ignore incoming data, or set spiMasterXfer.pRXData16 to NULL to ignore RX data  */
          0;

  /* Transfer will terminate after current buffer is sent. If terminate is not set, the buffers
    must be setup by the callbacks		*/
  spiMasterXfer.terminate = true;

  /* Use SPI select 0 */
  spiMasterXfer.sselNum = 0;
  mEnd = false;
}

void SPI_DATA_STOP(void)
{
#ifndef ATH_SPI_DMA
  /* Sleep until transfers are complete */
  while (mEnd == false) {
    //__WFI();
    asm("wfi");
  }
#else
//  while ((bDMASPITXDoneFlag == false)&&(bDMASPIRXDoneFlag == false)) {
//    __WFI();
//  }
/*Force an end to the current transfer */
  Chip_SPI_ClearStatus(LPC_SPI0, SPI_STAT_EOT);
#endif
}
extern volatile uint32_t timer0_cnt;
uint32_t SPI_DATA_READ(uint8_t * pBuffer, uint32_t length)
{
  uint32_t transfer_size;
  OSStatus result;

//  printf("SPI_DATA_READ 0x%x\r\n", length);
  spi_cs_Set(0);
#ifndef ATH_SPI_DMA
  spiMasterXfer.pTXData8 = NULL;
  spiMasterXfer.txCount = length;/* Count is in transfer size */	
  spiMasterXfer.pRXData8 = pBuffer;
  spiMasterXfer.rxCount = length;/* Count is in transfer size */	
	
  Chip_SPIM_Xfer(LPC_SPI0, &spiMasterXfer);	
	
  while (spiMasterXfer.dataRXferred != length) {
    //__WFI();
    asm("wfi");
  }
  
  if(length == 1) {
    if(spiMasterXfer.dataRXferred == length) {
      mEnd = true;
    }
  }
  
  transfer_size = spiMasterXfer.dataRXferred;
#else
  bDMASPITXDoneFlag = false;
  bDMASPIRXDoneFlag = false;
  memset(pBuffer, 0xFF, length);
	Chip_SPI_ClearStatus(LPC_SPI0, SPI_STAT_RXOV | SPI_STAT_TXUR | SPI_STAT_SSA | SPI_STAT_SSD);

  timer0_cnt=0;
  LPC_CTIMER0->TC = 0;
   if(length<=1000) {
    Chip_SPI_ClearStatus(LPC_SPI0, SPI_STAT_RXOV | SPI_STAT_TXUR | SPI_STAT_SSA | SPI_STAT_SSD);

    Chip_DMA_InitChannel( DMAREQ_SPI0_RX, DMA_ADDR(&LPC_SPI0->RXDAT), DMA_XFERCFG_SRCINC_0, 
                          DMA_ADDR(&pBuffer[0]), DMA_XFERCFG_DSTINC_1, WIDTH_8_BITS,
                          length, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(1)));
    Chip_DMA_StartTransfer(DMAREQ_SPI0_RX, DMA_XFERCFG_SRCINC_0, DMA_XFERCFG_DSTINC_1, DMA_XFERCFG_WIDTH_8, length);

    Chip_DMA_InitChannel( DMAREQ_SPI0_TX, DMA_ADDR(&pBuffer[0]), DMA_XFERCFG_SRCINC_1,
                          DMA_ADDR(&LPC_SPI0->TXDAT), DMA_XFERCFG_DSTINC_0, WIDTH_8_BITS,
                          length, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(2)));
    Chip_DMA_StartTransfer(DMAREQ_SPI0_TX, DMA_XFERCFG_SRCINC_1, DMA_XFERCFG_DSTINC_0, DMA_XFERCFG_WIDTH_8, length);
  }
  else {
    bDMASPITXDoneFlag = false;
    bDMASPIRXDoneFlag = false;
    Chip_SPI_ClearStatus(LPC_SPI0, SPI_STAT_RXOV | SPI_STAT_TXUR | SPI_STAT_SSA | SPI_STAT_SSD);

    Chip_DMA_InitChannel( DMAREQ_SPI0_RX, DMA_ADDR(&LPC_SPI0->RXDAT), DMA_XFERCFG_SRCINC_0, 
                          DMA_ADDR(&pBuffer[0]), DMA_XFERCFG_DSTINC_1, WIDTH_8_BITS,
                          1000, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(1)));
    Chip_DMA_StartTransfer(DMAREQ_SPI0_RX, DMA_XFERCFG_SRCINC_0, DMA_XFERCFG_DSTINC_1, DMA_XFERCFG_WIDTH_8, 1000);

    Chip_DMA_InitChannel( DMAREQ_SPI0_TX, DMA_ADDR(&pBuffer[0]), DMA_XFERCFG_SRCINC_1,
                          DMA_ADDR(&LPC_SPI0->TXDAT), DMA_XFERCFG_DSTINC_0, WIDTH_8_BITS,
                          1000, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(2)));
    Chip_DMA_StartTransfer(DMAREQ_SPI0_TX, DMA_XFERCFG_SRCINC_1, DMA_XFERCFG_DSTINC_0, DMA_XFERCFG_WIDTH_8, 1000);
#ifdef SEMAPHORE
  
  result = mico_rtos_get_semaphore( &spi_transfer_finished_semaphore, SEMDELAY );
  
#else
  while(bDMASPITXDoneFlag == false) {
    //__WFI();
    asm("wfi");
  } 
  while(bDMASPIRXDoneFlag == false) {
    //__WFI();s
    asm("wfi");
  }
#endif
    bDMASPITXDoneFlag = false;
    bDMASPIRXDoneFlag = false;
    Chip_SPI_ClearStatus(LPC_SPI0, SPI_STAT_RXOV | SPI_STAT_TXUR | SPI_STAT_SSA | SPI_STAT_SSD);

    Chip_DMA_InitChannel( DMAREQ_SPI0_RX, DMA_ADDR(&LPC_SPI0->RXDAT), DMA_XFERCFG_SRCINC_0, 
                          DMA_ADDR(&pBuffer[1000]), DMA_XFERCFG_DSTINC_1, WIDTH_8_BITS,
                          length - 1000, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(1)));
    Chip_DMA_StartTransfer(DMAREQ_SPI0_RX, DMA_XFERCFG_SRCINC_0, DMA_XFERCFG_DSTINC_1, DMA_XFERCFG_WIDTH_8, length - 1000);

    Chip_DMA_InitChannel( DMAREQ_SPI0_TX, DMA_ADDR(&pBuffer[1000]), DMA_XFERCFG_SRCINC_1,
                          DMA_ADDR(&LPC_SPI0->TXDAT), DMA_XFERCFG_DSTINC_0, WIDTH_8_BITS,
                          length - 1000, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(2)));
    Chip_DMA_StartTransfer(DMAREQ_SPI0_TX, DMA_XFERCFG_SRCINC_1, DMA_XFERCFG_DSTINC_0, DMA_XFERCFG_WIDTH_8, length - 1000);    
  }  
  
#ifdef SEMAPHORE
  result = mico_rtos_get_semaphore( &spi_transfer_finished_semaphore, SEMDELAY );
#else 
  while (bDMASPITXDoneFlag == false) { // &&(bDMASPIRXDoneFlag == false)) {
   //__WFI();
    asm("wfi");
  }
  while (bDMASPIRXDoneFlag == false) { // &&(bDMASPIRXDoneFlag == false)) {
    //__WFI();
    asm("wfi");
  }  
#endif 
  
#ifdef SPIDEBUG
  if(length >= 1000) //1023)
  printf("timer0_cnt = %d, size %d\r\n", LPC_CTIMER0->TC, length);
#endif
//  Chip_SPI_ClearStatus(LPC_SPI0, SPI_STAT_EOT);
  transfer_size = length;
#endif
  spi_cs_Set(1);
  return transfer_size;
}

uint32_t SPI_DATA_WRITE(uint8_t * pBuffer, uint32_t length)
{
  uint32_t transfer_size;
  OSStatus result;
//  printf("SPI_DATA_WRITE 0x%x\r\n", length);
  spi_cs_Set(0);
#ifndef ATH_SPI_DMA
  spiMasterXfer.pTXData8 = pBuffer;
  spiMasterXfer.txCount  = length;/* Count is in transfer size */	
/* Start master transfer */
  Chip_SPIM_Xfer(LPC_SPI0, &spiMasterXfer);	
//	for(i=0; i<100; i++);
  while (spiMasterXfer.dataTXferred < length) {
    //__WFI();
    asm("wfi");
  }
  
  if(length == 1) {
    if(spiMasterXfer.dataTXferred == length) {
      mEnd = true;
    }
  }
  
  transfer_size = spiMasterXfer.dataTXferred;
#else
  bDMASPITXDoneFlag = false;
  bDMASPIRXDoneFlag = false;
  timer0_cnt = 0;
  LPC_CTIMER0->TC = 0;
  if(length<=1000) {
    Chip_SPI_ClearStatus(LPC_SPI0, SPI_STAT_RXOV | SPI_STAT_TXUR | SPI_STAT_SSA | SPI_STAT_SSD);

  Chip_DMA_InitChannel( DMAREQ_SPI0_RX, DMA_ADDR(&LPC_SPI0->RXDAT), DMA_XFERCFG_SRCINC_0, 
                        DMA_ADDR(&pBuffer[0]), DMA_XFERCFG_DSTINC_1, WIDTH_8_BITS,
                        length, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(1)));
  Chip_DMA_StartTransfer(DMAREQ_SPI0_RX, DMA_XFERCFG_SRCINC_0, DMA_XFERCFG_DSTINC_1, DMA_XFERCFG_WIDTH_8, length);

    Chip_DMA_InitChannel( DMAREQ_SPI0_TX, DMA_ADDR(&pBuffer[0]), DMA_XFERCFG_SRCINC_1,
                          DMA_ADDR(&LPC_SPI0->TXDAT), DMA_XFERCFG_DSTINC_0, WIDTH_8_BITS,
                          length, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(2)));
    Chip_DMA_StartTransfer(DMAREQ_SPI0_TX, DMA_XFERCFG_SRCINC_1, DMA_XFERCFG_DSTINC_0, DMA_XFERCFG_WIDTH_8, length);
  }
  else {
    bDMASPITXDoneFlag = false;
    bDMASPIRXDoneFlag = false;
    Chip_SPI_ClearStatus(LPC_SPI0, SPI_STAT_RXOV | SPI_STAT_TXUR | SPI_STAT_SSA | SPI_STAT_SSD);

    Chip_DMA_InitChannel( DMAREQ_SPI0_RX, DMA_ADDR(&LPC_SPI0->RXDAT), DMA_XFERCFG_SRCINC_0, 
                          DMA_ADDR(&pBuffer[0]), DMA_XFERCFG_DSTINC_1, WIDTH_8_BITS,
                          1000, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(1)));
    Chip_DMA_StartTransfer(DMAREQ_SPI0_RX, DMA_XFERCFG_SRCINC_0, DMA_XFERCFG_DSTINC_1, DMA_XFERCFG_WIDTH_8, 1000);

    Chip_DMA_InitChannel( DMAREQ_SPI0_TX, DMA_ADDR(&pBuffer[0]), DMA_XFERCFG_SRCINC_1,
                          DMA_ADDR(&LPC_SPI0->TXDAT), DMA_XFERCFG_DSTINC_0, WIDTH_8_BITS,
                          1000, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(2)));
    Chip_DMA_StartTransfer(DMAREQ_SPI0_TX, DMA_XFERCFG_SRCINC_1, DMA_XFERCFG_DSTINC_0, DMA_XFERCFG_WIDTH_8, 1000);
#ifdef SEMAPHORE
  
  result = mico_rtos_get_semaphore( &spi_transfer_finished_semaphore, SEMDELAY );
  
#else
  while(bDMASPITXDoneFlag == false) {
    //__WFI();
    asm("wfi");
  } 
  while(bDMASPIRXDoneFlag == false) {
    //__WFI();
    asm("wfi");
  }
#endif
    bDMASPITXDoneFlag = false;
    bDMASPIRXDoneFlag = false;
    Chip_SPI_ClearStatus(LPC_SPI0, SPI_STAT_RXOV | SPI_STAT_TXUR | SPI_STAT_SSA | SPI_STAT_SSD);

    Chip_DMA_InitChannel( DMAREQ_SPI0_RX, DMA_ADDR(&LPC_SPI0->RXDAT), DMA_XFERCFG_SRCINC_0, 
                          DMA_ADDR(&pBuffer[1000]), DMA_XFERCFG_DSTINC_1, WIDTH_8_BITS,
                          length-1000, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(1)));
    Chip_DMA_StartTransfer(DMAREQ_SPI0_RX, DMA_XFERCFG_SRCINC_0, DMA_XFERCFG_DSTINC_1, DMA_XFERCFG_WIDTH_8, length - 1000);

    Chip_DMA_InitChannel( DMAREQ_SPI0_TX, DMA_ADDR(&pBuffer[1000]), DMA_XFERCFG_SRCINC_1,
                          DMA_ADDR(&LPC_SPI0->TXDAT), DMA_XFERCFG_DSTINC_0, WIDTH_8_BITS,
                          length-1000, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(2)));
    Chip_DMA_StartTransfer(DMAREQ_SPI0_TX, DMA_XFERCFG_SRCINC_1, DMA_XFERCFG_DSTINC_0, DMA_XFERCFG_WIDTH_8, length - 1000);    
  }  
#ifdef SEMAPHORE
  
  result = mico_rtos_get_semaphore( &spi_transfer_finished_semaphore, SEMDELAY );
  
#else
  while(bDMASPITXDoneFlag == false) {
    //__WFI();
    asm("wfi");
  } 
  while(bDMASPIRXDoneFlag == false) {
    //__WFI();s
    asm("wfi");
  }
#endif
  
#ifdef SPIDEBUG
  if(length >= 1000) // 1023)
  printf("timer0_cnt = %d, size %d\r\n", LPC_CTIMER0->TC, length);
#endif
//  Chip_SPI_ClearStatus(LPC_SPI0, SPI_STAT_EOT);
  transfer_size = length;
#endif
  spi_cs_Set(1);
  return transfer_size;
}

uint32_t SPI_DATA_WRITEREAD(uint8_t * pBufferW, uint8_t * pBufferR, uint32_t lengthW, uint32_t lengthR)
{
  uint32_t transfer_size;
  OSStatus result;
 // printf("SPI_DATA_WRITEREAD 0x%x\r\n", lengthW);
  spi_cs_Set(0);
  
#ifndef ATH_SPI_DMA
  spiMasterXfer.pTXData8 = pBufferW;
  spiMasterXfer.txCount  = lengthW;/* Count is in transfer size */	
  spiMasterXfer.pRXData8 = pBufferR;
  spiMasterXfer.rxCount  = lengthR;/* Count is in transfer size */		

/* Start master transfer */
  Chip_SPIM_Xfer(LPC_SPI0, &spiMasterXfer);
//	for(i=0; i<100; i++);
  while (spiMasterXfer.dataTXferred < lengthW) {
    //__WFI();
    asm("wfi");
  }
  transfer_size = spiMasterXfer.dataTXferred;
#else
  uint32_t i;
  
  bDMASPITXDoneFlag = false;
  bDMASPIRXDoneFlag = false;
  timer0_cnt = 0;
  LPC_CTIMER0->TC = 0;
   if(lengthR<=1000) {
    Chip_SPI_ClearStatus(LPC_SPI0, SPI_STAT_RXOV | SPI_STAT_TXUR | SPI_STAT_SSA | SPI_STAT_SSD);

  Chip_DMA_InitChannel( DMAREQ_SPI0_RX, DMA_ADDR(&LPC_SPI0->RXDAT), DMA_XFERCFG_SRCINC_0, 
                        DMA_ADDR(&pBufferR[0]), DMA_XFERCFG_DSTINC_1, WIDTH_8_BITS,
                        lengthR, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(1)));
  Chip_DMA_StartTransfer(DMAREQ_SPI0_RX, DMA_XFERCFG_SRCINC_0, DMA_XFERCFG_DSTINC_1, DMA_XFERCFG_WIDTH_8, lengthR);

    Chip_DMA_InitChannel( DMAREQ_SPI0_TX, DMA_ADDR(&pBufferW[0]), DMA_XFERCFG_SRCINC_1,
                          DMA_ADDR(&LPC_SPI0->TXDAT), DMA_XFERCFG_DSTINC_0, WIDTH_8_BITS,
                          lengthW, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(2)));
    Chip_DMA_StartTransfer(DMAREQ_SPI0_TX, DMA_XFERCFG_SRCINC_1, DMA_XFERCFG_DSTINC_0, DMA_XFERCFG_WIDTH_8, lengthW);
  }
  else {
    bDMASPITXDoneFlag = false;
    bDMASPIRXDoneFlag = false;
    Chip_SPI_ClearStatus(LPC_SPI0, SPI_STAT_RXOV | SPI_STAT_TXUR | SPI_STAT_SSA | SPI_STAT_SSD);

    Chip_DMA_InitChannel( DMAREQ_SPI0_RX, DMA_ADDR(&LPC_SPI0->RXDAT), DMA_XFERCFG_SRCINC_0, 
                          DMA_ADDR(&pBufferR[0]), DMA_XFERCFG_DSTINC_1, WIDTH_8_BITS,
                          1000, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(1)));
    Chip_DMA_StartTransfer(DMAREQ_SPI0_RX, DMA_XFERCFG_SRCINC_0, DMA_XFERCFG_DSTINC_1, DMA_XFERCFG_WIDTH_8, 1000);

    Chip_DMA_InitChannel( DMAREQ_SPI0_TX, DMA_ADDR(&pBufferW[0]), DMA_XFERCFG_SRCINC_1,
                          DMA_ADDR(&LPC_SPI0->TXDAT), DMA_XFERCFG_DSTINC_0, WIDTH_8_BITS,
                          1000, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(2)));
    Chip_DMA_StartTransfer(DMAREQ_SPI0_TX, DMA_XFERCFG_SRCINC_1, DMA_XFERCFG_DSTINC_0, DMA_XFERCFG_WIDTH_8, 1000);
#ifdef SEMAPHORE
  
  result = mico_rtos_get_semaphore( &spi_transfer_finished_semaphore, SEMDELAY );
  
#else
  while(bDMASPITXDoneFlag == false) {
    //__WFI();
    asm("wfi");
  } 
  while(bDMASPIRXDoneFlag == false) {
    //__WFI();s
    asm("wfi");
  }
#endif
  
    bDMASPITXDoneFlag = false;
    bDMASPIRXDoneFlag = false;
//    Chip_SPI_ClearStatus(LPC_SPI0, SPI_STAT_RXOV | SPI_STAT_TXUR | SPI_STAT_SSA | SPI_STAT_SSD);
    Chip_DMA_InitChannel( DMAREQ_SPI0_RX, DMA_ADDR(&LPC_SPI0->RXDAT), DMA_XFERCFG_SRCINC_0, 
                          DMA_ADDR(&pBufferR[1000]), DMA_XFERCFG_DSTINC_1, WIDTH_8_BITS,
                          lengthR-1000, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(1)));
    Chip_DMA_StartTransfer(DMAREQ_SPI0_RX, DMA_XFERCFG_SRCINC_0, DMA_XFERCFG_DSTINC_1, DMA_XFERCFG_WIDTH_8, lengthR - 1000);

    Chip_DMA_InitChannel( DMAREQ_SPI0_TX, DMA_ADDR(&pBufferW[1000]), DMA_XFERCFG_SRCINC_1,
                          DMA_ADDR(&LPC_SPI0->TXDAT), DMA_XFERCFG_DSTINC_0, WIDTH_8_BITS,
                          lengthW-1000, (DMA_CFG_PERIPHREQEN | DMA_CFG_BURSTPOWER_1 | DMA_CFG_CHPRIORITY(2)));
    Chip_DMA_StartTransfer(DMAREQ_SPI0_TX, DMA_XFERCFG_SRCINC_1, DMA_XFERCFG_DSTINC_0, DMA_XFERCFG_WIDTH_8, lengthW - 1000);    
  }  
  
#ifdef SEMAPHORE
  
  result = mico_rtos_get_semaphore( &spi_transfer_finished_semaphore, SEMDELAY );
 
#else
   while (bDMASPITXDoneFlag == false) { // &&(bDMASPIRXDoneFlag == false)) {
   //__WFI();
    asm("wfi");
  }
 while (bDMASPIRXDoneFlag == false) { // &&(bDMASPIRXDoneFlag == false)) {
    //__WFI();
    asm("wfi");
  }   
#endif

//  Chip_SPI_ClearStatus(LPC_SPI0, SPI_STAT_EOT);
  transfer_size = lengthR;
#endif
  
#ifdef SPIDEBUG
  if(lengthR > 1000) // 1023)
  printf("timer0_cnt = %d, size %d\r\n", LPC_CTIMER0->TC, lengthR);
#endif
//  printf("W %x, %x R %x, %x, lenW %d, lenR %d\r\n", pBufferW[0], pBufferW[1], pBufferR[0], pBufferR[1], lengthW, lengthR);
//  printf("BuffW 0x%x BuffR 0x%x\r\n", &pBufferW[lengthW-1], &pBufferR[lengthR-1]);
  spi_cs_Set(1);
  return transfer_size;
}

void SPI_IRQ_ENABLE(void)
{
  Chip_PININT_Init(LPC_PININT);
  Chip_IOCON_PinMuxSet(LPC_IOCON, GPIO_SPIINT_PORT, GPIO_SPIINT_PIN, (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN  | IOCON_GPIO_MODE));
  Chip_GPIO_SetPinDIRInput(LPC_GPIO, GPIO_SPIINT_PORT, GPIO_SPIINT_PIN);
  /* Configure pin interrupt selection for the GPIO pin in Input Mux Block */
  Chip_INMUX_PININT_Config(LPC_INMUX, GPIO_SPIINT_INDEX, GPIO_SPIINT_PORT, GPIO_SPIINT_PIN);
  /* Configure channel interrupt as edge sensitive and falling edge interrupt */
  Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH(GPIO_SPIINT_INDEX));
  Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH(GPIO_SPIINT_INDEX));
  Chip_PININT_EnableIntLow(LPC_PININT, PININTCH(GPIO_SPIINT_INDEX));
//  Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH(GPIO_SPIINT_INDEX));
  NVIC_SetPriority(PININT0_IRQn, 1);
  /* Enable interrupt in the NVIC */
  NVIC_ClearPendingIRQ(PININT0_IRQn);
  NVIC_EnableIRQ(PININT0_IRQn);
}

void SPI_IRQ_DISABLE(void)
{
  /* Enable interrupt in the NVIC */
  NVIC_ClearPendingIRQ(PININT0_IRQn);
  NVIC_DisableIRQ(PININT0_IRQn);
}

// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------

#include "spi_flash_platform_interface.h"
//#define SPIFLASHDEBUG 1
// Initial SPI Flash
#define SPIMASTERIRQHANDLER             SPI1_IRQHandler
#define LPC_SPIMASTERPORT               LPC_SPI1
#define LPC_SPIMASTERIRQNUM             SPI1_IRQn
/* The slave can handle data up to the point of overflow or underflow.
   Adjust this clock rate and the master's timing delays to get a working
   SPI slave rate. */
#define LPCMASTERCLOCKRATE              (4000000)
#define BUFFER_SIZE 300

#define SPI_FLASH_CMD_READ_DEVID        0x90
#define SPI_FLASH_CMD_WRITE_ENABLE      0x06
#define SPI_FLASH_CMD_WRITE_DISABLE     0x04

#define SPI_FLASH_CMD_READ_DATA         0x03
#define SPI_FLASH_CMD_READ_FAST         0x0B

#define SPI_FLASH_CMD_WRITE_DATA        0x02
#define SPI_FLASH_CMD_ERASE             0x20
#define SPI_FLASH_CMD_ERASE_32KB        0x52
#define SPI_FLASH_CMD_ERASE_64KB        0xD8
#define SPI_FLASH_CMD_ERASE_CHIP        0xC7

#define SPI_FLASH_CMD_WRITE_STATUS      0x01
#define SPI_FLASH_CMD_READ_STATUS1      0x05
#define SPI_FLASH_CMD_READ_STATUS2      0x35

#define SPI_FLASH_PAGESIZE              256  

volatile uint8_t Flash_mEnd;
/* Master transmit and receive buffers */
uint8_t masterRXBuffer8[BUFFER_SIZE], masterTXBuffer8[BUFFER_SIZE];

/* SPI master transfer descriptor */
SPIM_XFER_T flash_spi_masterxfer; //spiMasterXfer;

/* SPI master select assertion callback function */
static void FlashSPIMasterAssert(SPIM_XFER_T *pMasterXfer)
{
	/* Indicates tha master just asserted the slave select
	   signal */
}

/* SPI master send data callback function */
static void FlashSPIMasterSendData(SPIM_XFER_T *pMasterXfer)
{
	/* This callback is called when the master needs more data to
	   send. The pMasterXfer->pTXData buffer pointer and transfer size
	   (in items) in pMasterXfer->txCount should be updated. */

	/* If this function sets the pMasterXfer->terminate flag to true,
	   this function won't be called again and the transfer will
	   terminate when the current transmit buffer is complete. */

	/* This example sets up the entire transfer structure without
	   using this callback. */
}

/* SPI master receive data callback function */
static void FlashSPIMasterRecvData(SPIM_XFER_T *pMasterXfer)
{
	/* This callback is called when the master needs another receive
	   buffer. The pMasterXfer->pRXData buffer pointer and transfer size
	   (in items) in pMasterXfer->rxCount shoudl be updated. */

	/* This example sets up the entire transfer structure without
	   using this callback. */
}

/* SPI master select de-assertion callback function */
static void FlashSPIMasterDeAssert(SPIM_XFER_T *pMasterXfer)
{
	/* Indicates tha master just deasserted the slave select
	   signal */
}

/* SPI master transfer done callback */
static void FlashSPIMasterDone(SPIM_XFER_T *pMasterXfer)
{
  /* Indicates tha transfer is complete */
  Flash_mEnd = true;
}

/* SPI master driver callbacks */
static const SPIM_CALLBACKS_T FlashspiMasterCallbacks = {
	&FlashSPIMasterAssert,
	&FlashSPIMasterSendData,
	&FlashSPIMasterRecvData,
	&FlashSPIMasterDeAssert,
	&FlashSPIMasterDone
};


static void flash_spi_pinmux_init(void)
{
  /* Connect the SPI1 signals to port pins */
  Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 12,  (IOCON_FUNC4 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* SPI1_SCK */
  Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 13,  (IOCON_FUNC4 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* SPI1_MOSI */
  Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 14,  (IOCON_FUNC4 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* SPI1_MISO */
//  Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 15,  (IOCON_FUNC4 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* SPI1_SSEL0 */
  Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 16,  (IOCON_FUNC4 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));	/* SPI1_SSEL0 */
  
  Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 15,  (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));
  Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 15);
  Chip_GPIO_SetPinState(LPC_GPIO, 1, 15, 1);
}

/* Setup master controller */
static void flash_spi_master_set(void)
{
  SPI_CFGSETUP_T spiSetup;
  SPIM_DELAY_CONFIG_T masterDelay;

  /* Initialize SPI controller */
  Chip_SPI_Init(LPC_SPIMASTERPORT);

  /* Call to initialize first SPI controller for mode0, master mode,
     MSB first */
  Chip_SPI_Enable(LPC_SPIMASTERPORT);
  spiSetup.master = 1;
  spiSetup.lsbFirst = 0;
  spiSetup.mode = SPI_CLOCK_MODE0;
  Chip_SPI_ConfigureSPI(LPC_SPIMASTERPORT, &spiSetup);

  /* Setup master controller SSEL0 for active low select */
  Chip_SPI_SetCSPolLow(LPC_SPIMASTERPORT, 0);

  /* Setup master clock rate, slave clock doesn't need to be setup */
  Chip_SPIM_SetClockRate(LPC_SPIMASTERPORT, LPCMASTERCLOCKRATE);

  /* Setup master delay (all chip selects) */
  masterDelay.PreDelay = 0x1;
  masterDelay.PostDelay = 0x1;
  masterDelay.FrameDelay = 0x1;
  masterDelay.TransferDelay = 0x1;
  Chip_SPIM_DelayConfig(LPC_SPIMASTERPORT, &masterDelay);

  /* Setup master transfer callbacks in the transfer descriptor */
  flash_spi_masterxfer.pCB = &FlashspiMasterCallbacks;
  Chip_SPI_SetXferSize(LPC_SPI1, 8);
	/* Setup master trasnfer options - 16 data bits per transfer, EOT, EOF */
  spiMasterXfer.options =
		SPI_TXCTL_FLEN(8) |		/* This must be enabled as a minimum, use 16 data bits */
		// SPI_TXCTL_EOT |			/* Enable this to assert and deassert SSEL for each individual byte/word, current slave functions for this example do not support this */
		// SPI_TXCTL_EOF |			/* Insert a delay between bytes/words as defined by frame delay time */
		// SPI_TXCTL_RXIGNORE |		/* Enable this to ignore incoming data, or set spiMasterXfer.pRXData16 to NULL to ignore RX data  */
		0;
  /* For the SPI controller configured in master mode, enable SPI master interrupts
     for interrupt service. Do not enable SPI_INTENSET_TXDYEN. */
  Chip_SPI_EnableInts(LPC_SPIMASTERPORT, (SPI_INTENSET_RXDYEN |
                      SPI_INTENSET_RXOVEN | SPI_INTENSET_TXUREN | SPI_INTENSET_SSAEN |
                      SPI_INTENSET_SSDEN));

  


	/* Setup master trasnfer options - 16 data bits per transfer, EOT, EOF */
  flash_spi_masterxfer.options =
		SPI_TXCTL_FLEN(8) |		/* This must be enabled as a minimum, use 16 data bits */
		// SPI_TXCTL_EOT |			/* Enable this to assert and deassert SSEL for each individual byte/word, current slave functions for this example do not support this */
		// SPI_TXCTL_EOF |			/* Insert a delay between bytes/words as defined by frame delay time */
		// SPI_TXCTL_RXIGNORE |		/* Enable this to ignore incoming data, or set spiMasterXfer.pRXData16 to NULL to ignore RX data  */
		0;

  /* Transfer will terminate after current buffer is sent. If terminate is not set, the buffers
    must be setup by the callbacks		*/
  flash_spi_masterxfer.terminate = true;

  /* Use SPI select 0 */
  flash_spi_masterxfer.sselNum = 0;
  
  Chip_SPI_Enable(LPC_SPIMASTERPORT);
  /* For the SPI controller configured in master mode, enable SPI master interrupts
     for interrupt service. Do not enable SPI_INTENSET_TXDYEN. */
  Chip_SPI_EnableInts(LPC_SPIMASTERPORT, (SPI_INTENSET_RXDYEN |
                      SPI_INTENSET_RXOVEN | SPI_INTENSET_TXUREN | SPI_INTENSET_SSAEN |
                      SPI_INTENSET_SSDEN));	
  Chip_SPI_FlushFifos(LPC_SPIMASTERPORT);	
  NVIC_EnableIRQ(LPC_SPIMASTERIRQNUM);
}

/**
 * @brief	SPI0 interrupt handler sub-routine (master)
 * @return	Nothing
 */
void SPI1_IRQHandler(void)
{
  uint32_t ints = Chip_SPI_GetPendingInts(LPC_SPIMASTERPORT);

  /* Handle SPI slave interrupts only */
  if ((ints & (SPI_INTENSET_RXDYEN | SPI_INTENSET_RXOVEN |
       SPI_INTENSET_TXUREN | SPI_INTENSET_SSAEN | SPI_INTENSET_SSDEN)) != 0) {
    /* SPI slave handler */
    Chip_SPIM_XferHandler(LPC_SPIMASTERPORT, &flash_spi_masterxfer);
  }
}

static uint8_t spi_flash_init(void)
{
  uint8_t ret = 0;
  
   /* SPI initialization */
  flash_spi_pinmux_init();
  /* Enable SysTick Timer */
  /* Setup SPI controllers */
  flash_spi_master_set();
  ret = 1;
  return ret;
}


int sflash_platform_init( int peripheral_id, void** platform_peripheral_out )
{
  spi_flash_init();
  __disable_irq();
  printf("Spi flash init ok\r\n");
  __enable_irq();
  return 0;
}

int sflash_platform_send_recv_byte( void* platform_peripheral, unsigned char MOSI_val, void* MISO_addr )
{
//    /* Wait until the SPI Data Register is empty */
//    while ( SPI_I2S_GetFlagStatus( SFLASH_SPI, SPI_I2S_FLAG_TXE ) == RESET )
//    {
//        /* wait */
//    }
//
//
//    SPI_I2S_SendData( SFLASH_SPI, MOSI_val );
//
//    /* Wait until the SPI peripheral indicates the received data is ready */
//    while ( SPI_I2S_GetFlagStatus( SFLASH_SPI, SPI_I2S_FLAG_RXNE ) == RESET )
//    {
//        /* wait */
//    }
//
//    /* read the received data */
//    char x  = SPI_I2S_ReceiveData( SFLASH_SPI );
//
//    *( (char*) MISO_addr ) = x;
  int ret;
#ifdef SPIFLASHDEBUG
  printf("SPI RW %x %d\r\n", MOSI_val, 1);
#endif
  flash_spi_masterxfer.pTXData8 = &MOSI_val;
  flash_spi_masterxfer.txCount  = 1;/* Count is in transfer size */	
  flash_spi_masterxfer.pRXData8 = MISO_addr;
  flash_spi_masterxfer.rxCount  = 1;/* Count is in transfer size */	
/* Start master transfer */
  Chip_SPIM_Xfer(LPC_SPI1, &flash_spi_masterxfer);	
  asm("wfi");
  while (flash_spi_masterxfer.dataTXferred < 1) {
    asm("wfi");
  }
  asm("wfi");
  for(ret =0; ret<100; ret++);
#ifdef SPIFLASHDEBUG
   printf("Wait SPI RW end");
#endif
  if(flash_spi_masterxfer.dataTXferred == 1) {
    asm("wfi");
    Flash_mEnd = true;
  }
  for(ret =0; ret<100; ret++);
#ifdef SPIFLASHDEBUG
  printf("SPI RW end");
#endif
  return 0;
}

int sflash_platform_chip_select( void* platform_peripheral )
{
  Chip_GPIO_SetPinState(LPC_GPIO, 1, 15, 0);
  return 0;
}

int sflash_platform_chip_deselect( void* platform_peripheral )
{
  Chip_GPIO_SetPinState(LPC_GPIO, 1, 15, 1);
  return 0;
}


// end file
