/**
******************************************************************************
* @file    hardfault.c 
* @author  William Xu
* @version V1.0.0
* @date    05-May-2014
* @brief   This file provide debug information in hardfault.
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
#ifdef STM32F4XX
#include "stdio.h"
#include "stm32f4xx.h"
#include "MicoRTOS.h"
#include "MicoDefaults.h"
#include "MicoPlatform.h"
#include "platform_common_config.h"
#include "stm32f4xx_platform.h"

OSStatus stdio_hardfault( char* data, uint32_t size )
{
#ifndef MICO_DISABLE_STDIO
  uint32_t idx;
  for(idx = 0; idx < size; idx++){
    while ( ( uart_mapping[ STDIO_UART ].usart->SR & USART_SR_TXE ) == 0 );
    uart_mapping[ STDIO_UART ].usart->DR = (data[idx] & (uint16_t)0x01FF);
    
  }
#endif
  return kNoErr;
}

void hard_fault_handler_c (unsigned int * hardfault_args)
{
  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;
  char logString[50];

  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);
 
  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);
 
  sprintf (logString, "\r\n\r\n[Hard fault handler - all numbers in hex]\r\n");
  stdio_hardfault( logString, strlen(logString)+1 );
  sprintf (logString, "R0 = 0x%x\r\n", stacked_r0);
  stdio_hardfault( logString, strlen(logString)+1 );
  sprintf (logString, "R1 = 0x%x\r\n", stacked_r1);
  stdio_hardfault( logString, strlen(logString)+1 );
  sprintf (logString, "R2 = 0x%x\r\n", stacked_r2);
  stdio_hardfault( logString, strlen(logString)+1 );
  sprintf (logString, "R3 = 0x%x\r\n", stacked_r3);
  stdio_hardfault( logString, strlen(logString)+1 );
  sprintf (logString, "R12 = 0x%x\r\n", stacked_r12);
  stdio_hardfault( logString, strlen(logString)+1 );
  sprintf (logString, "LR [R14] = 0x%x  subroutine call return address\r\n", stacked_lr);
  stdio_hardfault( logString, strlen(logString)+1 );
  sprintf (logString, "PC [R15] = 0x%x  program counter\r\n", stacked_pc);
  stdio_hardfault( logString, strlen(logString)+1 );
  sprintf (logString, "PSR = 0x%x\r\n", stacked_psr);
  stdio_hardfault( logString, strlen(logString)+1 );
  sprintf (logString, "BFAR = 0x%x\r\n", (*((volatile unsigned long *)(0xE000ED38))));
  stdio_hardfault( logString, strlen(logString)+1 );
  sprintf (logString, "CFSR = 0x%x\r\n", (*((volatile unsigned long *)(0xE000ED28))));
  stdio_hardfault( logString, strlen(logString)+1 );
  sprintf (logString, "HFSR = 0x%x\r\n", (*((volatile unsigned long *)(0xE000ED2C))));
  stdio_hardfault( logString, strlen(logString)+1 );
  sprintf (logString, "DFSR = 0x%x\r\n", (*((volatile unsigned long *)(0xE000ED30))));
  stdio_hardfault( logString, strlen(logString)+1 );
  sprintf (logString, "AFSR = 0x%x\r\n", (*((volatile unsigned long *)(0xE000ED3C))));
  stdio_hardfault( logString, strlen(logString)+1 );
  sprintf (logString, "SCB_SHCSR = 0x%x\r\n", SCB->SHCSR);
  stdio_hardfault( logString, strlen(logString)+1 );
 
  while (1);
}

#else
#ifdef LPC54XX
/*
#include "stdio.h"

#include "MicoRTOS.h"
#include "MicoDefaults.h"
#include "MicoPlatform.h"
#include "platform_common_config.h"
#include "nxp_platform.h"
*/
#include "board.h"

struct exception_stack_frame
{
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;
	uint32_t psr;
};


void hard_fault_handler_c (struct exception_stack_frame* contex)
{
	DEBUGOUT("psr: 0x%08x\n", contex->psr);
	DEBUGOUT(" pc: 0x%08x\n", contex->pc);
	DEBUGOUT(" lr: 0x%08x\n", contex->lr);
	DEBUGOUT("r12: 0x%08x\n", contex->r12);
	DEBUGOUT("r03: 0x%08x\n", contex->r3);
	DEBUGOUT("r02: 0x%08x\n", contex->r2);
	DEBUGOUT("r01: 0x%08x\n", contex->r1);
	DEBUGOUT("r00: 0x%08x\n", contex->r0);  
 
  while (1);
}
#endif

#endif
