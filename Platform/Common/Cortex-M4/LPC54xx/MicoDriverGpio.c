/**
******************************************************************************
* @file    MicoDriverGpio.c 
* @author  William Xu
* @version V1.0.0
* @date    05-May-2014
* @brief   This file provide GPIO driver functions.
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


#include "MICOPlatform.h"
#include "MICORTOS.h"
#include "gpio_irq.h"

#include "platform.h" //gpio conf in board.h
#include "platform_common_config.h"
#include "PlatformLogging.h"
//#include "chip.h"

/******************************************************
*                    Constants
******************************************************/

/******************************************************
*                   Enumerations
******************************************************/

/******************************************************
*                 Type Definitions
******************************************************/

/******************************************************
*                    Structures
******************************************************/

/******************************************************
*               Variables Definitions
******************************************************/

/******************************************************
*               Function Declarations
******************************************************/

/******************************************************
*               Function Definitions
******************************************************/

OSStatus MicoGpioInitialize( mico_gpio_t gpio, mico_gpio_config_t configuration )
{
  
  
  return kNoErr;
}

OSStatus MicoGpioFinalize( mico_gpio_t gpio )
{
  
  
  return kNoErr;
}

OSStatus MicoGpioOutputHigh( mico_gpio_t gpio )
{
 //Chip_GPIO_SetPinOutHigh();
  
  return kNoErr;
}

OSStatus MicoGpioOutputLow( mico_gpio_t gpio )
{
  
  return kNoErr;
}

OSStatus MicoGpioOutputTrigger( mico_gpio_t gpio )
{
  
  
  return kNoErr;    
}

bool MicoGpioInputGet( mico_gpio_t gpio )
{

  
  return kNoErr;
}

OSStatus MicoGpioEnableIRQ( mico_gpio_t gpio, mico_gpio_irq_trigger_t trigger, mico_gpio_irq_handler_t handler, void* arg )
{

  return 0;
}

OSStatus MicoGpioDisableIRQ( mico_gpio_t gpio )
{

  return 0;
}
