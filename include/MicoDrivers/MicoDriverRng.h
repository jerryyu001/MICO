/**
  ******************************************************************************
  * @file    MICORTOS.h 
  * @author  William Xu
  * @version V1.0.0
  * @date    05-May-2014
  * @brief   This file provides all the headers of GPIO operation functions.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, MXCHIP Inc. SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2014 MXCHIP Inc.</center></h2>
  ******************************************************************************
  */ 

#ifndef __MICODRIVERRNG_H__
#define __MICODRIVERRNG_H__

#pragma once
#include "Common.h"
#include "platform.h"

/******************************************************
 *                   Macros
 ******************************************************/  
//#define host_platform_rand MicoRandomNumberRead
#define PlatformRandomBytes MicoRandomNumberRead
/******************************************************
 *                   Enumerations
 ******************************************************/


/******************************************************
 *                 Type Definitions
 ******************************************************/


/*****************************************************************************/
/** @addtogroup rng       Random number generater
 *  @ingroup platform
 *
 * Random number generater (RNG) Functions
 *
 *
 *  @{
 */
/*****************************************************************************/


int MicoRandomNumberRead( void *inBuffer, int inByteCount );

/** @} */

#endif


