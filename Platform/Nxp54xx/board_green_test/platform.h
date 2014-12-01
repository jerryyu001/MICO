

#ifndef __PLATFORM_H__
#define __PLATFORM_H__
//#include "board.h" //this include chip.h which include all drivers
#include "chip.h"


#ifndef LPC54XX
#define LPC54XX
#endif

#define DEBUG_ENABLE

/** Board UART used for debug output and input using the DEBUG* macros. This
    is also the port used for Board_UARTPutChar, Board_UARTGetChar, and
    Board_UARTPutSTR functions. Although you can setup multiple UARTs here,
    the board code only supoprts UART0 in the Board_UART_Init() fucntion,
    so be sure to change it there too if not using UART0.
 */
#define DEBUG_UART                      LPC_USART0

/* Board name */
#define BOARD_NXP_LPCXPRESSO_54000

//#include "board_api.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define MCU_CLOCK_HZ            100000000
  
#define HARDWARE_REVISION   "1062_1"
#define DEFAULT_NAME        "NXP EVB"
#define MODEL               "NXP_EVB_1"


#define USE_MICO_SPI_FLASH
//#define SFLASH_SUPPORT_MACRONIX_PART 
//#define SFLASH_SUPPORT_SST_PARTS
#define SFLASH_SUPPORT_WINBOND_PARTS

/* WLAN Powersave Clock Source
 * The WLAN sleep clock can be driven from one of two sources:
 * 1. Timer/PWM (default)
 *    - With the PWM selected, the STM32 can *NOT* be put into MCU powersave mode or the PWM output will be disabled
 * 2. MCO (MCU Clock Output). 
 *    - Change the following directive to MICO_WLAN_POWERSAVE_CLOCK_IS_MCO
 */
#define MICO_WLAN_POWERSAVE_CLOCK_SOURCE MICO_WLAN_POWERSAVE_CLOCK_IS_MCO

#define RestoreDefault_TimeOut          3000  /**< Restore default and start easylink after 
                                                   press down EasyLink button for 3 seconds. */

/* MICO RTOS tick rate in Hz */
#define MICO_DEFAULT_TICK_RATE_HZ                   (1000)
/************************************************************************
 * Uncomment to disable watchdog. For debugging only */
//#define MICO_DISABLE_WATCHDOG

/************************************************************************
 * Uncomment to disable standard IO, i.e. printf(), etc. */
//#define MICO_DISABLE_STDIO

/************************************************************************
 * Uncomment to disable MCU powersave API functions */
//#define MICO_DISABLE_MCU_POWERSAVE

/************************************************************************
 * Uncomment to enable MCU real time clock */
#define MICO_ENABLE_MCU_RTC

#define MICO_WLAN_POWERSAVE_CLOCK_IS_NOT_EXIST  0
#define MICO_WLAN_POWERSAVE_CLOCK_IS_PWM        1 //Jer diff from MICO_1119
#define MICO_WLAN_POWERSAVE_CLOCK_IS_MCO        2 //
#define WLAN_POWERSAVE_CLOCK_FREQUENCY 32768 /* 32768Hz        */
#define WLAN_POWERSAVE_CLOCK_DUTY_CYCLE   50 /* 50% duty-cycle */
/******************************************************
 *                    Constants
 ******************************************************/


#include "board_api.h" //TBD! Jer

typedef enum
{
  MICO_GPIO_UNUSED = -1,
  MICO_GPIO_WLAN_POWERSAVE_CLOCK = 0,
  WL_GPIO0,
  WL_GPIO1,
  WL_REG_RESERVED, //Jer copy from MICO_1119 NXP
  WL_RESET,
  MICO_SYS_LED,
  MICO_RF_LED,
  BOOT_SEL,
  MFG_SEL,
//  Standby_SEL, 	//Jer temp
  EasyLink_BUTTON,
//  STDIO_UART_RX,  //Jer temp
//  STDIO_UART_TX,  //Jer temp
//  STDIO_UART_CTS,  //Jer temp
//  STDIO_UART_RTS,  //Jer temp
  MICO_COMMON_GPIO_MAX,
} mico_common_gpio_t;
// these for EMW3162 board , 1062 maybe don't need these.Jer1201
typedef enum
{
    MICO_GPIO_1 = MICO_COMMON_GPIO_MAX,
    MICO_GPIO_2,
    MICO_GPIO_MAX, /* Denotes the total number of GPIO port aliases. Not a valid GPIO alias */
} mico_gpio_t;

typedef enum
{
    MICO_SPI_1,
    MICO_SPI_MAX, /* Denotes the total number of SPI port aliases. Not a valid SPI alias */
} mico_spi_t;
typedef enum
{
    MICO_I2C_1,
    MICO_I2C_MAX, /* Denotes the total number of SPI port aliases. Not a valid SPI alias */
} mico_i2c_t;
typedef enum
{
    MICO_ADC_1,
    MICO_ADC_MAX, /* Denotes the total number of SPI port aliases. Not a valid SPI alias */
} mico_adc_t;
typedef enum
{
    MICO_PWM_1,
    MICO_PWM_MAX, /* Denotes the total number of SPI port aliases. Not a valid SPI alias */
} mico_pwm_t;
typedef enum
{
    MICO_UART_1,
    MICO_UART_MAX, /* Denotes the total number of SPI port aliases. Not a valid SPI alias */
} mico_uart_t;
typedef enum
{
    MICO_SPI_FLASH,
    MICO_INTERNAL_FLASH,
    MICO_FLASH_MAX,
} mico_flash_t;

#endif // __PLATFORM_H__
