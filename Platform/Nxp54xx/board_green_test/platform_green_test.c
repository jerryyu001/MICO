


#include "MICOPlatform.h"

#include "PlatformLogging.h"




OSStatus mico_platform_init( void )
{
  platform_log( "mico_platform_init()" );
  //platform_log( "Platform initialised" );
  
  return kNoErr;
}

void init_platform( void )
{
  platform_log( "init_platform" );
  RESET_GPIO_Init();
  MicoFlashInitialize( MICO_SPI_FLASH );
}

void init_platform_bootloader( void )
{
  platform_log( "init_platform_bootloader" );
  
}

void host_platform_reset_wifi( bool reset_asserted )
{
  platform_log( "host_platform_reset_wifi" );
  if (reset_asserted)
    RESET_GPIO_Set(0);
  else
    RESET_GPIO_Set(1);
}

void host_platform_power_wifi( bool power_enabled )
{
  platform_log( "host_platform_power_wifi" );
  
}

void MicoSysLed(bool onoff)
{
 // Board_LED_Set(0, onoff);
  Chip_GPIO_SetPinState(LPC_GPIO, 0, 29, onoff);
}

void MicoRfLed(bool onoff)
{
 // Board_LED_Set(1, onoff);
  Chip_GPIO_SetPinState(LPC_GPIO, 0, 30, onoff);
}

void Mico2rdLED(bool onoff)
{
  platform_log( "Mico2rdLED" );
  //Board_LED_Set(2, onoff);
   Chip_GPIO_SetPinState(LPC_GPIO, 0, 31, onoff);
}


bool MicoShouldEnterMFGMode(void)
{
  platform_log( "MicoShouldEnterMFGMode" );
  //if(MicoGpioInputGet((mico_gpio_t)BOOT_SEL)==false && MicoGpioInputGet((mico_gpio_t)MFG_SEL)==false)
  //  return true;
  //else
    return false;
}



