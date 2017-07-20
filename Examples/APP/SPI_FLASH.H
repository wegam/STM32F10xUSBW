#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

//#include "STM32_USART.H"
//#include "STM32_TIM.H"
//#include "STM32_DMA.H"
//#include "STM32_PWM.H"
//#include "STM32_GPIO.H"
//#include "STM32_ADC.H"
//#include "STM32_DAC.H"
//#include "STM32_SPI.H"

#include "stm32f10x_type.h"
#include "stm32f10x_gpio.h"

#include	"stdio.h"				//用于printf
#include	"string.h"			//用于printf
#include	"stdarg.h"			//用于获取不确定个数的参数
#include	"stdlib.h"			//malloc动态申请内存空间
	
#include	"stddef.h"
#include	"stdint.h"

#include "STM32F10x_BitBand.H"

#include "STM32_SYSTICK.H"


void SPI_FLASH_Configuration(void);
void SPI_FLASH_Server(void);
void SPI_FLASH_GPIO_Configuration(void);
void SPI_FLASH_SpiPort_Configuration(void);
u8 SPI_FLASH_SendByte(u8 byte);
u32 SPI_FLASH_ReadID(void);







#endif

