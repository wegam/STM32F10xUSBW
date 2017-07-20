/******************************** User_library *********************************
* �ļ��� 	: STM32_SDCard.H
* ����   	: wegam@sina.com
* �汾   	: V
* ����   	: 2016/01/01
* ˵��   	: 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


#ifndef __STM32_EXTI_H
#define __STM32_EXTI_H

#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_nvic.h"
#include "stm32f10x_rcc.h"
#include "math.h"

//#include "STM32_WOW.H"
//#include "stdio.h"

void EXTI_Configuration(GPIO_TypeDef* GPIOx, u16 GPIO_Pin,EXTIMode_TypeDef Mode);
void EXTI_ClockConf(GPIO_TypeDef* GPIOx, u16 GPIO_Pin_x);						//�������GPIOʱ��	
void EXTI_PortSourceConf(GPIO_TypeDef* GPIOx,u8 *GPIO_PortSource);	//�����ж���
void EXTI_PinConf(GPIO_TypeDef* GPIOx, u16 GPIO_Pin_x,u8 *EXTI_IRQChannel);			//ѡ������ж���·
void EXTI_InitStructure(GPIO_TypeDef* GPIOx, u16 GPIO_Pin_x,EXTIMode_TypeDef Mode,u8 EXTI_IRQChannel);		//�ⲿ�жϳ�ʼ��
void EXTI_Interrupt_Configuration(u32 EXTI_Line);
void EXTI_Server(void);
#endif

