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


#ifndef __STM32_TIM_H
#define __STM32_TIM_H

#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_nvic.h"
#include "stm32f10x_gpio.h"
//#include "STM32_WOW.H"


void TIMx_RCC_ENABLE(TIM_TypeDef* TIMx);	//��ʱ������
void TIM_TIME_SET(TIM_TypeDef* TIMx,u16 Prescaler,u16 Period);	//��ʱʱ���趨
void TIM_Configuration(TIM_TypeDef* TIMx,u16 Prescaler,u16 Period);	//��ʱ������
void TIM_Interrupt(TIM_TypeDef* TIMx,u16 Prescaler,u16 Period);
void TIM_Server(void);






#endif

