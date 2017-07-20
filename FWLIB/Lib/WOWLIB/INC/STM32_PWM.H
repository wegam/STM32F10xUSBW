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


#ifndef __STM32_PWM_H
#define __STM32_PWM_H



#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

//#include "STM32_TIM.H"


typedef enum
{
  PWM_OUTChannel1	=	0,
	PWM_OUTChannel2,
	PWM_OUTChannel3,
	PWM_OUTChannel4,
} PWM_OUTChannelTypeDef;



//void PWM_Configuration(TIM_TypeDef* TIMx,u16 Prescaler,u16 Period,u8 PWM_RATIO);
//void PWM_Configuration0(TIM_TypeDef* TIMx,u16 Prescaler,u16 Period,u8 PWM_RATIO);
void PWM_ClockCmd(TIM_TypeDef* TIMx,u16 Prescaler,u16 Period,u8 PWM_RATIO);		//�������GPIOʱ��	
void PWM_GPIO_Configuration(void);			//PWMʹ�õ�GPIO����
void PWM_SET(TIM_TypeDef* TIMx,u8 PWM_RATIO);	//PWM�趨
void PWM_OUT(TIM_TypeDef* TIMx,PWM_OUTChannelTypeDef PWM_OUTChanneln,double PWM_Frequency ,u16 PWM_Ratio);	//PWM�趨-20161127�汾
#endif