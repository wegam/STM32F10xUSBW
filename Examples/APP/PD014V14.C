/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : PC001V21.c
* Author             : WOW
* Version            : V2.0.1
* Date               : 06/26/2017
* Description        : PC001V21����ư�.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifdef PD014V14			//�м���ҩ���ư�

#include "PD014V14.H"

#include "DRV8801.H"
#include "A3987.H"

#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_PWM.H"

#include "string.h"				//�����ڴ��������ͷ�ļ�
#include "stm32f10x_dma.h"

u16	DelayTime=0;
u16	StepTime=0;

u16 Lock_Toggle_CNT=0;


void PD014V14_PinSet(void);
void Lock_Toggle(void);			//˫�����������
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PD014V14_Configuration(void)
{
	SYS_Configuration();					//ϵͳ����---��ϵͳʱ�� STM32_SYS.H
	
	GPIO_DeInitAll();							//�����е�GPIO�ر�----V20170605
	
	PD014V14_PinSet();
	
	SysTick_Configuration(500);	//ϵͳ���ʱ������72MHz,��λΪuS
	
	IWDG_Configuration(1000);			//�������Ź�����---������λms	
	
	PWM_OUT(TIM2,PWM_OUTChannel1,1,900);						//PWM�趨-20161127�汾--SYS-LED
	
	P_Sens=1;

}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PD014V14_Server(void)
{	
	IWDG_Feed();								//�������Ź�ι��
//	DelayTime++;
//	StepTime++;

	Lock_Toggle();			//˫�����������
//	GPIO_Toggle	(GPIOC,	GPIO_Pin_1);	//������
//	if(DelayTime>=500)
//	{
//		DelayTime=0;
//		
//	}
//	if(StepTime>=2000)
//	{
//		StepTime=0;
////		GPIO_Toggle	(GPIOA,	GPIO_Pin_5);		//��GPIO��Ӧ�ܽ������ת----V20170605
//	}
}

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PD014V14_PinSet(void)
{	
	//��ҩ���������
	GPIO_Configuration_OPP50	(GPIOA,	GPIO_Pin_7);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOC,	GPIO_Pin_5);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOB,	GPIO_Pin_1);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOB,	GPIO_Pin_11);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOB,	GPIO_Pin_13);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOB,	GPIO_Pin_15);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOC,	GPIO_Pin_7);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	GPIO_Configuration_OPP50	(GPIOC,	GPIO_Pin_9);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	
	//��ҩ���������翪��
	GPIO_Configuration_OPP50	(GPIOA,	GPIO_Pin_5);			//��GPIO��Ӧ�ܽ�����ΪPP(����)���ģʽ������ٶ�50MHz----V20170605
	
	//��ҩ������
	GPIO_Configuration_IPU(GPIOA,	GPIO_Pin_6);					//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605
	GPIO_Configuration_IPU(GPIOC,	GPIO_Pin_4);					//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605
	GPIO_Configuration_IPU(GPIOB,	GPIO_Pin_0);					//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605
	GPIO_Configuration_IPU(GPIOB,	GPIO_Pin_10);					//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605
	GPIO_Configuration_IPU(GPIOB,	GPIO_Pin_12);					//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605
	GPIO_Configuration_IPU(GPIOB,	GPIO_Pin_14);					//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605
	GPIO_Configuration_IPU(GPIOC,	GPIO_Pin_6);					//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605
	GPIO_Configuration_IPU(GPIOC,	GPIO_Pin_8);					//��GPIO��Ӧ�ܽ�����Ϊ��������ģʽ----V20170605	
}
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Lock_Toggle(void)
{
	Lock_Toggle_CNT++;
	if(Lock_Toggle_CNT>=2000)
	{
		Lock_Toggle_CNT=0;
		ct_pmos1=0;
		ct_pmos2=0;
		ct_pmos3=0;
		ct_pmos4=0;
		ct_pmos5=0;
		ct_pmos6=0;
		ct_pmos7=0;
		ct_pmos8=0;
	}
	else if(Lock_Toggle_CNT==100)
	{
		if(Sens_In1)
			ct_pmos1=1;
		if(Sens_In2)
			ct_pmos2=1;
		if(Sens_In3)
			ct_pmos3=1;
		if(Sens_In4)
			ct_pmos4=1;
		if(Sens_In5)
			ct_pmos5=1;
		if(Sens_In6)
			ct_pmos6=1;
		if(Sens_In7)
			ct_pmos7=1;
		if(Sens_In8)
			ct_pmos8=1;
		
	}
	else if(Lock_Toggle_CNT==200)
	{
		ct_pmos1=0;
		ct_pmos2=0;
		ct_pmos3=0;
		ct_pmos4=0;
		ct_pmos5=0;
		ct_pmos6=0;
		ct_pmos7=0;
		ct_pmos8=0;		
	}
	else if(Lock_Toggle_CNT>=300)
	{
		ct_pmos1=0;
		ct_pmos2=0;
		ct_pmos3=0;
		ct_pmos4=0;
		ct_pmos5=0;
		ct_pmos6=0;
		ct_pmos7=0;
		ct_pmos8=0;
	}

}
#endif