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


#include "STM32_EXTI.H"

#include "STM32_WOW.H"
//#include "STM32F10x_BitBand.H"

//EXTI�ṹ��
//typedef struct
//{
//  u32 EXTI_Line;
//  EXTIMode_TypeDef EXTI_Mode;
//  EXTITrigger_TypeDef EXTI_Trigger;
//  FunctionalState EXTI_LineCmd;
//}EXTI_InitTypeDef;


//u8 a=0;
/*******************************************************************************
* ������		:	EXTI_Configuration	
* ��������	:	�ⲿ�ж����� 
* ����		:	GPIOx��xΪA...G
						GPIO_PinΪGPIO_Pin_x��xȡֵΪ0...15
						Mode���ж�ģʽ��ȡֵΪ EXTI_Mode_Interrupt�������ж�
																	EXTI_Mode_Event���¼��ж�
* ���		:
* ���� 		:
*******************************************************************************/
void EXTI_Configuration(GPIO_TypeDef* GPIOx, u16 GPIO_Pin_x,EXTIMode_TypeDef Mode)
{
	//1)**********������ر���	
//	GPIO_InitTypeDef GPIO_InitStructure;				//GPIO�ṹ��
	u8 GPIO_PortSource=0;
	u8 GPIO_PinSource=0;
	u8 EXTI_IRQChannel=0;
//	u32 EXTI_Line=(u32)GPIO_Pin_x;
	
	//2)**********������
	/* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
	assert_param(IS_GPIO_PIN(GPIO_Pin_x));
	//A---------�˿�ѡ�񼰴����GPIOʱ��
	EXTI_ClockConf(GPIOx,GPIO_Pin_x);				//�������GPIOʱ��	
	
	//---------����ѡ��---��ָ��
	//	GPIO_PinSource=(u8)log((double)GPIO_Pin)/log((double)2);		//���׹�ʽ�����	
	GPIO_PinSource=(u8)(log(GPIO_Pin_x)/log(2));		//���׹�ʽ�����		
	
	//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource3);
	GPIO_EXTILineConfig(GPIO_PortSource,GPIO_PinSource);
	
	
	EXTI_PortSourceConf(GPIOx,&GPIO_PortSource);									//�����ж���
	EXTI_PinConf(GPIOx,GPIO_Pin_x,&EXTI_IRQChannel);							//ѡ������ж���·
	EXTI_InitStructure(GPIOx,GPIO_Pin_x,Mode,EXTI_IRQChannel);		//�ⲿ�жϳ�ʼ��
	//3)**********ѡ��GPIO�ܽ������ⲿ�ж���·,�����GPIOʱ��
	
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void EXTI_ClockConf(GPIO_TypeDef* GPIOx, u16 GPIO_Pin_x)				//�������GPIOʱ��	
{
	switch (*(u32*)&GPIOx)
  {
    case GPIOA_BASE:
//			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		if((GPIO_Pin_x==GPIO_Pin_13)||(GPIO_Pin_x==GPIO_Pin_14)||(GPIO_Pin_x==GPIO_Pin_All))
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
			GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);		//�ر�JTAG
		}
		else
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
      break;

    case GPIOB_BASE:
		if((GPIO_Pin_x==GPIO_Pin_3)||(GPIO_Pin_x==GPIO_Pin_4)||(GPIO_Pin_x==GPIO_Pin_All))
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
			GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);		//�ر�JTAG
		}
		else
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
      break;

    case GPIOC_BASE:
			if((GPIO_Pin_x==GPIO_Pin_14)||(GPIO_Pin_x==GPIO_Pin_15))
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_AFIO, ENABLE);
			else
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
      break;

    case GPIOD_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
      break;
      
    case GPIOE_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
      break; 

    case GPIOF_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
      break;

    case GPIOG_BASE:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
      break;                       

    default:
      break;
  }

}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void EXTI_PortSourceConf(GPIO_TypeDef* GPIOx,u8 *GPIO_PortSource)	//�����ж���
{
//	u8 GPIO_PortSource=0;
	
//	GPIO_PortSourceP=&GPIO_PortSource;
	
	switch (*(u32*)&GPIOx)
  {
    case GPIOA_BASE:
      *GPIO_PortSource=GPIO_PortSourceGPIOA;
      break;

    case GPIOB_BASE:
      *GPIO_PortSource=GPIO_PortSourceGPIOB;
      break;

    case GPIOC_BASE:
      *GPIO_PortSource=GPIO_PortSourceGPIOC;
      break;

    case GPIOD_BASE:
      *GPIO_PortSource=GPIO_PortSourceGPIOD;
      break;
      
    case GPIOE_BASE:
      *GPIO_PortSource=GPIO_PortSourceGPIOE;
      break; 

    case GPIOF_BASE:
      *GPIO_PortSource=GPIO_PortSourceGPIOF;
      break;

    case GPIOG_BASE:
      *GPIO_PortSource=GPIO_PortSourceGPIOG;
      break;                       

    default:
      break;
  }

}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void EXTI_PinConf(GPIO_TypeDef* GPIOx, u16 GPIO_Pin_x,u8 *EXTI_IRQChannel)			//ѡ������ж���·
{
	
	
	if(GPIO_Pin_x<0x0020)
	{
		switch(GPIO_Pin_x)
		{
			case GPIO_Pin_0:
				*EXTI_IRQChannel=EXTI0_IRQChannel;
			break;
			case GPIO_Pin_1:
				*EXTI_IRQChannel=EXTI1_IRQChannel;
			break;
			case GPIO_Pin_2:
				*EXTI_IRQChannel=EXTI2_IRQChannel;
			break;
			case GPIO_Pin_3:
				*EXTI_IRQChannel=EXTI3_IRQChannel;			
				if((*(u32*)&GPIOx)==GPIOB_BASE)
				{
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	
					GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);		//�ر�JTAG
				}
			break;
			case GPIO_Pin_4:
				*EXTI_IRQChannel=EXTI4_IRQChannel;
				if((*(u32*)&GPIOx)==GPIOB_BASE)
				{
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	
					GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);		//�ر�JTAG
				}
			break;
				
			default:
				break;
		}
	}
	else if(GPIO_Pin_x>0x0010&&GPIO_Pin_x<0x0400)
	{
		*EXTI_IRQChannel=EXTI9_5_IRQChannel;
	}
	else
	{
		*EXTI_IRQChannel=EXTI15_10_IRQChannel;
		if((GPIO_Pin_13||GPIO_Pin_14||GPIO_Pin_15)&&((*(u32*)&GPIOx)==GPIOA_BASE))
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	
			GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);		//�ر�JTAG
		}
	}

}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void EXTI_InitStructure(GPIO_TypeDef* GPIOx, u16 GPIO_Pin_x,EXTIMode_TypeDef Mode,u8 EXTI_IRQChannel)		//�ⲿ�жϳ�ʼ��
{
	GPIO_InitTypeDef GPIO_InitStructure;				//GPIO�ṹ��
	EXTI_InitTypeDef EXTI_Initstructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	u32 EXTI_Line=(u32)GPIO_Pin_x;
	
	//4)**********GPIO����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_x;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOx,&GPIO_InitStructure);
	
	EXTI_Initstructure.EXTI_Line=EXTI_Line;												//�ⲿ�ж���·
	EXTI_Initstructure.EXTI_Mode=Mode;														//�ж�ģʽ
//	EXTI_Initstructure.EXTI_Mode=EXTI_Mode_Interrupt;						//�ж�ģʽ
	EXTI_Initstructure.EXTI_Trigger=EXTI_Trigger_Falling;					//������ʽ-
	EXTI_Initstructure.EXTI_LineCmd=ENABLE;												//����ʹ��
	EXTI_Init(&EXTI_Initstructure);
	
	EXTI_GenerateSWInterrupt(EXTI_Line);													//ʹ���ж�
	
	//7)**********�����ж�
	NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQChannel;					//ѡ���ж�ͨ��-�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;			//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;						//��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ��
	NVIC_Init(&NVIC_InitStructure);	

	EXTI_ClearITPendingBit(EXTI_Line);

}

/*******************************************************************************
* ������		:	
* ��������	:	�ⲿ�ж� 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void EXTI_Interrupt_Configuration(u32 EXTI_Line)
{
	EXTI_InitTypeDef EXTI_Initstructure;			
	
	EXTI_Initstructure.EXTI_Line=EXTI_Line5;								//�ⲿ�ж���·
	EXTI_Initstructure.EXTI_Mode=EXTI_Mode_Interrupt;				//�ж�ģʽ
	EXTI_Initstructure.EXTI_Trigger=EXTI_Trigger_Falling;		//������ʽ-
	EXTI_Initstructure.EXTI_LineCmd=ENABLE;									//����ʹ��
	EXTI_Init(&EXTI_Initstructure);
	EXTI_GenerateSWInterrupt(EXTI_Line5);										//ʹ���ж�

}
/*******************************************************************************
* ������		:	
* ��������	:	�¼��ж� 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void EXTI_Event_Configuration(void)
{

}

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void EXTI_Server(void)
{
	WOW_Server();				//������
	EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_ClearITPendingBit(EXTI_Line1);
	EXTI_ClearITPendingBit(EXTI_Line2);
	EXTI_ClearITPendingBit(EXTI_Line3);
	EXTI_ClearITPendingBit(EXTI_Line4);
	EXTI_ClearITPendingBit(EXTI_Line5);
	EXTI_ClearITPendingBit(EXTI_Line6);
	EXTI_ClearITPendingBit(EXTI_Line7);
	EXTI_ClearITPendingBit(EXTI_Line8);
	EXTI_ClearITPendingBit(EXTI_Line9);
	EXTI_ClearITPendingBit(EXTI_Line10);
	EXTI_ClearITPendingBit(EXTI_Line11);
	EXTI_ClearITPendingBit(EXTI_Line12);
	EXTI_ClearITPendingBit(EXTI_Line13);
	EXTI_ClearITPendingBit(EXTI_Line14);
	EXTI_ClearITPendingBit(EXTI_Line15);
}

