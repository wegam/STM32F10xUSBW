/******************************** User_library *********************************
* �ļ��� 	: SPI_FLASH.C
* ����   	: wegam@sina.com
* �汾   	: V
* ����   	: 2017/04/16
* ˵��   	: 
********************************************************************************
SPI_FLASHʹ�ù��ܲ���
1����Ҫ�궨�� SPI_FLASH
2��ʹ��USB_TEST �����
3����Ҫ�궨��SPI����
--------------------------------FLASH�����������-----------------------------
Sector����BLOCK��С�ĵ�λ
Block��	�󲿷ֵ�FLASH����64KΪ��λ�ռ�ERASE
Chip��	��ƬFLASH
Page��
����(256-byte per page)(4K-byte per sector/16page per sector)(64K-byte per block/16-sector per block)(8-block per chip)
*******************************************************************************/
#ifdef OFFLINE_ISP							//���������SPI_FLASH �˹�����Ч

#include "OFFLINE_ISP.H"

#include "SPI_FLASH.H"
#include "USART_ISP.H"

#include "STM32_PWM.H"
#include "STM32_SYS.H"
#include "STM32_GPIO.H"
#include "STM32_SYSTICK.H"
#include "STM32_USART.H"
#include "STM32_WDG.H"

#include "string.h"


#define 	USART_BufferSize				512

//ISP_Conf_TypeDef 	ISP_Conf;
//SPI_FLASH_TypeDef	SPI_FLASH;

u8 RxdBuffe[USART_BufferSize]={0};
u8 RevBuffe[USART_BufferSize]={0};
u16 PWM_Ratio=0;
u8 flag=0;

typedef struct
{
	ISP_Conf_TypeDef 	ISP_Conf;
	SPI_FLASH_TypeDef	SPI_FLASH;
	
}OFFLINE_ISP_TypeDef;

OFFLINE_ISP_TypeDef OFFLINE_Cof;

void OFFLINE_ISP_Conf(void);
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void OFFLINE_ISP_Configuration(void)
{
	SYS_Configuration();											//ϵͳ���� STM32_SYS.H	
	GPIO_DeInitAll();													//�����е�GPIO�ر�----V20170605
	PWM_OUT(TIM2,PWM_OUTChannel1,1,900);			//PWM�趨-20161127�汾
	
	OFFLINE_ISP_Conf();
	
	IWDG_Configuration(2000);					//�������Ź�����---������λms	
	SysTick_Configuration(10);				//ϵͳ���ʱ������72MHz,��λΪuS
}
/*******************************************************************************
* ������		:
* ��������	:
* ����		:
* ���		:
* ���� 		:
*******************************************************************************/
void OFFLINE_ISP_Server(void)
{
	u16 RxNum=0;
	IWDG_Feed();								//�������Ź�ι��	
//	RxNum=USART_ReadBufferIDLE(USART1,(u32*)RevBuffe,(u32*)RxdBuffe);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(RxNum)
	{
		Usart_ISP_Process(&(OFFLINE_Cof.ISP_Conf));
		SPI_FLASH_Process(&(OFFLINE_Cof.SPI_FLASH));			//FLASH���ݴ��������е�FLASH��������ӿ�
		
//		SPI_FLASH_BufferWrite	(&(OFFLINE_Cof.SPI_FLASH),	RevBuffe, OFFLINE_Cof.SPI_FLASH.SPI_FLASH_Info.SPI_FLASH_WriteAdrr, RxNum);	//FLASHд��������
//		SPI_FLASH_BufferRead	(&(OFFLINE_Cof.SPI_FLASH),	RevBuffe, OFFLINE_Cof.SPI_FLASH.SPI_FLASH_Info.SPI_FLASH_WriteAdrr, RxNum);
//		OFFLINE_Cof.SPI_FLASH.SPI_FLASH_Info.SPI_FLASH_WriteAdrr+=RxNum;
	}
	OFFLINE_ISP_StatusProcess();		//״̬����
	Usart_ISP_Process(&(OFFLINE_Cof.ISP_Conf));
	SPI_FLASH_Process(&(OFFLINE_Cof.SPI_FLASH));			//FLASH���ݴ��������е�FLASH��������ӿ�
	
//	PWM_Ratio+=1;
//	if(PWM_Ratio>=3600)
//	{
//		flag=1;
//		PWM_Ratio=0;
//	}
//	if(flag==0)
//	{
//		PWM_Ratio++;
//		if(PWM_Ratio>=3600)
//		{
//			flag=1;
////			PWM_Ratio=0;
//		}
//	}
//	else
//	{
//		PWM_Ratio--;
//		if(PWM_Ratio<=1800)
//		{
//			flag=0;
//		}
//	}
//	
//	SetPWM_Ratio(PWM_Ratio);		//����ռ�ձ�
	
}
/*******************************************************************************
*������			:	function
*��������		:	��������˵��
*����				: 
*����ֵ			:	��
*******************************************************************************/
void OFFLINE_ISP_StatusProcess(void)		//״̬����
{
	if(Usart_ISP_GetSlaveStatus(&(OFFLINE_Cof.ISP_Conf))==ISP_STATUS_WaitReadData)	//���شӻ�״ֵ̬
	{
//		memcpy((OFFLINE_Cof.ISP_Conf.ISP_DATA.ISP_TvBuffer), OFFLINE_Cof.ISP_Conf.ISP_DATA.ISP_RvBuffer, OFFLINE_Cof.ISP_Conf.ISP_DATA.WriteLen+1);	//��������
		Usart_ISP_SetSlaveStatus(&(OFFLINE_Cof.ISP_Conf),ISP_STATUS_WaitSData);	//���ôӻ�״̬
	}
	else if(Usart_ISP_GetSlaveStatus(&(OFFLINE_Cof.ISP_Conf))==ISP_STATUS_WaitWrite)	//ISP�ȴ�д����
	{
		memcpy((OFFLINE_Cof.ISP_Conf.ISP_DATA.ISP_TvBuffer), OFFLINE_Cof.ISP_Conf.ISP_DATA.ISP_RvBuffer, OFFLINE_Cof.ISP_Conf.ISP_DATA.WriteLen+1);	//��������
		Usart_ISP_SetSlaveStatus(&(OFFLINE_Cof.ISP_Conf),ISP_STATUS_WaitWrited);	//���ôӻ�״̬--ISP�ȴ�д�����
	}
	else if(Usart_ISP_GetSlaveStatus(&(OFFLINE_Cof.ISP_Conf))==ISP_STATUS_Eraseing)	//ISP���ڲ���---SPI_FLASHִ�в�������
	{
		Usart_ISP_SetSlaveStatus(&(OFFLINE_Cof.ISP_Conf),ISP_STATUS_WaitErased);			//���ôӻ�״̬--�ȴ��������
	}
}

/*******************************************************************************
* ������		:
* ��������	:
* ����		:
* ���		:
* ���� 		:
*******************************************************************************/
void OFFLINE_ISP_Conf(void)
{
	OFFLINE_Cof.ISP_Conf.USARTx=USART1;
	
	SPI_FLASH_Conf(&(OFFLINE_Cof.SPI_FLASH));
	Usart_ISP_Cof(&(OFFLINE_Cof.ISP_Conf));	
}




#endif