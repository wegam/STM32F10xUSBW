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

#include "USART_ISP.H"

#include "STM32_USART.H"
#include "STM32_TOOLS.H"

//#define USB_TEST_BD				//USB_TEST��
//#define SPI_FLASH_OSTL
//#define CMSIS_CDC_BD			//CMSIS-CDC��

#include "string.h"
//#include "stdlib.h"
//#include "stdio.h"

#include "stm32f10x_type.h"
//#include "stm32f10x_spi.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
//#include "stm32f10x_nvic.h"

//#include "STM32_WOW.H"
//#include "STM32_PWM.H"
//#include "STM32_SYS.H"
//#include "STM32_SPI.H"
//#include "STM32_GPIO.H"
//#include "STM32_SYSTICK.H"
//#include "STM32_USART.H"

//#define	 Usart_ISP_Simulation 		//ģ��ӻ�
/*******************************************************************************
* ������			:	function
* ��������		:	��������˵�� 
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Usart_ISP_Cof(ISP_Conf_TypeDef *ISP_Conf)
{
	USART_DMA_ConfigurationEV	(ISP_Conf->USARTx,115200,(u32*)ISP_Conf->ISP_DATA.ISP_RxBuffer,ISP_BufferSize);	//USART_DMA����--��ѯ��ʽ�������ж�
	Usart_ISP_Reset(ISP_Conf);																																								//���ñ����---�ָ����в���ΪĬ��ֵ
	ISP_Conf->ISP_FUN=ISP_SLAVE;			//����---��ģ������Ϊ�ӻ�
}
/*******************************************************************************
* ������			:	Usart_ISP_CommandSend
* ��������		:	���ڱ�̷����������
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Usart_ISP_Process(ISP_Conf_TypeDef *ISP_Conf)
{
	if(ISP_Conf->ISP_FUN==ISP_SLAVE)				//ģ����Ϊ�ӻ�----���³���
	{
		Usart_ISP_SlaveProcess(ISP_Conf);			//ģ����Ϊ�ӻ�ʱ�Ĵ�������
	}
	else if(ISP_Conf->ISP_FUN==ISP_MASTER)	//ģ����Ϊ����----��¼Ŀ���
	{
		Usart_ISP_SlaveProcess(ISP_Conf);			//ģ����Ϊ����ʱ�Ĵ�������
	}
	else
	{
		Usart_ISP_CheckFun(ISP_Conf);					//���ISPģ�鹤��ģʽ---����ʱ���
	}
}
/*******************************************************************************
* ������			:	Usart_ISP_SlaveProcess
* ��������		:	ģ����Ϊ�ӻ�ʱ�Ĵ�������
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Usart_ISP_SlaveProcess(ISP_Conf_TypeDef *ISP_Conf)		//ģ����Ϊ�ӻ�ʱ�Ĵ�������
{
	ISP_Conf->ISP_DATA.ReceivedLen=USART_ReadBufferIDLE(ISP_Conf->USARTx,(u32*)ISP_Conf->ISP_DATA.ISP_RvBuffer,(u32*)ISP_Conf->ISP_DATA.ISP_RxBuffer);	//���ڿ���ģʽ�����ڽ��ջ���������������ݣ������ݿ�����RevBuffer,�����ؽ��յ������ݸ�����Ȼ�����½����ջ�������ַָ��RxdBuffer
	if(ISP_Conf->ISP_DATA.ReceivedLen)
	{
		ISP_Conf->OverRunTime=0;									//��ʱʱ��
		if(ISP_Conf->ISP_DATA.ReceivedLen==1)			//���ֽ�----ʶ������
		{
			if(ISP_Conf->ISP_DATA.ISP_RvBuffer[0]==0x7F)	//�Ծ�Ӧ��
			{
				Usart_ISP_Reset(ISP_Conf);														//���ñ����---�ָ����в���ΪĬ��ֵ
				Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitCommand);		//ISP�ȴ������Ϊ�ӻ�)
				Usart_ISP_ACK(ISP_Conf);															//ISPӦ��
			}
		}
		else if(ISP_Conf->ISP_DATA.ReceivedLen==2&&ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitCommand)				//2�ֽ�---�����������
		{
			Usart_ISP_CommandRead(ISP_Conf);			//���ڽ����������->�ӻ�)
		}
		else if((ISP_Conf->ISP_DATA.ReceivedLen==5&&(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitReadAddr||ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitWAddr||ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitGoAddr))||ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitEraseAddr)				//5���ֽ�---��ַ�����ݽ��յ�ַ����
		{
			Usart_ISP_GetAddr(ISP_Conf);						//ISP��ȡд������ʼ��ַ(����->�ӻ�)
		}
		else if(ISP_Conf->ISP_DATA.ReceivedLen==2&&ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitLengh)				//2�ֽ�---���մ��������ݳ��ȣ�������ʱ�������ַ���ٴ������ȡ�����ݳ���
		{
			Usart_ISP_GetLengh(ISP_Conf);					//ISP��ȡ��Ҫ��ȡ�����ݳ���(����->�ӻ�)��������ʱ�������ַ���ٴ������ȡ�����ݳ���
		}
		else if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitWData)	//ISP�ȴ����մ�д������,д����ʱ�������ַ��ȴ��������ݣ���ִ��д����
		{
			Usart_ISP_WriteMemory(ISP_Conf);			//ISPд���ݣ�����->�ӻ���д����ʱ�������ַ��ȴ��������ݣ���ִ��д����
		}
				
	}
	else if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitReadData)	//�ȴ����ݶ�ȡ���
	{
		Usart_ISP_ReadMemory(ISP_Conf);															//ISP�����ݣ��ӻ�->������
	}	
	else if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitSData)			//ISP�ȴ���������
	{
		Usart_ISP_SendBuffer(ISP_Conf);															//ISP�ϴ����ݣ��ӻ�->����)
	}
	else if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitErase||ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitErased)		//ISPISP�ȴ�����
	{
		Usart_ISP_Erase(ISP_Conf);						//ISP�������������յ����������Ӧ�𣬵ȴ������������ַ��ȫ����������ҳ�������ȴ�������ɣ���ɺ�Ӧ��
	}
	else if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitWrited)	//ISP�ȴ����մ�д������,д����ʱ�������ַ��ȴ��������ݣ���ִ��д����
	{
		Usart_ISP_WriteMemory(ISP_Conf);			//ISPд���ݣ�����->�ӻ���д����ʱ�������ַ��ȴ��������ݣ���ִ��д����
	}
	else	if(ISP_Conf->ISP_SLAVE_STATUS!=ISP_STATUS_IDLE)				//�ǳ�ʼ״̬ʱ��ʱ��λ
	{
		ISP_Conf->OverRunTime=ISP_Conf->OverRunTime+1;					//��ʱʱ��
		if(ISP_Conf->OverRunTime>=5000000)		//Լ5��
		Usart_ISP_Reset(ISP_Conf);				//���ñ����---�ָ����в���ΪĬ��ֵ
	}
//	ISP_Conf->ISP_DATA.NumHaveRead=0;	//���յ����ݸ���
}
/*******************************************************************************
* ������			:	Usart_ISP_MasterProcess
* ��������		:	ģ����Ϊ����ʱ�Ĵ�������
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Usart_ISP_MasterProcess(ISP_Conf_TypeDef *ISP_Conf)		//ģ����Ϊ����ʱ�Ĵ�������
{
}
/*******************************************************************************
* ������			:	Usart_ISP_CheckFun
* ��������		:	���ISPģ�鹤��ģʽ---����ʱ���
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Usart_ISP_CheckFun(ISP_Conf_TypeDef *ISP_Conf)				//���ISPģ�鹤��ģʽ---����ʱ���
{
	Usart_ISP_Reset(ISP_Conf);	//���ñ����---�ָ����в���ΪĬ��ֵ---ģʽ�л���λ
}
/*******************************************************************************
* ������			:	Usart_ISP_ACK
* ��������		:	ISPӦ��
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Usart_ISP_ACK(ISP_Conf_TypeDef *ISP_Conf)	//ISPӦ��
{
	ISP_Conf->ISP_DATA.Command[0]=ISP_ANSWER_ACK;
	USART_DMASend(ISP_Conf->USARTx,(u32*)ISP_Conf->ISP_DATA.Command,1);	//����DMA���ͳ���
}
/*******************************************************************************
* ������			:	Usart_ISP_NACK
* ��������		:	ISP��Ӧ��
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Usart_ISP_NACK(ISP_Conf_TypeDef *ISP_Conf)		//ISP��Ӧ��
{
	ISP_Conf->ISP_DATA.Command[0]=ISP_ANSWER_NACK;
	USART_DMASend(ISP_Conf->USARTx,(u32*)ISP_Conf->ISP_DATA.Command,1);	//����DMA���ͳ���
}
/*******************************************************************************
* ������			:	Usart_ISP_CommandSend
* ��������		:	���ڱ�̷����������
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Usart_ISP_CommandSend(ISP_Conf_TypeDef *ISP_Conf,unsigned char Command)	//���ڱ�̷����������
{
	ISP_Conf->ISP_DATA.Command[0]=Command;
	ISP_Conf->ISP_DATA.Command[1]=Command^0XFF;
	USART_DMASend(ISP_Conf->USARTx,(u32*)ISP_Conf->ISP_DATA.Command,2);	//����DMA���ͳ���
}
/*******************************************************************************
* ������			:	Usart_ISP_CommandSend
* ��������		:	���ڱ�̷����������
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Usart_ISP_CommandRead(ISP_Conf_TypeDef *ISP_Conf)			//���ڽ����������->�ӻ�)
{
//	unsigned short rxNum=0;
	unsigned char C0=ISP_Conf->ISP_DATA.ISP_RvBuffer[0];
	unsigned char C1=ISP_Conf->ISP_DATA.ISP_RvBuffer[1];
	C1=C1^0XFF;
	if(C0!=C1)
	{
		return;
	}
	if(C0==ISP_COMMAND_Get)			//��ȡ��ǰ�Ծٳ���汾������ʹ�õ�����
	{
		ISP_Conf->ISP_DATA.ISP_TvBuffer[0]=ISP_ANSWER_ACK;			//Ӧ��
		ISP_Conf->ISP_DATA.ISP_TvBuffer[1]=0x0B;								//����
		ISP_Conf->ISP_DATA.ISP_TvBuffer[2]=0x22;								//�汾
		ISP_Conf->ISP_DATA.ISP_TvBuffer[3]=ISP_COMMAND_Get;		//Get ����
		ISP_Conf->ISP_DATA.ISP_TvBuffer[4]=ISP_COMMAND_GetVS;	//Get Version and Read Protection Status
		ISP_Conf->ISP_DATA.ISP_TvBuffer[5]=ISP_COMMAND_GetID;	//Get ID
		ISP_Conf->ISP_DATA.ISP_TvBuffer[6]=ISP_COMMAND_RM;			//Read Memory ����
		ISP_Conf->ISP_DATA.ISP_TvBuffer[7]=ISP_COMMAND_Go;			//Go ����
		ISP_Conf->ISP_DATA.ISP_TvBuffer[8]=ISP_COMMAND_WM;			//Write Memory ����
		ISP_Conf->ISP_DATA.ISP_TvBuffer[9]=ISP_COMMAND_Erase;	//Erase ����
		ISP_Conf->ISP_DATA.ISP_TvBuffer[10]=ISP_COMMAND_WP;		//Write Protect ����
		ISP_Conf->ISP_DATA.ISP_TvBuffer[11]=ISP_COMMAND_WU;		//Write Unprotect ����
		ISP_Conf->ISP_DATA.ISP_TvBuffer[12]=ISP_COMMAND_RP;		//Readout Protect ����
		ISP_Conf->ISP_DATA.ISP_TvBuffer[13]=ISP_COMMAND_RU;		//Readout Unprotect ����
		ISP_Conf->ISP_DATA.ISP_TvBuffer[14]=ISP_ANSWER_ACK;		//Ӧ��
		
		Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitCommand);		//ISP�ȴ������Ϊ�ӻ�)--������Get�����ȴ���һ������
		ISP_Conf->ISP_DATA.SendLen=15;												//������Ҫ���͵����ݳ���
		
		memcpy(ISP_Conf->ISP_DATA.ISP_TxBuffer, ISP_Conf->ISP_DATA.ISP_TvBuffer, ISP_Conf->ISP_DATA.SendLen);
		USART_DMASend(ISP_Conf->USARTx,(u32*)ISP_Conf->ISP_DATA.ISP_TxBuffer,ISP_Conf->ISP_DATA.SendLen);	//����DMA���ͳ���	
			
	}
	else if(C0==ISP_COMMAND_GetVS)	//��ȡ�Ծٳ���汾�� Flash �Ķ�����״̬
	{
		ISP_Conf->ISP_DATA.ISP_TvBuffer[0]=ISP_ANSWER_ACK;		//Ӧ��
		ISP_Conf->ISP_DATA.ISP_TvBuffer[1]=0x22;							//�Ծٳ���汾��0 < �汾 �� 255����ʾ����0x10 = �汾 1.0
		ISP_Conf->ISP_DATA.ISP_TvBuffer[2]=0x00;							//ѡ���ֽ� 1��0x00��������ͨ���Ծٳ���Э��ļ�����
		ISP_Conf->ISP_DATA.ISP_TvBuffer[3]=0x00;							//ѡ���ֽ� 2��0x00��������ͨ���Ծٳ���Э��ļ�����
		ISP_Conf->ISP_DATA.ISP_TvBuffer[4]=ISP_ANSWER_ACK;		//Ӧ��
		
		Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitCommand);		//ISP�ȴ������Ϊ�ӻ�)--������GetVS�����ȴ���һ������
		ISP_Conf->ISP_DATA.SendLen=5;												//������Ҫ���͵����ݳ���
		
		memcpy(ISP_Conf->ISP_DATA.ISP_TxBuffer, ISP_Conf->ISP_DATA.ISP_TvBuffer, ISP_Conf->ISP_DATA.SendLen);
		USART_DMASend(ISP_Conf->USARTx,(u32*)ISP_Conf->ISP_DATA.ISP_TxBuffer,ISP_Conf->ISP_DATA.SendLen);	//����DMA���ͳ���	
	}
	else if(C0==ISP_COMMAND_GetID)	//��ȡоƬ ID
	{
		ISP_Conf->ISP_DATA.ISP_TvBuffer[0]=ISP_ANSWER_ACK;		//Ӧ��
		ISP_Conf->ISP_DATA.ISP_TvBuffer[1]=0x01;							//N = �ֽ��� �C 1���� STM32 N = 1��������ǰ�ֽں� ACK ֮�⡣
		ISP_Conf->ISP_DATA.ISP_TvBuffer[2]=0x04;							//PID(1) �ֽ� 3 = 0x04���ֽ� 4 = 0xXX
		ISP_Conf->ISP_DATA.ISP_TvBuffer[3]=0x10;							//PID(1) �ֽ� 3 = 0x04���ֽ� 4 = 0xXX
		ISP_Conf->ISP_DATA.ISP_TvBuffer[4]=ISP_ANSWER_ACK;		//Ӧ��
		
		Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitCommand);		//ISP�ȴ������Ϊ�ӻ�)--������GetID�����ȴ���һ������
		ISP_Conf->ISP_DATA.SendLen=5;												//������Ҫ���͵����ݳ���
		
		memcpy(ISP_Conf->ISP_DATA.ISP_TxBuffer, ISP_Conf->ISP_DATA.ISP_TvBuffer, ISP_Conf->ISP_DATA.SendLen);
		USART_DMASend(ISP_Conf->USARTx,(u32*)ISP_Conf->ISP_DATA.ISP_TxBuffer,ISP_Conf->ISP_DATA.SendLen);	//����DMA���ͳ���	
	}
	else if(C0==ISP_COMMAND_RM)			//��Ӧ�ó���ָ���ĵ�ַ��ʼ��ȡ��� 256 ���ֽڵĴ洢���ռ䣺�����ݷ��岽��1-���������2-��������ʼ��ַ��3-��Ҫ��ȡ�ĳ��ȣ�4-�����ݹ��̣�5-�ϱ�����
	{
		Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitReadAddr);			//ISP�ȴ�����ȡ��ַ---�ӻ����յ������������Ӧ��Ȼ��ȴ�����ȡ���ݵ�ַ
		Usart_ISP_ACK(ISP_Conf);															//ISPӦ��
		return;
	}
	else if(C0==ISP_COMMAND_Go)			//�ȴ�����ת���ڲ� Flash �� SRAM �ڵ�Ӧ�ó�������ַ//Go �������ڴ�Ӧ�ó���ָ���ĵ�ַ��ʼִ�������صĴ���������κδ���
	{
		Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitGoAddr);			//���յ�Go�����Ӧ��Ȼ��ȴ���ʼִ����ʼ��ַ
		Usart_ISP_ACK(ISP_Conf);	//ISPӦ��
	}
	else if(C0==ISP_COMMAND_WM)			//Write Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ����� 256 ���ֽڵ�����д�� RAM �� Flash
	{
		Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitWAddr);			//ISP�ȴ���д���ַ---���յ�д�����Ӧ��Ȼ��ȴ�д����ʼ��ַ
		Usart_ISP_ACK(ISP_Conf);	//ISPӦ��
	}
	else if(C0==ISP_COMMAND_Erase)	//����һ����ȫ�� Flash ҳ��
	{
		Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitEraseAddr);	//ISP�ȴ���������ַ�����յ����������Ӧ�𣬵ȴ������������ַ��ȫ����������ҳ������
		Usart_ISP_ACK(ISP_Conf);	//ISPӦ��
	}
	else if(C0==ISP_COMMAND_EE)			//ʹ��˫�ֽ�Ѱַģʽ����һ����ȫ�� Flash ҳ�棨������v3.0 usart �Ծٳ���汾�����ϰ汾����
	{
		Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitEraseAddr);	//ISP�ȴ���������ַ�����յ����������Ӧ�𣬵ȴ������������ַ��ȫ����������ҳ������
		Usart_ISP_ACK(ISP_Conf);	//ISPӦ��
	}
	else if(C0==ISP_COMMAND_WP)			//ʹ��ĳЩ������д����
	{
		/*Write Protect ��������Ϊһ���ֻ����� Flash ����ʹ��д�������Ծٳ�����յ� Write Protect �����
			�Ὣ ACK �ֽڷ��͵����������� ACK �ֽں��Ծٳ��򽫵ȴ�Ҫ���յ� �ֽ������ܱ�������������֮���Ӧ�ó������ Flash �������롣
			�� Write Protect �������ʱ���Ծٳ���ᷢ�� ACK �ֽڲ�����ϵͳ��λ����ʵʩ�µ�ѡ�� �ֽ����á�
		*/
		return;			//�ݲ�����
	}
	else if(C0==ISP_COMMAND_WU)			//�ر����� Flash ������д����
	{
		/*Write Unprotect �������ڽ�ֹ���� Flash ������д�������Ծٳ�����յ� Write Unprotect �����
		�Ὣ ACK �ֽڷ��͵����������� ACK �ֽں��Ծٳ�����ֹ���� Flash ������ д������ִ�н�ֹ�����������Ծٳ����� ACK �ֽڡ�
		�� Write Unprotect �������ʱ���Ծٳ���ᷢ�� ACK �ֽڲ�����ϵͳ��λ����ʵʩ�µ�ѡ�� �ֽ����á�
		*/
		return;			//�ݲ�����
	}
	else if(C0==ISP_COMMAND_RP)			//ʹ�ܶ�����
	{
		/*Readout Protect ��������ʹ�� Flash ���������Ծٳ�����յ� Readout Protect �����
			�Ὣ ACK �ֽڷ��͵����������� ACK �ֽں��Ծٳ���ʹ�� Flash �Ķ�������
			�� Readout Protect �������ʱ���Ծٳ���ᷢ�� ACK �ֽڲ�����ϵͳ��λ����ʵʩ�µ�ѡ ���ֽ����á�
		*/
		return;			//�ݲ�����
	}
	else if(C0==ISP_COMMAND_RU)			//�رն�����
	{
		/*Readout Unprotect �������ڽ�ֹ Flash ���������Ծٳ�����յ� Readout Unprotect ���� �󣬻Ὣ ACK �ֽڷ��͵�������
			������ ACK �ֽں��Ծٳ��򽫲������� Flash �������� ֹ���� Flash �Ķ�������������������ɹ���ɣ��Ծٳ���ͣ�� RDP��
			�����������ʧ�ܣ��Ծٳ���ᷢ��һ�� NACK �Ҷ�������Ȼ��Ч��
			�� Readout Unprotect �������ʱ���Ծٳ���ᷢ�� ACK �ֽڲ�����ϵͳ��λ����ʵʩ�� ��ѡ���ֽ����á�
		*/
		return;			//�ݲ�����
	}
	else
	{
	}	
//	if(rxNum)
//	{
//		memcpy(ISP_Conf->ISP_DATA.ISP_TxBuffer, ISP_Conf->ISP_DATA.ISP_TvBuffer, rxNum);
//		USART_DMASend(ISP_Conf->USARTx,(u32*)ISP_Conf->ISP_DATA.ISP_TxBuffer,rxNum);	//����DMA���ͳ���	
//	}
}
/*******************************************************************************
* ������			:	Usart_ISP_ReadAddr
* ��������		:	ISP����ַ
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Usart_ISP_GetAddr(ISP_Conf_TypeDef *ISP_Conf)		//ISP��ȡд������ʼ��ַ(����->�ӻ�)
{	
	//ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitReadAddr||ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitWAddr||ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitEraseAddr)
	//��Ҫ���յ�ַ��ָ������ݣ�д���ݣ�����
	if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitEraseAddr)			//ISP�������������յ����������Ӧ�𣬵ȴ������������ַ��ȫ����������ҳ�������ȴ�������ɣ���ɺ�Ӧ��
	{
		unsigned char C0=ISP_Conf->ISP_DATA.ISP_RvBuffer[0];
		unsigned char C1=ISP_Conf->ISP_DATA.ISP_RvBuffer[1];
		C1=C1^0XFF;
		if(C0!=C1)
		{
			return;
		}
		ISP_Conf->ISP_DATA.WriteAddr=C0;													//��д���ݵ���ʼ��ַ--����Ϊд��0xFF
		Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitErase);						//�ȴ����մ�д�������
	}
	else
	{
		unsigned int addr=0x00;		//�ϲ���ַ����
		unsigned char Bcc=BCC8(ISP_Conf->ISP_DATA.ISP_RvBuffer,4);		//���У��;
		if(Bcc!=ISP_Conf->ISP_DATA.ISP_RvBuffer[4])
		{
			return;
		}
		else		//�ϲ���ַ---4�ֽ�
		{		
			addr=	(ISP_Conf->ISP_DATA.ISP_RvBuffer[0])<<24;
			addr+=(ISP_Conf->ISP_DATA.ISP_RvBuffer[1])<<16;
			addr+=(ISP_Conf->ISP_DATA.ISP_RvBuffer[2])<<8;
			addr+=(ISP_Conf->ISP_DATA.ISP_RvBuffer[3]);
		}
		if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitReadAddr)				//�ȴ����ն��������ַ�����յ���ַ��Ӧ���ٵȴ�����ȡ���ݳ���
		{
			ISP_Conf->ISP_DATA.ReadAddr=addr;														//�������ݵ���ʼ��ַ
			Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitLengh);						//�ȴ�������Ҫ��ȡ�����ݳ���
			Usart_ISP_ACK(ISP_Conf);	//ISPӦ��
		}
		else if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitWAddr)			//�ȴ�����д���ݵ�ַ�����յ���ַ��Ӧ���ٵȴ���д�������
		{
			ISP_Conf->ISP_DATA.WriteAddr=addr;													//��д���ݵ���ʼ��ַ
			Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitWData);		//���ôӻ�״̬--�ȴ����մ�д�������
//			ISP_Conf->ISP_SLAVE_STATUS=ISP_STATUS_WaitWData;						//�ȴ����մ�д�������
			Usart_ISP_ACK(ISP_Conf);	//ISPӦ��
		}
		else if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitGoAddr)			//�ȴ�����д���ݵ�ַ�����յ���ַ��Ӧ���ٵȴ���д�������
		{
			ISP_Conf->ISP_DATA.GoAddr=addr;																//��д���ݵ���ʼ��ַ
			Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_IDLE);						//ISP����״̬�����Զ�д
			Usart_ISP_ACK(ISP_Conf);	//ISPӦ��
		}
	}
}
/*******************************************************************************
*������			:	function
*��������		:	��������˵��
*����				: 
*����ֵ			:	��
*******************************************************************************/
void Usart_ISP_SetAddr(ISP_Conf_TypeDef *ISP_Conf)					//ISP����д������ʼ��ַ(����->�ӻ�)
{
	#ifdef	Usart_ISP_Simulation 		//ģ��ӻ�
	{
	}
	#else
	{
	}
	#endif
}
/*******************************************************************************
*������			:	function
*��������		:	��������˵��
*����				: 
*����ֵ			:	��
*******************************************************************************/
void Usart_ISP_GetLengh(ISP_Conf_TypeDef *ISP_Conf)				//ISP��ȡ��Ҫ��ȡ�����ݳ���(����->�ӻ�)��������ʱ�������ַ���ٴ������ȡ�����ݳ���
{	
	//ISP��ȡ��Ҫ��ȡ�����ݳ���(����->�ӻ�)��������ʱ�������ַ���ٴ������ȡ�����ݳ��ȣ����յ��������ݺ�ִ�ж����ݲ�����Ȼ���ϴ�����
	unsigned char C0=ISP_Conf->ISP_DATA.ISP_RvBuffer[0];
	unsigned char C1=ISP_Conf->ISP_DATA.ISP_RvBuffer[1];
	C1=C1^0XFF;
	if(C0!=C1)		//У���ַ
	{
		return;
	}
	else
	{
		ISP_Conf->ISP_DATA.SendLen=C0;										//��Ҫ�����ڷ��͵����ݸ���
		Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitReadData);	//���ôӻ�״̬--ISP�ȴ������ݲ���
	}
//		ISP_Conf->ISP_SLAVE_STATUS=ISP_STATUS_WaitReadData;	//ISP�ȴ������ݲ���
}
/*******************************************************************************
*������			:	function
*��������		:	��������˵��
*����				: 
*����ֵ			:	��
*******************************************************************************/
void Usart_ISP_SetLengh(ISP_Conf_TypeDef *ISP_Conf)				//ISP��ȡ��Ҫ��ȡ�����ݳ���(����->�ӻ�)
{
	#ifdef	Usart_ISP_Simulation 		//ģ��ӻ�
	{
	}
	#else
	{
	}
	#endif
}

/*******************************************************************************
* ������			:	Usart_ISP_CommandSend
* ��������		:	���ڱ�̷���������򣻶����ݷ��岽��1-���������2-��������ʼ��ַ��3-��Ҫ��ȡ�ĳ��ȣ�4-�����ݹ��̣�5-�ϱ�����
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Usart_ISP_ReadMemory(ISP_Conf_TypeDef *ISP_Conf)		//ISP�����ݣ��ӻ�->������
{	
	#ifdef	Usart_ISP_Simulation 		//ģ��ӻ�
	{
		//*******************ִ�ж����ݲ���
//		ISP_Conf->ISP_DATA.ISP_TxBuffer[0]=0x79;
//		memcpy(&ISP_Conf->ISP_DATA.ISP_TxBuffer[1], ISP_Conf->ISP_DATA.ISP_TvBuffer, ISP_Conf->ISP_DATA.SendLen+1);	//��������
		Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitSData);		//ISP�ȴ���������			
	}
	#else
	{
		//*******************ִ�ж����ݲ���
		//----��Ϊ�ȴ��ⲿFlash��ȡ������ɣ������ݶ�ȡ��ɺ����ⲿ��������ISP_STATUS_WaitSData״̬
		//1���ⲿ��⵽ISP_STATUS_WaitReadData״̬��
		//2�����ݵ�ַReadAddr�ʹ����ͳ���SendLen��ȡ��Ӧ����
		//3����ȡ��ɺ󣬽����ݸ��Ƶ�ISP_Conf->ISP_DATA.ISP_TvBuffer���ͱ�����
		//4������ISP_STATUS_WaitSData״̬����ʾ����׼����ɣ��ȴ�����
	}
	#endif
}
/*******************************************************************************
* ������			:	Usart_ISP_CommandSend
* ��������		:	���ڱ�̷����������
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Usart_ISP_WriteMemory(ISP_Conf_TypeDef *ISP_Conf)	//ISPд���ݣ�����->�ӻ���д����ʱ�������ַ��ȴ��������ݣ���ִ��д����
{	
	#ifdef	Usart_ISP_Simulation 		//ģ��ӻ�
	{
		if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitWData)		//ISP�ȴ����մ�д������
		{
			unsigned char	Wlen=ISP_Conf->ISP_DATA.ISP_RvBuffer[0];						//Ҫд��Flash�����ݳ��ȣ��ֽ���)
			unsigned char Bcc=BCC8(ISP_Conf->ISP_DATA.ISP_RvBuffer,Wlen+2);		//���У��;					
			if(Bcc!=ISP_Conf->ISP_DATA.ISP_RvBuffer[Wlen+2])
			{
				return;
			}
			ISP_Conf->ISP_DATA.WriteLen=ISP_Conf->ISP_DATA.ISP_RvBuffer[0];		//Ҫд��Flash�����ݳ��ȣ��ֽ���)	
			memcpy(ISP_Conf->ISP_DATA.ISP_TvBuffer, &ISP_Conf->ISP_DATA.ISP_RvBuffer[1], ISP_Conf->ISP_DATA.WriteLen+1);	//��������
			Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitWrited);				//ISP�ȴ�д�����
		}
		else if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitWrited)	//ISP�ȴ�д�����
		{
			Usart_ISP_ACK(ISP_Conf);	//ISPӦ��
			Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitCommand);				//ISP�ȴ������Ϊ�ӻ�)
		}
	}
	#else
	{
		if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitWData)		//ISP�ȴ����մ�д������
		{
			unsigned char	Wlen=ISP_Conf->ISP_DATA.ISP_RvBuffer[0];						//Ҫд��Flash�����ݳ��ȣ��ֽ���)
			unsigned char Bcc=BCC8(ISP_Conf->ISP_DATA.ISP_RvBuffer,Wlen+2);		//���У��;					
			if(Bcc!=ISP_Conf->ISP_DATA.ISP_RvBuffer[Wlen+2])
			{
				return;
			}
			ISP_Conf->ISP_DATA.WriteLen=ISP_Conf->ISP_DATA.ISP_RvBuffer[0];		//Ҫд��Flash�����ݳ��ȣ��ֽ���)	
			//���ⲿ��⵽ISP_STATUS_WaitWrite״̬ʱ������ISP_Conf->ISP_DATA.WriteLen������Flashд�����ݣ�д����ɵ�����ISP_STATUS_WaitWrited״̬
			memcpy(ISP_Conf->ISP_DATA.ISP_RvBuffer, &ISP_Conf->ISP_DATA.ISP_RvBuffer[1], ISP_Conf->ISP_DATA.WriteLen+1);	//��������
			Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitWrite);				//ISP�ȴ�д����
		}
		else if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitWrited)	//ISP�ȴ�д�����
		{
			//���ⲿ��⵽ISP_STATUS_WaitWrite״̬ʱ������ISP_Conf->ISP_DATA.WriteLen������Flashд�����ݣ�д����ɵ�����ISP_STATUS_WaitWrited״̬
			Usart_ISP_ACK(ISP_Conf);	//ISPӦ��
			Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitCommand);				//ISP�ȴ������Ϊ�ӻ�)
		}
	}
	#endif
//	unsigned short Wlen=ISP_Conf->ISP_RvBuffer[0];	//��ȡҪд������ݳ��ȣ��ֽ���)
//	unsigned char Bcc=BCC8(ISP_Conf->ISP_RvBuffer,Wlen+2);		//���У��;
//	if(Bcc!=ISP_Conf->ISP_RvBuffer[Wlen+2])
//	{
//		return;
//	}
//	ISP_Conf->ISP_STATUS=ISP_STATUS_WRITE;		//FLASHִ��д����
//	memcpy(ISP_Conf->ISP_TvBuffer, &ISP_Conf->ISP_RvBuffer[1], Wlen+1);
//	Usart_ISP_ACK(ISP_Conf);						//ISPӦ��
}
/*******************************************************************************
* ������			:	Usart_ISP_CommandSend
* ��������		:	���ڱ�̷����������
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Usart_ISP_SendBuffer(ISP_Conf_TypeDef *ISP_Conf)	//ISP�ϴ����ݣ��ӻ�->����)
{	

	ISP_Conf->ISP_DATA.ISP_TxBuffer[0]=0x79;
	memcpy(&ISP_Conf->ISP_DATA.ISP_TxBuffer[1], ISP_Conf->ISP_DATA.ISP_TvBuffer, ISP_Conf->ISP_DATA.SendLen+1);	//��������
	USART_DMASend(ISP_Conf->USARTx,(u32*)ISP_Conf->ISP_DATA.ISP_TxBuffer,ISP_Conf->ISP_DATA.SendLen+2);				//����DMA���ͳ���
	Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitCommand);				//�Ѿ�������ݷ������״̬����Ϊ�ȴ���һ������ISP_STATUS_WaitCommand

	
//	unsigned short Wlen=ISP_Conf->ISP_RvBuffer[0];	//��ȡҪд������ݳ��ȣ��ֽ���)
//	unsigned char Bcc=BCC8(ISP_Conf->ISP_RvBuffer,Wlen+2);		//���У��;
//	if(Bcc!=ISP_Conf->ISP_RvBuffer[Wlen+2])
//	{
//		return;
//	}
//	ISP_Conf->ISP_STATUS=ISP_STATUS_WRITE;		//FLASHִ��д����
//	memcpy(ISP_Conf->ISP_TvBuffer, &ISP_Conf->ISP_RvBuffer[1], Wlen+1);
//	Usart_ISP_ACK(ISP_Conf);						//ISPӦ��
}
/*******************************************************************************
* ������			:	Usart_ISP_CheckFun
* ��������		:	���ISPģ�鹤��ģʽ---����ʱ���
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Usart_ISP_Erase(ISP_Conf_TypeDef *ISP_Conf)				//ISP�������������յ����������Ӧ�𣬵ȴ������������ַ��ȫ����������ҳ�������ȴ�������ɣ���ɺ�Ӧ��
{
	//ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitErase||ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitErased
	#ifdef	Usart_ISP_Simulation 		//ģ��ӻ�
	{
		if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitErase)	//��ʼ����
		{
			if(ISP_Conf->ISP_DATA.WriteAddr==0xFF)	//ȫ������
			{
				Usart_ISP_ACK(ISP_Conf);													//ISPӦ��---�������
				Usart_ISP_Reset(ISP_Conf);												//���ñ����---�ָ����в���ΪĬ��ֵ---ģʽ�л���λ
				Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitErased);	//״̬Ϊ�������
			}
		}
		else if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitErased)	//�ȴ��������
		{
			Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitCommand);	//״̬Ϊ������ɣ�ISP�ȴ������Ϊ�ӻ�)
		}
	}
	#else
	{
		if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitErase)	//��ʼ����
		{
			if(ISP_Conf->ISP_DATA.WriteAddr==0xFF)	//ȫ������
			{
				Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_Eraseing);			//���ôӻ�״̬--ISP���ڲ���
			}
		}
		else if(ISP_Conf->ISP_SLAVE_STATUS==ISP_STATUS_WaitErased)			//�ȴ��������
		{
			Usart_ISP_ACK(ISP_Conf);							//ISPӦ��---�������
			Usart_ISP_SetSlaveStatus(ISP_Conf,ISP_STATUS_WaitCommand);	//״̬Ϊ������ɣ�ISP�ȴ������Ϊ�ӻ�)
		}
		
	}	
	#endif
}
/*******************************************************************************
* ������			:	Usart_ISP_Reset
* ��������		:	���ñ����---�ָ����в���ΪĬ��ֵ
* ����			: void
* ����ֵ			: void
*******************************************************************************/
u8 Usart_ISP_GetSlaveStatus(ISP_Conf_TypeDef *ISP_Conf)	//���شӻ�״ֵ̬
{	
	return(ISP_Conf->ISP_SLAVE_STATUS);		//���شӻ�״ֵ̬
}
/*******************************************************************************
* ������			:	Usart_ISP_Reset
* ��������		:	���ñ����---�ָ����в���ΪĬ��ֵ
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Usart_ISP_SetSlaveStatus(ISP_Conf_TypeDef *ISP_Conf,ISP_SLAVE_STATUS_TypeDef Status)	//���ôӻ�״̬
{	
	ISP_Conf->ISP_SLAVE_STATUS=Status;	//����״ֵ̬
}
/*******************************************************************************
* ������			:	Usart_ISP_Reset
* ��������		:	���ñ����---�ָ����в���ΪĬ��ֵ
* ����			: void
* ����ֵ			: void
*******************************************************************************/
void Usart_ISP_Reset(ISP_Conf_TypeDef *ISP_Conf)	//���ñ����---�ָ����в���ΪĬ��ֵ
{	
	ISP_Conf->ISP_SLAVE_STATUS			=	ISP_STATUS_IDLE;	//FLASH����״̬�����Զ�д
	ISP_Conf->OverRunTime						=	0;								//��ʱʱ��
	ISP_Conf->ISP_DATA.ReceivedLen	=	0;		//���ڽ��յ����ݸ���
	ISP_Conf->ISP_DATA.SendLen			=	0;		//��Ҫ�����ڷ��͵����ݸ���
	ISP_Conf->ISP_DATA.OffsetAddr		=	0;		//д�ӻ�ʱ�ĵ�ַƫ��
	ISP_Conf->ISP_DATA.StartAddr		=	0;		//��ʼ��ַ
	ISP_Conf->ISP_DATA.ReadAddr			=	0;		//��������ʼ��ַ
	ISP_Conf->ISP_DATA.WriteAddr		=	0;		//д��������ʼ��ַ
	ISP_Conf->ISP_DATA.WriteLen			=	0;		//��Ҫд��ĳ���
	memset(ISP_Conf->ISP_DATA.ISP_RxBuffer,0xFF, ISP_BufferSize);	//���ջ�����
	memset(ISP_Conf->ISP_DATA.ISP_RvBuffer,0xFF, ISP_BufferSize);	//���ջ�����--������
	memset(ISP_Conf->ISP_DATA.ISP_TxBuffer,0xFF, ISP_BufferSize);	//���ͻ�����
	memset(ISP_Conf->ISP_DATA.ISP_TvBuffer,0xFF, ISP_BufferSize);	//���ͻ�����--������
}



