#ifndef __USART_ISP_H
#define __USART_ISP_H

#include "stm32f10x_type.h"
#include "stm32f10x_usart.h"
//#include "stm32f10x_gpio.h"
//#include "stm32f10x_spi.h"

//#include	"stdio.h"				//����printf
//#include	"string.h"			//����printf
//#include	"stdarg.h"			//���ڻ�ȡ��ȷ�������Ĳ���
//#include	"stdlib.h"			//malloc��̬�����ڴ�ռ�
//	
//#include	"stddef.h"
//#include	"stdint.h"

//#include "STM32F10x_BitBand.H"

//#include "STM32_SYSTICK.H"
//#include "STM32_SPI.H"

/*-------------------------ͨ�Ű�ȫ--------------------
��̹��� (PC) ������������ͨ�ž�ͨ�����·�ʽ��֤��
1. У��ͣ����յ��������ֽڿ����������㡣ÿ��ͨ�Ž�β����һ���ֽڣ�У����ֽڣ���
	����ǰ�������ֽ��������Ľ��������������н��յ����ֽڣ������ݰ�����У����ֽڣ��������Ϊ 0x00
2. ���ÿ������������ᷢ��һ���ֽڼ��䲹�루����� = 0x00��
3. UART��������żУ�飨żУ�飩
ÿ�����ݰ����߱����ܣ�ACK Ӧ�𣩻��߱�������NACK Ӧ�𣩣�
--------------------------------------------------------------------------------------*/
#define	ISP_BufferSize	256
	
typedef	enum _ISP_COMMAND			//USART �Ծٳ�������
{
	ISP_COMMAND_Get		=0x00,		//Get:��ȡ��ǰ�Ծٳ���汾������ʹ�õ�����
	ISP_COMMAND_GetVS	=0x01,		//Get Version & Read Protection Status:��ȡ�Ծٳ���汾�� Flash �Ķ�����״̬
	ISP_COMMAND_GetID	=0x02,		//Get ID:��ȡоƬ ID
	ISP_COMMAND_RM		=0x11,		//Read Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ��ȡ��� 256 ���ֽڵĴ洢���ռ�
	ISP_COMMAND_Go		=0x21,		//Go:��ת���ڲ� Flash �� SRAM �ڵ�Ӧ�ó������
	ISP_COMMAND_WM		=0x31,		//Write Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ����� 256 ���ֽڵ�����д�� RAM �� Flash
	ISP_COMMAND_Erase	=0x43,		//Erase:����һ����ȫ�� Flash ҳ��
	ISP_COMMAND_EE		=0x44,		//Extended Erase:ʹ��˫�ֽ�Ѱַģʽ����һ����ȫ�� Flash ҳ�棨������v3.0 usart �Ծٳ���汾�����ϰ汾����
	ISP_COMMAND_WP		=0x63,		//Write Protect:ʹ��ĳЩ������д����
	ISP_COMMAND_WU		=0x73,		//Write Unprotect:��ֹ���� Flash ������д����
	ISP_COMMAND_RP		=0x82,		//Readout Protect:ʹ�ܶ�����
	ISP_COMMAND_RU		=0x92			//Readout Unprotect:��ֹ������
	//˵����
	//1)������յ��ܾ��������ִ�������ڼ���ִ����Ծٳ�����ᷢ�� NACK �ֽڲ����ؼ������״̬��
	//2)������ - ���� RDP����������ѡ���ֻ��ʹ����һ���޵������Ӽ�������������յ� NACK Ӧ�𣬲��Ҳ�������������á�ȡ�� RDP ���ɼ����������
	//3)Erase (x043) �� Extended Erase (0x44) ��Ϊ��ռ���һ��������֧�� Erase ����� Extended Erase ���������ͬʱ֧�����������
}ISP_COMMAND_TypeDef;

typedef	enum _ISP_Answer	//USART Ӧ��
{
	ISP_ANSWER_ACK		=0x79,		//ACK Ӧ��(�����ܣ�
	ISP_ANSWER_NACK		=0x1F			//NACK Ӧ�𣨲����ܻ��߲�ʶ��--����)
}ISP_ANSWER_TypeDef;
typedef	enum _ISP_FUN	//USART Ӧ��
{
	ISP_SLAVE			=0x00,			//ISP��Ϊ�ӻ�
	ISP_MASTER		=0x01				//ISP��Ϊ����--������������
}ISP_FUN_TypeDef;

//typedef	enum _ISP_STEPS			//ISPִ�в���
//{
//	Step_Start				=	0,			//��ʼ���裬��ǰΪ����
//	Step_Get					=	1,			//�ȴ�д������ɣ�ֻ��д������ɺ�ſ��Խ���д���߲�������
//	Step_GetVS				=	2,			//дʹ��
//	Step_GetID				=	3,			//д״̬��		��ȡ״̬Ϊ1��ʾ����ִ��д�����������ʾд�����
//	Step_RM						=	4,			//��
//	Step_WaitRaddr		=	5,			//�ȴ�������Ҫ��ȡ�ĵ�ַ
//	Step_WaitRlen			=	6,			//�ȴ�������Ҫ��ȡ�����ݳ���
//	Step_ReadData			=	7,			//�ȴ�������Ҫ��ȡ�����ݳ���
//	Step_SendRData		=	8,			//���Ͷ�ȡ������
//	Step_Erase				=	9,			//��������
//	Step_WaitWaddr		=	10,			//�ȴ�������Ҫд���������ʼ��ַ
//	Step_WriteData		=	11,			//����д������
//	Step_EraseAll			=	12,			//ȫ������
//	Step_ErasePage		=	13,			//����ҳ
//	Step_WaitErasePageA	=	14,		//WaitErasePageAҪ������ҳ��
//	Step_WaitWlen			=	15,			//�ȴ�������Ҫд������ݳ���
//	Step_WaitWData		=	16			//�ȴ�����Ҫд�������
//}ISP_STEPS_TypeDef;

typedef enum _ISP_SLAVE_STATUS			//ISP״̬��־---�����ڴӻ�״̬ʱ
{
	ISP_STATUS_IDLE=0,					//ISP����״̬�����Զ�д
	ISP_STATUS_WaitCommand,			//ISP�ȴ������Ϊ�ӻ�)
	ISP_STATUS_WaitEraseAddr,		//ISP�ȴ���������ַ
	ISP_STATUS_WaitErase,				//ISP�ȴ�����
	ISP_STATUS_Eraseing,				//ISP���ڲ���
	ISP_STATUS_WaitErased,			//ISP�ȴ��������
	ISP_STATUS_WaitWAddr,				//ISP�ȴ���д���ַ
	ISP_STATUS_WaitWData,				//ISP�ȴ����մ�д������
	ISP_STATUS_WaitWrite,				//ISP�ȴ�д����
	ISP_STATUS_WaitWrited,			//ISP�ȴ�д�����
	ISP_STATUS_WaitReadAddr,		//ISP�ȴ�����ȡ��ַ
	ISP_STATUS_WaitReadData,		//ISP�ȴ������ݲ���
	ISP_STATUS_WaitLengh,				//ISP�ȴ�����ȡ����
	ISP_STATUS_WaitSData,				//ISP�ȴ���������
	ISP_STATUS_WaitGoAddr				//�ȴ�����ת���ڲ� Flash �� SRAM �ڵ�Ӧ�ó�������ַ//Go �������ڴ�Ӧ�ó���ָ���ĵ�ַ��ʼִ�������صĴ���������κδ���
}ISP_SLAVE_STATUS_TypeDef;
typedef enum _ISP_MASTER_STATUS			//ISP״̬��־---����������״̬ʱ
{
	ISP_MSTATUS_IDLE					=	0,	//ISP����״̬�����Զ�д
	ISP_MSTATUS_WaitCommand		=	1,	//ISP�ȴ������Ϊ�ӻ�)
	ISP_MSTATUS_WaitEraseAddr	=	2,	//ISPִ��д����
	ISP_MSTATUS_WaitErased		=	3,	//ISPִ��д����
	ISP_MSTATUS_WaitWAddr			=	4,	//ISPִ�ж�����
	ISP_MSTATUS_WaitRAddr			=	5,	//ISPִ�ж�����
	ISP_MSTATUS_WaitLengh			=	6,	//ISP����ִ�в�������
	ISP_MSTATUS_WaitWrited		=	7,	//ISP����ִ�в�������
	ISP_MSTATUS_WaitReaded		=	8,	//ISP����ִ�в�������
	ISP_MSTATUS_WaitSended		=	9,	//ISP����ִ�в�������
	ISP_MSTATUS_WaitGoAddr		=	10	//��ת���ڲ� Flash �� SRAM �ڵ�Ӧ�ó������	
}ISP_MASTER_STATUS_TypeDef;

typedef	struct _ISP_Version		//USART �Ծٳ���汾
{
	unsigned char ISP_VS				;		//�Ծٳ���汾��0 < �汾 < 255����ʾ����0x10 = �汾 1.0
	unsigned char ISP_VS_Get		;		//Get:��ȡ��ǰ�Ծٳ���汾������ʹ�õ�����
	unsigned char ISP_VS_GetVS	;		//Get Version & Read Protection Status:��ȡ�Ծٳ���汾�� Flash �Ķ�����״̬
	unsigned char ISP_VS_GetID	;		//Get ID:��ȡоƬ ID
	unsigned char ISP_VS_RM			;		//Read Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ��ȡ��� 256 ���ֽڵĴ洢���ռ�
	unsigned char ISP_VS_Go			;		//Go:��ת���ڲ� Flash �� SRAM �ڵ�Ӧ�ó������
	unsigned char ISP_VS_WM			;		//Write Memory:��Ӧ�ó���ָ���ĵ�ַ��ʼ����� 256 ���ֽڵ�����д�� RAM �� Flash
	unsigned char ISP_VS_Erase	;		//Erase:����һ����ȫ�� Flash ҳ��
	unsigned char ISP_VS_WP			;		//Write Protect:ʹ��ĳЩ������д����
	unsigned char ISP_VS_WU			;		//Write Unprotect:��ֹ���� Flash ������д����
	unsigned char ISP_VS_RP			;		//Readout Protect:ʹ�ܶ�����
	unsigned char ISP_VS_RU			;		//Readout Unprotect:��ֹ������
}ISP_VERSION_TypeDef;

typedef	struct _ISP_DATA				//ISP���ݴ洢����
{
	unsigned int OffsetAddr		;		//д�ӻ�ʱ�ĵ�ַƫ��
	unsigned int StartAddr		;		//��ʼ��ַ
	unsigned int GoAddr				;		//��ʼ��ַ
	
	unsigned int ReceivedLen	;		//���ڽ��յ����ݸ���
	unsigned int SendLen			;		//��Ҫ�����ڷ��͵����ݸ���
//	unsigned int NumToSave		;		//���յ�����Ч����
//	unsigned int NumToSend		;		//��Ҫ�����ڷ��͵����ݸ���
	
	unsigned int ReadAddr			;		//��������ʼ��ַ
//	unsigned int ReadLen			;		//��Ҫ��ȡ�ĳ���
	
	unsigned int WriteAddr		;		//Ҫд��Flash��������ʼ��ַ
	unsigned int WriteLen			;		//Ҫд��Flash�����ݳ��ȣ��ֽ���)
	
	unsigned char Command[2]	;		//�Ծٳ���������У����

	unsigned char ISP_RxBuffer[ISP_BufferSize]	;		//���ջ�����
	unsigned char ISP_RvBuffer[ISP_BufferSize]	;		//���ջ�����--������
	unsigned char ISP_TxBuffer[ISP_BufferSize]	;		//���ͻ�����
	unsigned char ISP_TvBuffer[ISP_BufferSize]	;		//���ͻ�����--������

}ISP_DATA_TypeDef;

typedef	struct _ISP_Conf					//USART ���߱�������ļ�
{
	USART_TypeDef* 	USARTx						;		//USART1,USART2,USART3,UART4;//UART5
	
	ISP_SLAVE_STATUS_TypeDef	ISP_SLAVE_STATUS		;		//ISP״̬��־---�����ڴӻ�״̬ʱ
	ISP_MASTER_STATUS_TypeDef	ISP_MASTER_STATUS		;		//ISP״̬��־---����������״̬ʱ
	ISP_VERSION_TypeDef	SlaveVersion							;		//�ӻ��Ծٳ���汾��Ϣ
	ISP_DATA_TypeDef		ISP_DATA									;		//ISP���ݴ洢����
	ISP_FUN_TypeDef			ISP_FUN										;		//ISPģ�鹤��״̬������-��¼/�ӻ�-������������)
	unsigned int OverRunTime											;		//��ʱʱ��
	

}ISP_Conf_TypeDef;


void Usart_ISP_Cof(ISP_Conf_TypeDef *ISP_Conf);
void Usart_ISP_ConfigurationNR(ISP_Conf_TypeDef *ISP_Conf,u32 USART_BaudRate,u32 *RXDBuffer,u32 BufferSize);	//
void Usart_ISP_Process(ISP_Conf_TypeDef *ISP_Conf);

void Usart_ISP_SlaveProcess(ISP_Conf_TypeDef *ISP_Conf);		//ģ����Ϊ�ӻ�ʱ�Ĵ�������
void Usart_ISP_MasterProcess(ISP_Conf_TypeDef *ISP_Conf);		//ģ����Ϊ����ʱ�Ĵ�������
void Usart_ISP_CheckFun(ISP_Conf_TypeDef *ISP_Conf);				//���ISPģ�鹤��ģʽ---����ʱ���


void Usart_ISP_ACK(ISP_Conf_TypeDef *ISP_Conf);	//ISPӦ��
void Usart_ISP_NACK(ISP_Conf_TypeDef *ISP_Conf);		//ISP��Ӧ��
void Usart_ISP_CommandSend(ISP_Conf_TypeDef *ISP_Conf,unsigned char Command);	//���ڱ�̷����������
void Usart_ISP_CommandRead(ISP_Conf_TypeDef *ISP_Conf);			//���ڽ����������->�ӻ�)
void Usart_ISP_GetAddr(ISP_Conf_TypeDef *ISP_Conf);					//ISP��ȡд������ʼ��ַ(����->�ӻ�)
void Usart_ISP_SetAddr(ISP_Conf_TypeDef *ISP_Conf);					//ISP����д������ʼ��ַ(����->�ӻ�)
void Usart_ISP_GetLengh(ISP_Conf_TypeDef *ISP_Conf);				//ISP��ȡ��Ҫ��ȡ�����ݳ���(����->�ӻ�)��������ʱ�������ַ���ٴ������ȡ�����ݳ���
void Usart_ISP_SetLengh(ISP_Conf_TypeDef *ISP_Conf);				//ISP��ȡ��Ҫ��ȡ�����ݳ���(����->�ӻ�)
void Usart_ISP_ReadMemory(ISP_Conf_TypeDef *ISP_Conf);			//ISP�����ݣ��ӻ�->������
void Usart_ISP_WriteMemory(ISP_Conf_TypeDef *ISP_Conf);			//ISPд���ݣ�����->�ӻ���д����ʱ�������ַ��ȴ��������ݣ���ִ��д����
void Usart_ISP_SendBuffer(ISP_Conf_TypeDef *ISP_Conf);			//ISP�ϴ����ݣ��ӻ�->����)
void Usart_ISP_Erase(ISP_Conf_TypeDef *ISP_Conf);						//ISP�������������յ����������Ӧ�𣬵ȴ������������ַ��ȫ����������ҳ�������ȴ�������ɣ���ɺ�Ӧ��
u8 Usart_ISP_GetSlaveStatus(ISP_Conf_TypeDef *ISP_Conf);	//���شӻ�״ֵ̬
void Usart_ISP_SetSlaveStatus(ISP_Conf_TypeDef *ISP_Conf,ISP_SLAVE_STATUS_TypeDef Status);	//���ôӻ�״̬
void Usart_ISP_Reset(ISP_Conf_TypeDef *ISP_Conf);						//���ñ����---�ָ����в���ΪĬ��ֵ
	
#endif
