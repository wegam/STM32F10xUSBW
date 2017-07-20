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


#ifndef __STM32_SPI_H
#define __STM32_SPI_H

#include "stm32f10x_spi.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_nvic.h"
//#include "STM32_WOW.H"

#include	"stdio.h"			//����printf
#include	"string.h"			//����printf
#include	"stdarg.h"			//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"			//malloc��̬�����ڴ�ռ�
	
#include	"stddef.h"
#include	"stdint.h"


extern char *SPI_BUFFER;
extern u32	SPI_BUFFERSIZE;

typedef struct
{
	GPIO_TypeDef* GPIO_NSS;
  u16 SPI_NSSP;													//���÷���				��2��ȫ˫����2��ֻ���ա�һ�߷��͡�һ�߽��գ�
	GPIO_TypeDef* GPIO_SCK;
  u16 SPI_SCKP;													//ģʽ         	���ӻ����豸��
	GPIO_TypeDef* GPIO_MISO;
  u16 SPI_MISOP;												//����         	��8��16λ��
	GPIO_TypeDef* GPIO_MOSI;
  u16 SPI_MOSIP;												//ʱ�Ӽ���     	���ͻ�ߣ�
	
	GPIO_InitTypeDef 	GPIO_InitStructure;	//GPIO��ʼ���ṹ��	
	SPI_InitTypeDef  	SPI_InitStructure;	//SPI��ʼ���ṹ��
	DMA_InitTypeDef  	DMA_InitStructure;	//DMA��ʼ���ṹ��
	
	SPI_TypeDef* SPIx;										//SPI�ӿ�
	
	u8 SPI_DMA_EN;												//SPI_DMA_EN==0����ʹ��SPI_DMA���䣬����ʹ��	
	u8 *SPI_RX_Buffer;										//SPI��ȡ������
	u8 *SPI_TX_Buffer;										//SPI���ͻ�����
//	SPI_InitTypeDef SPI_Set;
	
}SPI_ConfTypeDef;



void SPI_BASIC_Configuration(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);					//SPI��������
void SPI_Server(void);
void SPI_DMA_Configuration(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct,u32* SPI_TX_Buffer,u32* SPI_RX_Buffer,u32 SPI_BUFFERSIZE);						//SPI_DMA����
void SPI_PinSet(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);											//SPI�ܽ����ü���ʼ��
int SPI_DMAPrintf(SPI_TypeDef* SPIx,const char *format,...);															//SPI_DMA���ͺ���----��ߵ�ʡ�Ժž��ǿɱ����
int SPI_DMA_BufferWrite(SPI_TypeDef* SPIx,u32 *SPI_TX_Buffer,u16 BufferSize);							//SPI_DMA���ͺ���----��ߵ�ʡ�Ժž��ǿɱ����
void SPI_DMA_BufferRead(u32 *ReadAddr);

 void SPI_DMA_ReceiveSendByte(SPI_TypeDef* SPIx,u16 num);				//DMA�շ�����
void SPI_DMA_Conf(SPI_TypeDef* SPIx,u16 num);										//����SPI_DMA�����û����С�����DMA��־
void SPI_DMA_Cmd(SPI_TypeDef* SPIx,FunctionalState NewState);		//SPI_DMAʹ�ܻ���ʧ��
unsigned char SPI_DMA_TX_Flag(SPI_TypeDef* SPIx);								//��ѯSPI_DMA������ɱ�־;0--δ��ɣ�1--���	

ErrorStatus SPI_TX_DMAFlagClear(SPI_TypeDef* SPIx);																				//���SPI_DMA��������жϱ�־

void SPI2_Conf_TEST(u32* SPI_TX_Buffer,u32* SPI_RX_Buffer,u32 SPI_BUFFERSIZE);
	
u8 SPI_SendByte(u8 byte);
#endif
