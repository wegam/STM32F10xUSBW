/******************************** User_library *********************************
* 文件名 	: STM32_SDCard.H
* 作者   	: wegam@sina.com
* 版本   	: V
* 日期   	: 2016/01/01
* 说明   	: 
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

#include	"stdio.h"			//用于printf
#include	"string.h"			//用于printf
#include	"stdarg.h"			//用于获取不确定个数的参数
#include	"stdlib.h"			//malloc动态申请内存空间
	
#include	"stddef.h"
#include	"stdint.h"


extern char *SPI_BUFFER;
extern u32	SPI_BUFFERSIZE;

typedef struct
{
	GPIO_TypeDef* GPIO_NSS;
  u16 SPI_NSSP;													//设置方向				（2线全双工、2线只接收、一线发送、一线接收）
	GPIO_TypeDef* GPIO_SCK;
  u16 SPI_SCKP;													//模式         	（从或主设备）
	GPIO_TypeDef* GPIO_MISO;
  u16 SPI_MISOP;												//宽度         	（8或16位）
	GPIO_TypeDef* GPIO_MOSI;
  u16 SPI_MOSIP;												//时钟极性     	（低或高）
	
	GPIO_InitTypeDef 	GPIO_InitStructure;	//GPIO初始化结构体	
	SPI_InitTypeDef  	SPI_InitStructure;	//SPI初始化结构体
	DMA_InitTypeDef  	DMA_InitStructure;	//DMA初始化结构体
	
	SPI_TypeDef* SPIx;										//SPI接口
	
	u8 SPI_DMA_EN;												//SPI_DMA_EN==0，不使用SPI_DMA传输，否则使用	
	u8 *SPI_RX_Buffer;										//SPI读取缓冲区
	u8 *SPI_TX_Buffer;										//SPI发送缓冲区
//	SPI_InitTypeDef SPI_Set;
	
}SPI_ConfTypeDef;



void SPI_BASIC_Configuration(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);					//SPI常规配置
void SPI_Server(void);
void SPI_DMA_Configuration(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct,u32* SPI_TX_Buffer,u32* SPI_RX_Buffer,u32 SPI_BUFFERSIZE);						//SPI_DMA配置
void SPI_PinSet(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct);											//SPI管脚配置及初始化
int SPI_DMAPrintf(SPI_TypeDef* SPIx,const char *format,...);															//SPI_DMA发送函数----后边的省略号就是可变参数
int SPI_DMA_BufferWrite(SPI_TypeDef* SPIx,u32 *SPI_TX_Buffer,u16 BufferSize);							//SPI_DMA发送函数----后边的省略号就是可变参数
void SPI_DMA_BufferRead(u32 *ReadAddr);

 void SPI_DMA_ReceiveSendByte(SPI_TypeDef* SPIx,u16 num);				//DMA收发数据
void SPI_DMA_Conf(SPI_TypeDef* SPIx,u16 num);										//设置SPI_DMA：设置缓存大小及清除DMA标志
void SPI_DMA_Cmd(SPI_TypeDef* SPIx,FunctionalState NewState);		//SPI_DMA使能或者失能
unsigned char SPI_DMA_TX_Flag(SPI_TypeDef* SPIx);								//查询SPI_DMA接收完成标志;0--未完成，1--完成	

ErrorStatus SPI_TX_DMAFlagClear(SPI_TypeDef* SPIx);																				//清除SPI_DMA发送完成中断标志

void SPI2_Conf_TEST(u32* SPI_TX_Buffer,u32* SPI_RX_Buffer,u32 SPI_BUFFERSIZE);
	
u8 SPI_SendByte(u8 byte);
#endif

