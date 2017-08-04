/******************************** User_library *********************************
* 文件名 	: SPI_FLASH.C
* 作者   	: wegam@sina.com
* 版本   	: V
* 日期   	: 2017/04/16
* 说明   	: 
********************************************************************************
SPI_FLASH使用功能测试
1）需要宏定义 SPI_FLASH
2）使用USB_TEST 板测试
3）需要宏定义SPI引脚
--------------------------------FLASH部分术语解析-----------------------------
Sector：比BLOCK更小的单位
Block：	大部分的FLASH都以64K为单位空间ERASE
Chip：	整片FLASH
Page：
例：(256-byte per page)(4K-byte per sector/16page per sector)(64K-byte per block/16-sector per block)(8-block per chip)
*******************************************************************************/

#include "USART_ISP.H"

#include "STM32_USART.H"

//#define USB_TEST_BD				//USB_TEST板
//#define SPI_FLASH_OSTL
//#define CMSIS_CDC_BD			//CMSIS-CDC板

//#include "string.h"
//#include "stdlib.h"
//#include "stdio.h"

#include "stm32f10x_type.h"
//#include "stm32f10x_spi.h"
//#include "stm32f10x_gpio.h"
//#include "stm32f10x_rcc.h"
//#include "stm32f10x_dma.h"
//#include "stm32f10x_nvic.h"

//#include "STM32_WOW.H"
//#include "STM32_PWM.H"
//#include "STM32_SYS.H"
//#include "STM32_SPI.H"
//#include "STM32_GPIO.H"
//#include "STM32_SYSTICK.H"
//#include "STM32_USART.H"
/*******************************************************************************
* 函数名			:	function
* 功能描述		:	函数功能说明 
* 输入			: void
* 返回值			: void
*******************************************************************************/
void Usart_ISP_Cof(ISP_Conf_TypeDef *ISP_Conf)
{
	USART_DMA_ConfigurationNR	(ISP_Conf->USARTx,115200,(u32*)ISP_Conf->ISP_RxBuffer,ISP_BufferSize);	//USART_DMA配置--查询方式，不开中断
}
/*******************************************************************************
* 函数名			:	Usart_ISP_CommandSend
* 功能描述		:	串口编程发送命令程序
* 输入			: void
* 返回值			: void
*******************************************************************************/
void Usart_ISP_Process(ISP_Conf_TypeDef *ISP_Conf)
{
	unsigned short RxNum=0;
	RxNum=USART_ReadBufferIDLE(ISP_Conf->USARTx,(u32*)ISP_Conf->ISP_RvBuffer,(u32*)ISP_Conf->ISP_RxBuffer);	//串口空闲模式读串口接收缓冲区，如果有数据，将数据拷贝到RevBuffer,并返回接收到的数据个数，然后重新将接收缓冲区地址指向RxdBuffer
	if(RxNum)
	{
		if(ISP_Conf->ISP_RvBuffer[0]==0x7F)
		{
			ISP_Conf->Command[0]=ISP_ANSWER_ACK;
			USART_DMASend(ISP_Conf->USARTx,(u32*)ISP_Conf->Command,1);	//串口DMA发送程序
		}
		else if(RxNum==2)
		{
			Usart_ISP_CommandRead(ISP_Conf);			//串口接收命令
		}
	}
//	ISP_Conf->ISP_RvBuffer[0]=ISP_COMMAND_Get;
//	Usart_ISP_CommandRead(ISP_Conf);			//串口接收命令
	Usart_ISP_CommandSend(ISP_Conf,ISP_COMMAND_Get);	//串口编程发送命令程序
}
/*******************************************************************************
* 函数名			:	Usart_ISP_CommandSend
* 功能描述		:	串口编程发送命令程序
* 输入			: void
* 返回值			: void
*******************************************************************************/
void Usart_ISP_ACK(ISP_Conf_TypeDef *ISP_Conf)
{
	ISP_Conf->Command[0]=ISP_ANSWER_ACK;
	USART_DMASend(ISP_Conf->USARTx,(u32*)ISP_Conf->Command,1);	//串口DMA发送程序
}
/*******************************************************************************
* 函数名			:	Usart_ISP_CommandSend
* 功能描述		:	串口编程发送命令程序
* 输入			: void
* 返回值			: void
*******************************************************************************/
void Usart_ISP_NACK(ISP_Conf_TypeDef *ISP_Conf)
{
	ISP_Conf->Command[0]=ISP_ANSWER_NACK;
	USART_DMASend(ISP_Conf->USARTx,(u32*)ISP_Conf->Command,1);	//串口DMA发送程序
}
/*******************************************************************************
* 函数名			:	Usart_ISP_CommandSend
* 功能描述		:	串口编程发送命令程序
* 输入			: void
* 返回值			: void
*******************************************************************************/
void Usart_ISP_CommandSend(ISP_Conf_TypeDef *ISP_Conf,unsigned char Command)	//串口编程发送命令程序
{
	ISP_Conf->Command[0]=Command;
	ISP_Conf->Command[1]=Command^0XFF;
	USART_DMASend(ISP_Conf->USARTx,(u32*)ISP_Conf->Command,2);	//串口DMA发送程序
}
/*******************************************************************************
* 函数名			:	Usart_ISP_CommandSend
* 功能描述		:	串口编程发送命令程序
* 输入			: void
* 返回值			: void
*******************************************************************************/
void Usart_ISP_CommandRead(ISP_Conf_TypeDef *ISP_Conf)			//串口接收命令
{
	if(ISP_Conf->ISP_RvBuffer[0]==ISP_COMMAND_Get)
	{
		ISP_Conf->ISP_TxBuffer[0]=0x79;								//应答
		ISP_Conf->ISP_TxBuffer[1]=0x0B;								//长度
		ISP_Conf->ISP_TxBuffer[2]=0x22;								//版本
		ISP_Conf->ISP_TxBuffer[3]=ISP_COMMAND_Get;		//Get 命令
		ISP_Conf->ISP_TxBuffer[4]=ISP_COMMAND_GetVS;	//Get Version and Read Protection Status
		ISP_Conf->ISP_TxBuffer[5]=ISP_COMMAND_GetID;	//Get ID
		ISP_Conf->ISP_TxBuffer[6]=ISP_COMMAND_RM;			//Read Memory 命令
		ISP_Conf->ISP_TxBuffer[7]=ISP_COMMAND_Go;			//Go 命令
		ISP_Conf->ISP_TxBuffer[8]=ISP_COMMAND_WM;			//Write Memory 命令
		ISP_Conf->ISP_TxBuffer[9]=ISP_COMMAND_Erase;	//Erase 命令
		ISP_Conf->ISP_TxBuffer[10]=ISP_COMMAND_WP;		//Write Protect 命令
		ISP_Conf->ISP_TxBuffer[11]=ISP_COMMAND_WU;		//Write Unprotect 命令
		ISP_Conf->ISP_TxBuffer[12]=ISP_COMMAND_RP;		//Readout Protect 命令
		ISP_Conf->ISP_TxBuffer[13]=ISP_COMMAND_RU;		//Readout Unprotect 命令
		ISP_Conf->ISP_TxBuffer[14]=0x79;							//应答
		USART_DMASend(ISP_Conf->USARTx,(u32*)ISP_Conf->ISP_TxBuffer,15);	//串口DMA发送程序		
	}
	else if(ISP_Conf->ISP_RvBuffer[0]==0x7F)
	{
	}
	else if(ISP_Conf->ISP_RvBuffer[0]==0x7F)
	{
	}
	else if(ISP_Conf->ISP_RvBuffer[0]==0x7F)
	{
	}
	else if(ISP_Conf->ISP_RvBuffer[0]==0x7F)
	{
	}
	else if(ISP_Conf->ISP_RvBuffer[0]==0x7F)
	{
	}
	else if(ISP_Conf->ISP_RvBuffer[0]==0x7F)
	{
	}
}






