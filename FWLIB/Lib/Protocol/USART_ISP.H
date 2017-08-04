#ifndef __USART_ISP_H
#define __USART_ISP_H

#include "stm32f10x_type.h"
#include "stm32f10x_usart.h"
//#include "stm32f10x_gpio.h"
//#include "stm32f10x_spi.h"

//#include	"stdio.h"				//用于printf
//#include	"string.h"			//用于printf
//#include	"stdarg.h"			//用于获取不确定个数的参数
//#include	"stdlib.h"			//malloc动态申请内存空间
//	
//#include	"stddef.h"
//#include	"stdint.h"

//#include "STM32F10x_BitBand.H"

//#include "STM32_SYSTICK.H"
//#include "STM32_SPI.H"

/*-------------------------通信安全--------------------
编程工具 (PC) 到器件的所有通信均通过如下方式验证：
1. 校验和：接收到的数据字节块进行异或运算。每个通信结尾增加一个字节（校验和字节），
	包含前面所有字节异或运算的结果。异或运算所有接收到的字节，即数据包加上校验和字节，结果必须为 0x00
2. 针对每条命令，主机都会发送一个字节及其补码（异或结果 = 0x00）
3. UART：激活奇偶校验（偶校验）
每个数据包或者被接受（ACK 应答）或者被丢弃（NACK 应答）：
--------------------------------------------------------------------------------------*/
#define	ISP_BufferSize	256
	
typedef	enum _ISP_COMMAND_USART	//USART 自举程序命令
{
	ISP_COMMAND_Get		=0x00,		//Get:获取当前自举程序版本及允许使用的命令
	ISP_COMMAND_GetVS	=0x01,		//Get Version & Read Protection Status:获取自举程序版本及 Flash 的读保护状态
	ISP_COMMAND_GetID	=0x02,		//Get ID:获取芯片 ID
	ISP_COMMAND_RM		=0x11,		//Read Memory:从应用程序指定的地址开始读取最多 256 个字节的存储器空间
	ISP_COMMAND_Go		=0x21,		//Go:跳转到内部 Flash 或 SRAM 内的应用程序代码
	ISP_COMMAND_WM		=0x31,		//Write Memory:从应用程序指定的地址开始将最多 256 个字节的数据写入 RAM 或 Flash
	ISP_COMMAND_Erase	=0x43,		//Erase:擦除一个到全部 Flash 页面
	ISP_COMMAND_EE		=0x44,		//Extended Erase:使用双字节寻址模式擦除一个到全部 Flash 页面（仅用于v3.0 usart 自举程序版本及以上版本）。
	ISP_COMMAND_WP		=0x63,		//Write Protect:使能某些扇区的写保护
	ISP_COMMAND_WU		=0x73,		//Write Unprotect:禁止所有 Flash 扇区的写保护
	ISP_COMMAND_RP		=0x82,		//Readout Protect:使能读保护
	ISP_COMMAND_RU		=0x92			//Readout Unprotect:禁止读保护
	//说明：
	//1)如果接收到拒绝命令或在执行命令期间出现错误，自举程序则会发送 NACK 字节并返回检查命令状态。
	//2)读保护 - 激活 RDP（读保护）选项后，只能使用这一有限的命令子集。其它命令都会收到 NACK 应答，并且不会对器件起作用。取消 RDP 即可激活其它命令。
	//3)Erase (x043) 和 Extended Erase (0x44) 均为独占命令。一个器件可支持 Erase 命令或 Extended Erase 命令，但不能同时支持这两个命令。
}ISP_COMMAND_USART_TypeDef;

typedef	enum _ISP_Answer	//USART 应答
{
	ISP_ANSWER_ACK		=0x79,		//ACK 应答(被接受）
	ISP_ANSWER_NACK		=0x1F			//NACK 应答（不接受或者不识别--丢弃)
}ISP_ANSWER_TypeDef;

typedef	struct _ISP_Version		//USART 自举程序版本
{
	unsigned char ISP_VS				;		//自举程序版本（0 < 版本 < 255），示例：0x10 = 版本 1.0
	unsigned char ISP_VS_Get		;		//Get:获取当前自举程序版本及允许使用的命令
	unsigned char ISP_VS_GetVS	;		//Get Version & Read Protection Status:获取自举程序版本及 Flash 的读保护状态
	unsigned char ISP_VS_GetID	;		//Get ID:获取芯片 ID
	unsigned char ISP_VS_RM			;		//Read Memory:从应用程序指定的地址开始读取最多 256 个字节的存储器空间
	unsigned char ISP_VS_Go			;		//Go:跳转到内部 Flash 或 SRAM 内的应用程序代码
	unsigned char ISP_VS_WM			;		//Write Memory:从应用程序指定的地址开始将最多 256 个字节的数据写入 RAM 或 Flash
	unsigned char ISP_VS_Erase	;		//Erase:擦除一个到全部 Flash 页面
	unsigned char ISP_VS_WP			;		//Write Protect:使能某些扇区的写保护
	unsigned char ISP_VS_WU			;		//Write Unprotect:禁止所有 Flash 扇区的写保护
	unsigned char ISP_VS_RP			;		//Readout Protect:使能读保护
	unsigned char ISP_VS_RU			;		//Readout Unprotect:禁止读保护
}ISP_VERSION_TypeDef;

typedef	struct _ISP_Conf					//USART 在线编程配置文件
{
	USART_TypeDef* 	USARTx		;			//USART1,USART2,USART3,UART4;//UART5
	unsigned int NumToSave		;			//接收到的有效数据
	unsigned int NumToSend		;			//需要往串口发送的数据个数
	unsigned int StartAddr		;			//起始地址
	ISP_COMMAND_USART_TypeDef RxCommand	;			//接收到的命令
	unsigned char Command[2]	;			//自举程序命令及异或校验码
	
	unsigned char ISP_RxBuffer[ISP_BufferSize]	;		//接收缓冲区
	unsigned char ISP_RvBuffer[ISP_BufferSize]	;		//接收缓冲区--备份区
	unsigned char ISP_TxBuffer[ISP_BufferSize]	;		//发送缓冲区
	unsigned char ISP_TvBuffer[ISP_BufferSize]	;		//发送缓冲区--备份区
}ISP_Conf_TypeDef;


void Usart_ISP_Cof(ISP_Conf_TypeDef *ISP_Conf);
void Usart_ISP_Process(ISP_Conf_TypeDef *ISP_Conf);
void Usart_ISP_CommandSend(ISP_Conf_TypeDef *ISP_Conf,unsigned char Command);	//串口编程发送命令程序
void Usart_ISP_CommandRead(ISP_Conf_TypeDef *ISP_Conf);			//串口接收命令
	
#endif

