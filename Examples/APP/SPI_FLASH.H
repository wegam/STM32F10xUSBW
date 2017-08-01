#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

//#include "STM32_USART.H"
//#include "STM32_TIM.H"
//#include "STM32_DMA.H"
//#include "STM32_PWM.H"
//#include "STM32_GPIO.H"
//#include "STM32_ADC.H"
//#include "STM32_DAC.H"
//#include "STM32_SPI.H"

#include "stm32f10x_type.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"

#include	"stdio.h"				//用于printf
#include	"string.h"			//用于printf
#include	"stdarg.h"			//用于获取不确定个数的参数
#include	"stdlib.h"			//malloc动态申请内存空间
	
#include	"stddef.h"
#include	"stdint.h"

#include "STM32F10x_BitBand.H"

//#include "STM32_SYSTICK.H"
//#include "STM32_SPI.H"

/*--------------------------------FLASH部分术语解析-----------------------------
Sector：比BLOCK更小的单位
Block：	大部分的FLASH都以64K为单位空间ERASE
Chip：	整片FLASH
Page：
例：(256-byte per page)(4K-byte per sector/16page per sector)(64K-byte per block/16-sector per block)(8-block per chip)
————————————————————————————————————————————*/
//----SPI管脚配置--------------------------------------------------------------
typedef struct	_SPI_Flash
{
	//____________端口定义
	SPI_TypeDef *			SPIx;				//SPI接口配置 x=0、1、2、3、

	//____________SPI_CS定义
	GPIO_TypeDef* 	SPI_CS_PORT;		//GPIOX
	unsigned short 	SPI_CS_PIN;			//GPIO_Pin_x

	//____________SPI_CLK定义
	GPIO_TypeDef* 	SPI_CLK_PORT;		//GPIOX
	unsigned short 	SPI_CLK_PIN;		//GPIO_Pin_x

	//____________SPI_MISO定义
	GPIO_TypeDef* 	SPI_MISO_PORT;	//GPIOX
	unsigned short 	SPI_MISO_PIN;		//GPIO_Pin_x

	//____________SPI_MOSI定义
	GPIO_TypeDef* 	SPI_MOSI_PORT;	//GPIOX
	unsigned short 	SPI_MOSI_PIN;		//GPIO_Pin_x
	
	//____________SPI_波特率
	unsigned short 	SPI_BaudRatePrescaler_x;		//x=2/4/8/16/32/64/128/256
	
	unsigned char SPI_Flash_NSS_CsFlg;									//如果使用纯硬件SPI（含CS脚），	SPI_Flash_NSS_CsFlg=1，		否则SPI_Flash_NSS_CsFlg=0；此由SPI_Flash_ConfigurationNR根据SPI管脚判断设置此值
	unsigned char SPI_Flash_USER_PotFlg;								//如果使用硬件SPI接口，				SPI_Flash_USER_PotFlg=1，	否则SSPI_Flash_USER_PotFlg=0；此由SPI_Flash_ConfigurationNR根据SPI管脚判断设置此值
	//------------配置完成后读取FLASH_ID确认FLASH型号后自动补充以下数据
	unsigned int	SPI_FLASH_PageSize;						//(256-byte per page)
	unsigned int	SPI_FLASH_SectorSize;					//(4K-byte per sector)
	unsigned int	SPI_FLASH_BlockSize;					//(64K-byte per block)
	
}SPI_FLASH_TypeDef;

typedef	enum _SPI_FLASH_CLASS		//FLASH_MX25L4006E
{
	ChipSize		=	0x01,
	SectorSize	=	4*1024,		//扇区大小4K Byte
	BlockSize		=	64*1024,	//块大小64K
	PageSize		=	256,		//页大小256 Byte
	
	MX25L4006E_PageSize		=	256,		//(256-byte per page)
	MX25L4006E_SectorSize,					//(4K-byte per sector)
	MX25L4006E_BlockSize,						//(64K-byte per block)
	
	
	sat
}SPI_FLASH_CLASS;		

/******************************Flash命令定义参考MX25L4006E*******************************/
#define	Flash_WREN		(unsigned char)0x06				//写使能
#define	Flash_WRDI		(unsigned char)0x04				//写失能
#define	Flash_WRSR		(unsigned char)0x01				//写状态寄存器
#define	Flash_RDID		(unsigned char)0x9F				//读设备信息，3byte；第一字节为制造商代码，第二第三字节为芯片代码
#define	Flash_RDSR		(unsigned char)0x05				//读状态寄存器
#define	Flash_READ		(unsigned char)0x03				//读数据
#define	Flash_FREAD		(unsigned char)0x0B				//快速读取数据
#define	Flash_RDSFDP	(unsigned char)0x5A				//
#define	Flash_RES			(unsigned char)0xAB				//读器件地址
#define	Flash_REMS		(unsigned char)0x90				//读制造商代码和产品代码
#define	Flash_DREAD		(unsigned char)0x3B				//
#define	Flash_SE			(unsigned char)0x20				//扇区擦除
#define	Flash_BE			(unsigned char)0x52				//块擦除		也可以用0XD8
#define	Flash_CE			(unsigned char)0x60				//芯片擦除; 整片擦除; 也可以用0XC7
#define	Flash_WPAGE		(unsigned char)0x02				//写页面
#define	Flash_DP			(unsigned char)0xB9				//深度省电状态
#define	Flash_RDP			(unsigned char)0xAB				//退出深度掉电
	
//标志位
#define	WIP_Flag			(unsigned char)1<<0				//Write In Progress (WIP) flag
#define	WEL_Flag			(unsigned char)1<<1				//write enable latch (WEL) flag
	

void SPI_FLASH_Configuration(void);
void SPI_FLASH_Server(void);

//------------配置函数
void SPI_FLASH_Conf(SPI_FLASH_TypeDef *SPI_Conf);			//SPI参数设置
//---------------------------------------------------------------------------------

void SPI_FLASH_ConfigurationNR(SPI_FLASH_TypeDef *SPI_Conf);					//普通SPI接口配置--未开中断和DMA
void SPI_FLASH_GetInfo(SPI_FLASH_TypeDef *SPI_Conf);		//获取FLASH信息---根据ID确定FLASH型号，配置页大小，扇区大小，块大小参数

//------------数据读写函数
void SPI_FLASH_PageWrite(SPI_FLASH_TypeDef *SPI_Conf,u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);		//FLASH写一页数据
void SPI_FLASH_BufferWrite(SPI_FLASH_TypeDef *SPI_Conf,u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);	//FLASH写缓冲数据
void SPI_FLASH_BufferRead(SPI_FLASH_TypeDef *SPI_Conf,u8* pBuffer, u32 ReadAddr, u16 NumByteToRead);		//SPI读缓冲区

//------------擦除函数
void SPI_FLASH_SectorErase(SPI_FLASH_TypeDef *SPI_Conf,u32 SectorAddr);		//Flash扇区擦除Fast erase time: 60ms(typ.)/sector (4K-byte per sector) ; 0.7s(typ.)/block (64K-byte per block)
void SPI_FLASH_BulkErase(SPI_FLASH_TypeDef *SPI_Conf);										//Flash块擦除Fast erase time: 60ms(typ.)/sector (4K-byte per sector) ; 0.7s(typ.)/block (64K-byte per block)
void SPI_FLASH_ChipErase(SPI_FLASH_TypeDef *SPI_Conf);										//FLASH整片擦除


//************************************以下为内部调用函数

//------------控制函数
void SPI_FLASH_ENALBE		(SPI_FLASH_TypeDef 	*SPI_Conf);								//SPI_FLASH_使能
void SPI_FLASH_DISALBE	(SPI_FLASH_TypeDef 	*SPI_Conf);								//SPI_FLASH_关闭
//------------
u8 	SPI_FLASH_WriteReadByte(SPI_FLASH_TypeDef *SPI_Conf,u8 byte);			//Flash发送一字节并读取一字节数据
//------------
void SPI_FLASH_WriteEnable(SPI_FLASH_TypeDef *SPI_Conf);							//0x06写使能
void SPI_FLASH_WriteDisable(SPI_FLASH_TypeDef *SPI_Conf);							//0x写保护--写禁止
void SPI_FLASH_WaitForWriteEnd(SPI_FLASH_TypeDef *SPI_Conf);					//等待FLASH写完成
//------------状态函数
u8 SPI_FLASH_ReadStatus(SPI_FLASH_TypeDef *SPI_Conf);									//读Flash状态寄存器
void SPI_FLASH_WriteStatus(SPI_FLASH_TypeDef *SPI_Conf,u8 Status);		//写Flash状态寄存器
//------------
u32 SPI_FLASH_ReadID(SPI_FLASH_TypeDef *SPI_Conf);										//读FlashID


//------------其它--暂未调试
void SPI_FLASH_StartReadSequence(SPI_FLASH_TypeDef *SPI_Conf,u32 ReadAddr);	//开始读数据
u16 SPI_FLASH_SendHalfWord(SPI_FLASH_TypeDef *SPI_Conf,u16 HalfWord);	//发送16位数据
//----------------------------------------------------------------------
//void SPI_FLASH_WriteCommand(SPI_FLASH_TypeDef *SPI_Conf,u8 Flash_Command);				//写命令


#endif

