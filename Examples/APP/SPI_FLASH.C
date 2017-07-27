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

*******************************************************************************/
#ifdef SPI_FLASH							//如果定义了SPI_FLASH 此功能生效
#include "SPI_FLASH.H"

#define USB_TEST_BD				//USB_TEST板
//#define CMSIS_CDC_BD			//CMSIS-CDC板

#include "stm32f10x_spi.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
//#include "stm32f10x_nvic.h"

//#include "STM32_WOW.H"
#include "STM32_PWM.H"
#include "STM32_SYS.H"
#include "STM32_SPI.H"
#include "STM32_SYSTICK.H"


u32 Temp = 0;
SPI_Conf_TypeDef	FLASH_Conf;

u8 RevBuffer[100]={0};
u8 SendBuffer[100]={0x9F,0xFF,0XFF,0XFF};

/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void SPI_FLASH_Configuration(void)
{
	SYS_Configuration();											//系统配置 STM32_SYS.H	
	PWM_OUT(TIM2,PWM_OUTChannel1,1,500);			//PWM设定-20161127版本
	
	SPI_FLASH_Conf(&FLASH_Conf);			//SPI设置
//	SPI_FLASH_GPIO_Configuration();						//相应管脚配置
//	SPI_FLASH_SpiPort_Configuration();				//SPI接口配置
		
	SysTick_Configuration(1000);							//系统嘀嗒时钟配置72MHz,单位为uS
}

/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void SPI_FLASH_Server(void)
{
	if(Temp!=0)
	{
		Temp=0;
//		SPI_FLASH_CS_ENABLE;
//		SPI_Cmd(SPI_FLASH_SPI_PORT, ENABLE);
//		SPI_FLASH_ReadID(&FLASH_Conf);
		STM32_SPI_ReadWriteBuffer(&FLASH_Conf,100,SendBuffer,RevBuffer);		//连接读数据
	}
	else
	{
		Temp=1;
//		SPI_FLASH_CS_DISABLE;
//		SPI_Cmd(SPI_FLASH_SPI_PORT, DISABLE);
	}
//	Temp=0;
//	Temp=SPI_FLASH_ReadID();
	
//	SPI_FLASH_CS_ENABLE;
//	SPI_FLASH_SendByte(0x9F);
//	SPI_FLASH_CS_DISABLE;
//	u16 num=0xFFFE;
//	SPI_Cmd(SPI1, ENABLE);
	/*
	//1）********SPI发送
	if(SPI1_NSS==SPI_NSS_Hard)		//1）**********NSS片选方式为硬件方式（SPI_NSS_Hard）时发送数据
	{
		SPI_Cmd(SPI1, ENABLE);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(SPI1,0X55);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
		SPI_Cmd(SPI1, DISABLE);
	}
	else				//2）**********NSS片选方式为软件件方式（SPI_NSS_Soft）时发送数据
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(SPI1,0X55);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
		GPIO_SetBits(GPIOA,GPIO_Pin_4);
	}
	//1.2）********SPI2发送
	if(SPI2_NSS==SPI_NSS_Hard)		//1）**********NSS片选方式为硬件方式（SPI_NSS_Hard）时发送数据
	{	
		SPI_Cmd(SPI2, ENABLE);
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(SPI2,0XAA);
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
		SPI_Cmd(SPI2, DISABLE);
	}
	else													//2）**********NSS片选方式为软件件方式（SPI_NSS_Soft）时发送数据
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(SPI2,0XAA);
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
	}
	*/


//		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
//		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
//		SPI_I2S_SendData(SPI1,0X55);
//		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
//		GPIO_SetBits(GPIOA,GPIO_Pin_4);
		
		
//	SPI_TX_DMAFlagClear(SPI1);
	//2）********SPI_DMA发送

//		SPI_Buffer
//		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
//		SPI_Cmd(SPI1, ENABLE);
////		SPI_DMAPrintf(SPI1,"%d",num);		//SPI_DMA发送函数----后边的省略号就是可变参数
//		//SPI_DMAPrintf(SPI1,"%s",SPI_Buffer);		//SPI_DMA发送函数----后边的省略号就是可变参数
//		SPI_DMA_BufferWrite(SPI1,(u32*)SPI_Buffer,10);		//SPI_DMA发送函数----后边的省略号就是可变参数
//		while(SPI_TX_DMAFlagClear(SPI1)==ERROR);
//		while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
//		SPI_Cmd(SPI1, DISABLE);
//		free(SPI_BUFFER);						//释放动态空间
//		GPIO_SetBits(GPIOA,GPIO_Pin_4);
//		
////		GPIO_ResetBits(GPIOB,GPIO_Pin_12);
//		SPI_Cmd(SPI2, ENABLE);
////		SPI_DMAPrintf(SPI1,"%d",num);		//SPI_DMA发送函数----后边的省略号就是可变参数
//		//SPI_DMAPrintf(SPI1,"%s",SPI_Buffer);		//SPI_DMA发送函数----后边的省略号就是可变参数
//		SPI_DMA_BufferWrite(SPI2,(u32*)SPI_Buffer,10);		//SPI_DMA发送函数----后边的省略号就是可变参数
//		while(SPI_TX_DMAFlagClear(SPI2)==ERROR);
//		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
//		SPI_Cmd(SPI2, DISABLE);
//		free(SPI_BUFFER);						//释放动态空间
//		GPIO_SetBits(GPIOB,GPIO_Pin_12);


//	free(SPI_BUFFER);						//释放动态空间
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
void SPI_FLASH_Conf(SPI_Conf_TypeDef *SPI_Conf)
{
//	#define USB_TEST_BD				//USB_TEST板
//	#define CMSIS_CDC_BD			//CMSIS-CDC板
	#ifdef	USB_TEST_BD
	SPI_Conf->SPIx=SPI2;
	SPI_Conf->SPI_CS_PORT=GPIOC;
	SPI_Conf->SPI_CS_PIN=GPIO_Pin_8;	
	SPI_Conf->SPI_BaudRatePrescaler_x=SPI_BaudRatePrescaler_8;
	
	#else
	
	SPI_Conf->SPIx=SPI1;
	SPI_Conf->SPI_CS_PORT=GPIOA;
	SPI_Conf->SPI_CS_PIN=GPIO_Pin_4;	
	SPI_Conf->SPI_BaudRatePrescaler_x=SPI_BaudRatePrescaler_2;
	
	#endif
	
	STM32_SPI_ConfigurationNR(SPI_Conf);				//普通SPI接口配置
}

/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
u32 SPI_FLASH_ReadID(SPI_Conf_TypeDef *SPI_Conf)
{
	STM32_SPI_ReadWriteBuffer(&FLASH_Conf,4,SendBuffer,RevBuffer);		//连接读数据
	return 0;
}







































#endif
