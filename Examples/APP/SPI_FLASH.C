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

//#define USB_TEST_BD				//USB_TEST板
#define CMSIS_CDC_BD			//CMSIS-CDC板

#include "stm32f10x_spi.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
//#include "stm32f10x_nvic.h"

//#include "STM32_WOW.H"
#include "STM32_PWM.H"
#include "STM32_SYS.H"



#if	defined(USB_TEST_BD)
	//SPI接口定义
	#define SPI_FLASH_RCC													RCC_APB1Periph_SPI2
	#define SPI_FLASH_SPI_PORT										SPI2
	#define SPI_FLASH_CLOCK(FunctionalState)			RCC_APB1PeriphClockCmd(SPI_FLASH_RCC, FunctionalState)

	//MOSI管脚定义
	#define SPI_FLASH_MOSI_RCC			RCC_APB2Periph_GPIOB
	#define	SPI_FLASH_MOSI_PORT			GPIOB
	#define	SPI_FLASH_MOSI_PIN			GPIO_Pin_15
	#define SPI_FLASH_MOSI_CLOCK(FunctionalState)			RCC_APB2PeriphClockCmd(SPI_FLASH_MOSI_RCC, FunctionalState)
	//MISO管脚定义
	#define SPI_FLASH_MISO_RCC			RCC_APB2Periph_GPIOB
	#define	SPI_FLASH_MISO_PORT			GPIOB
	#define	SPI_FLASH_MISO_PIN			GPIO_Pin_14
	#define SPI_FLASH_MISO_CLOCK(FunctionalState)			RCC_APB2PeriphClockCmd(SPI_FLASH_MISO_RCC, FunctionalState)
	//SCLK管脚定义
	#define SPI_FLASH_SCLK_RCC			RCC_APB2Periph_GPIOB
	#define	SPI_FLASH_SCLK_PORT			GPIOB
	#define	SPI_FLASH_SCLK_PIN			GPIO_Pin_13
	#define SPI_FLASH_SCLK_CLOCK(FunctionalState)			RCC_APB2PeriphClockCmd(SPI_FLASH_SCLK_RCC, FunctionalState)
	//NSS管脚定义
	#define SPI_FLASH_NSS_RCC				RCC_APB2Periph_GPIOC
	#define	SPI_FLASH_NSS_PORT			GPIOC
	#define	SPI_FLASH_NSS_PIN				GPIO_Pin_8
	#define SPI_FLASH_NSS_CLOCK(FunctionalState)			RCC_APB2PeriphClockCmd(SPI_FLASH_NSS_RCC, FunctionalState)

	#define	SPI_FLASH_CS_ENABLE		PC8=0
	#define	SPI_FLASH_CS_DISABLE	PC8=1
	
#elif	defined(CMSIS_CDC_BD)

	//SPI接口定义
	#define SPI_FLASH_RCC													RCC_APB2Periph_SPI1
	#define SPI_FLASH_SPI_PORT										SPI1
	#define SPI_FLASH_CLOCK(FunctionalState)			RCC_APB2PeriphClockCmd(SPI_FLASH_RCC, FunctionalState)

	//MOSI管脚定义
	#define SPI_FLASH_MOSI_RCC			RCC_APB2Periph_GPIOA
	#define	SPI_FLASH_MOSI_PORT			GPIOA
	#define	SPI_FLASH_MOSI_PIN			GPIO_Pin_7
	#define SPI_FLASH_MOSI_CLOCK(FunctionalState)			RCC_APB2PeriphClockCmd(SPI_FLASH_MOSI_RCC, FunctionalState)
	//MISO管脚定义
	#define SPI_FLASH_MISO_RCC			RCC_APB2Periph_GPIOA
	#define	SPI_FLASH_MISO_PORT			GPIOA
	#define	SPI_FLASH_MISO_PIN			GPIO_Pin_6
	#define SPI_FLASH_MISO_CLOCK(FunctionalState)			RCC_APB2PeriphClockCmd(SPI_FLASH_MISO_RCC, FunctionalState)
	//SCLK管脚定义
	#define SPI_FLASH_SCLK_RCC			RCC_APB2Periph_GPIOA
	#define	SPI_FLASH_SCLK_PORT			GPIOA
	#define	SPI_FLASH_SCLK_PIN			GPIO_Pin_5
	#define SPI_FLASH_SCLK_CLOCK(FunctionalState)			RCC_APB2PeriphClockCmd(SPI_FLASH_SCLK_RCC, FunctionalState)
	//NSS管脚定义
	#define SPI_FLASH_NSS_RCC				RCC_APB2Periph_GPIOA
	#define	SPI_FLASH_NSS_PORT			GPIOA
	#define	SPI_FLASH_NSS_PIN				GPIO_Pin_4
	#define SPI_FLASH_NSS_CLOCK(FunctionalState)			RCC_APB2PeriphClockCmd(SPI_FLASH_NSS_RCC, FunctionalState)

	#define	SPI_FLASH_CS_ENABLE		PA4=0
	#define	SPI_FLASH_CS_DISABLE	PA4=1	
#else
	#error "Board undefined"
#endif

u32 Temp = 0;

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
	PWM_Configuration(TIM2,7200,10000,50);		//运行指示灯配置
	SPI_FLASH_GPIO_Configuration();						//相应管脚配置
	SPI_FLASH_SpiPort_Configuration();				//SPI接口配置	
	SysTick_Configuration(5000);							//系统嘀嗒时钟配置72MHz,单位为uS
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
		SPI_FLASH_ReadID();
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
void SPI_FLASH_GPIO_Configuration(void)
{
	//结构体定义
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//打开相应管脚时钟
	SPI_FLASH_MOSI_CLOCK(ENABLE);
	SPI_FLASH_MISO_CLOCK(ENABLE);
	SPI_FLASH_SCLK_CLOCK(ENABLE);
	SPI_FLASH_NSS_CLOCK(ENABLE);
	
	//MOSI管脚定义
	GPIO_InitStructure.GPIO_Pin=SPI_FLASH_MOSI_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_Init(SPI_FLASH_MOSI_PORT,&GPIO_InitStructure);
	//MISO管脚定义
	GPIO_InitStructure.GPIO_Pin=SPI_FLASH_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;					//GPIO_Mode_IPU			//GPIO_Mode_AF_PP
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_Init(SPI_FLASH_MISO_PORT,&GPIO_InitStructure);
	//SCLK管脚定义
	GPIO_InitStructure.GPIO_Pin=SPI_FLASH_SCLK_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_Init(SPI_FLASH_SCLK_PORT,&GPIO_InitStructure);
	//NSS管脚定义
	GPIO_InitStructure.GPIO_Pin=SPI_FLASH_NSS_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;				//GPIO_Mode_Out_PP  //GPIO_Mode_AF_PP
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_Init(SPI_FLASH_NSS_PORT,&GPIO_InitStructure);
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
void SPI_FLASH_SpiPort_Configuration(void)
{
	//结构体定义
	SPI_InitTypeDef	SPI_InitStructure;
	//打开SPI时钟
	SPI_FLASH_CLOCK(ENABLE);
	SPI_Cmd(SPI_FLASH_SPI_PORT,DISABLE);
	//设置SPI参数
	SPI_InitStructure.SPI_Direction						=		SPI_Direction_2Lines_FullDuplex;	//设置方向				（2线全双工、2线只接收、一线发送、一线接收）
	SPI_InitStructure.SPI_Mode								=		SPI_Mode_Master;									//模式         	（从或主设备）
	SPI_InitStructure.SPI_DataSize						=		SPI_DataSize_8b;									//宽度         	（8或16位）
	SPI_InitStructure.SPI_CPOL								=		SPI_CPOL_High;										//时钟极性     	（低或高）
	SPI_InitStructure.SPI_CPHA								=		SPI_CPHA_2Edge;					 					//时钟相位     	（第一个或第二个跳变沿）
	SPI_InitStructure.SPI_NSS									=		SPI_NSS_Hard;											//片选方式     	（硬件或软件方式）
	SPI_InitStructure.SPI_BaudRatePrescaler		=		SPI_BaudRatePrescaler_8;					//波特率预分频 	（从2---256分频）
	SPI_InitStructure.SPI_FirstBit						=		SPI_FirstBit_MSB;									//最先发送的位 	（最低位，还是最高位在先）
	SPI_InitStructure.SPI_CRCPolynomial				=		0X07;															//设置crc多项式 （数字）如7
	//初始化SPI
	SPI_Init(SPI_FLASH_SPI_PORT,&SPI_InitStructure);
	//如果在主机模式下的片选方式为硬件（SPI_NSS_Hard）方式，此处必须打开，否则NSS无信号
	SPI_SSOutputCmd(SPI_FLASH_SPI_PORT, ENABLE);		
	//使能SPI
	SPI_Cmd(SPI_FLASH_SPI_PORT,ENABLE);
	
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
u8 SPI_FLASH_SendByte(u8 byte)
{
  /* 等待DR寄存器空 */
  while (SPI_I2S_GetFlagStatus(SPI_FLASH_SPI_PORT, SPI_I2S_FLAG_TXE) == RESET);
	 
  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(SPI_FLASH_SPI_PORT, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI_FLASH_SPI_PORT, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI_FLASH_SPI_PORT);
}

/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
u32 SPI_FLASH_ReadID(void)
{
	u32 Temp = 0,Temp0=0,Temp1=0,Temp2=0;

  /* Select the FLASH: Chip Select low */
//  SPI_FLASH_CS_LOW();
//	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	SPI_FLASH_CS_ENABLE;
	SPI_Cmd(SPI_FLASH_SPI_PORT, ENABLE);
  /* Send "RDID " instruction */
  SPI_FLASH_SendByte(0x9F);
//	SPI_SendByte(0X00);
//	SPI_SendByte(0X00);
//	SPI_SendByte(0X00);
  /* Read a byte from the FLASH */
  Temp0 = SPI_FLASH_SendByte(0X01);

  /* Read a byte from the FLASH */
  Temp1 = SPI_FLASH_SendByte(0X01);

  /* Read a byte from the FLASH */
  Temp2 = SPI_FLASH_SendByte(0X01);

  /* Deselect the FLASH: Chip Select high */
//  SPI_FLASH_CS_HIGH();
//	GPIO_SetBits(GPIOA,GPIO_Pin_4);

  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;
	SPI_Cmd(SPI_FLASH_SPI_PORT, DISABLE);
	SPI_FLASH_CS_DISABLE;
  return Temp;	
}







































#endif
