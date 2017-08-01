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

#define 	Dummy_Byte 0xA5					//FLASH空操作数
#define		SPI_FLASH_PageSize   		0x100		//Flash页大小
#define		FLASH_WriteAddress     	0x00
#define		FLASH_ReadAddress      	FLASH_WriteAddress
#define		FLASH_SectorToErase    	FLASH_WriteAddress
#define 	FlASH_BufferSize				100

u32 Temp = 0;
SPI_FLASH_TypeDef	FLASH_Conf;

u16	FLASH_ChipEraseFlag=0;			//启动后擦除一次FLASH
u16	RWF=0;			//读写标志位

u32 FlashID=0;
u8 FlashStatus=WEL_Flag;
u8 RevBuffer[FlASH_BufferSize]={0};
u8 SendBuffer[FlASH_BufferSize]={0x9F,0xFF,0XFF,0XFF};
u8 testSbuffer[FlASH_BufferSize]={0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfA};
u8 testRbuffer[FlASH_BufferSize]={0};
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
	FlashStatus=WEL_Flag;	
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
//		SPI_FLASH_ReadStatus(&FLASH_Conf);		//读Flash状态寄存器
//		SPI_FLASH_WriteEnable(&FLASH_Conf);		//0x06写使能
//		SPI_FLASH_WriteStatus(&FLASH_Conf,0x00);	//Flash发送一字节并读取一字节数据
//		SPI_FLASH_WriteEnable(&FLASH_Conf);		//0x06写使能
//		SPI_FLASH_ReadStatus(&FLASH_Conf);		//读Flash状态寄存器
//		SPI_FLASH_ReadStatus(&FLASH_Conf);		//读Flash状态寄存器
//		SPI_FLASH_ReadStatus(&FLASH_Conf);		//读Flash状态寄存器
//		SPI_FLASH_ReadStatus(&FLASH_Conf);		//读Flash状态寄存器
//		STM32_SPI_ReadWriteBuffer(&FLASH_Conf,100,SendBuffer,RevBuffer);		//连接读数据
//		if(FLASH_ChipEraseFlag>=1000)
//		SPI_FLASH_ReadID(&FLASH_Conf);			//读FlashID
	}
	else
	{
		Temp=1;
//		SPI_FLASH_CS_DISABLE;
//		SPI_Cmd(SPI_FLASH_SPI_PORT, DISABLE);
	}
//	FLASH_ChipEraseFlag=0;			//启动后擦除一次FLASH
	if(FLASH_ChipEraseFlag<=1000)
	{
		FLASH_ChipEraseFlag++;
		if(FLASH_ChipEraseFlag==1000)
		{
//		SPI_FLASH_BulkErase(&FLASH_Conf);										//FLASH整片擦除
//		SPI_FLASH_ChipErase(&FLASH_Conf);										//FLASH整片擦除
//			SPI_FLASH_SectorErase(&FLASH_Conf,FLASH_ReadAddress);	//Flash扇区擦除
		}
	}
	RWF++;
	if(RWF==3000)
	{
		/* Erase SPI FLASH Sector to write on */
//		SPI_FLASH_SectorErase(&FLASH_Conf,FLASH_SectorToErase);
//		FlashID=SPI_FLASH_ReadID(&FLASH_Conf);			//读FlashID
//		testSbuffer[10]=(u8)(FlashID>>16);
//		testSbuffer[11]=(u8)(FlashID>>8);
//		testSbuffer[12]=(u8)(FlashID>>0);
//		SPI_FLASH_ReadID(&FLASH_Conf);
//		SPI_FLASH_ReadStatus(&FLASH_Conf);		//读Flash状态寄存器
//		SPI_FLASH_WriteEnable(&FLASH_Conf);		//0x06写使能
//		SPI_FLASH_WriteStatus(&FLASH_Conf,0xFF);	//Flash发送一字节并读取一字节数据
//		SPI_FLASH_ReadStatus(&FLASH_Conf);		//读Flash状态寄存器
//		SPI_FLASH_ReadStatus(&FLASH_Conf);		//读Flash状态寄存器
//		SPI_FLASH_ReadStatus(&FLASH_Conf);		//读Flash状态寄存器
//		SPI_FLASH_ReadStatus(&FLASH_Conf);		//读Flash状态寄存器
//			SPI_FLASH_WriteEnable(&FLASH_Conf);		//0x06写使能
//			SPI_FLASH_PageWrite(&FLASH_Conf,testSbuffer, FLASH_WriteAddress, FlASH_BufferSize);		//FLASH写一页数据
//		SPI_FLASH_BufferWrite(&FLASH_Conf,testSbuffer, FLASH_WriteAddress, FlASH_BufferSize);	//FLASH写缓冲数据
	}
	else if(RWF==5000)
	{
		RWF=0; 
//		memset(testRbuffer, 0x00, 10);
//		FlashStatus=SPI_FLASH_ReadStatus(&FLASH_Conf);		//读Flash状态寄存器
		/* Read data from SPI FLASH memory */
//		FlashID=SPI_FLASH_ReadID(&FLASH_Conf);			//读FlashID
//		testRbuffer[10]=(u8)(FlashID>>16);
//		testRbuffer[11]=(u8)(FlashID>>8);
//		testRbuffer[12]=(u8)(FlashID>>0);
		SPI_FLASH_BufferRead(&FLASH_Conf,testRbuffer, FLASH_ReadAddress, FlASH_BufferSize);
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
void SPI_FLASH_Conf(SPI_FLASH_TypeDef *SPI_Conf)
{
//	#define USB_TEST_BD				//USB_TEST板
//	#define CMSIS_CDC_BD			//CMSIS-CDC板
	#ifdef	USB_TEST_BD
	SPI_Conf->SPIx=SPI2;
	SPI_Conf->SPI_CS_PORT=GPIOC;
	SPI_Conf->SPI_CS_PIN=GPIO_Pin_8;	
	SPI_Conf->SPI_BaudRatePrescaler_x=SPI_BaudRatePrescaler_64;
	
	#else
	
	SPI_Conf->SPIx=SPI1;
	SPI_Conf->SPI_CS_PORT=GPIOA;
	SPI_Conf->SPI_CS_PIN=GPIO_Pin_4;	
	SPI_Conf->SPI_BaudRatePrescaler_x=SPI_BaudRatePrescaler_2;
	
	#endif
	
	SPI_FLASH_ConfigurationNR(SPI_Conf);				//普通SPI接口配置
}


/*******************************************************************************
* Function Name  : SPI_FLASH_PageWrite
* Description    : Writes more than one byte to the FLASH with a single WRITE
*                  cycle(Page WRITE sequence). The number of byte can't exceed
*                  the FLASH page size.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH,
*                    must be equal or less than "SPI_FLASH_PageSize" value.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_PageWrite(SPI_FLASH_TypeDef *SPI_Conf,u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)	//FLASH写一页数据
{
	SPI_FLASH_WaitForWriteEnd(SPI_Conf);									//等待FLASH写完成
	SPI_FLASH_WriteEnable(SPI_Conf);											//0x06写使能
	
  SPI_FLASH_ENALBE(SPI_Conf);														//SPI_FLASH_使能
	
  SPI_FLASH_WriteReadByte(SPI_Conf,Flash_WPAGE);											//页写命令
  SPI_FLASH_WriteReadByte(SPI_Conf,(WriteAddr & 0xFF0000) >> 16);		//写入高8位地址
	SPI_FLASH_WriteReadByte(SPI_Conf,(WriteAddr & 0xFF00) >> 8);				//写入中间8位地址
  SPI_FLASH_WriteReadByte(SPI_Conf,WriteAddr & 0xFF);								//写入低8位地址
	
  while (NumByteToWrite--)		//开始写入数据，直到写完
  {
    SPI_FLASH_WriteReadByte(SPI_Conf,*pBuffer);				//写入数据
    pBuffer++;
  }
	SPI_FLASH_DISALBE(SPI_Conf);											//SPI_FLASH_关闭
	
  SPI_FLASH_WaitForWriteEnd(SPI_Conf);						//等待写入完成
	
}
/*******************************************************************************
* Function Name  : SPI_FLASH_BufferWrite
* Description    : Writes block of data to the FLASH. In this function, the
*                  number of WRITE cycles are reduced, using Page WRITE sequence.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BufferWrite(SPI_FLASH_TypeDef *SPI_Conf,u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
  u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  Addr = WriteAddr % SPI_FLASH_PageSize;
  count = SPI_FLASH_PageSize - Addr;
  NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
  NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

  if (Addr == 0) /* WriteAddr is SPI_FLASH_PageSize aligned  */
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      SPI_FLASH_PageWrite(SPI_Conf,pBuffer, WriteAddr, NumByteToWrite);
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(SPI_Conf,pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      SPI_FLASH_PageWrite(SPI_Conf,pBuffer, WriteAddr, NumOfSingle);
    }
  }
  else /* WriteAddr is not SPI_FLASH_PageSize aligned  */
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      if (NumOfSingle > count) /* (NumByteToWrite + WriteAddr) > SPI_FLASH_PageSize */
      {
        temp = NumOfSingle - count;

        SPI_FLASH_PageWrite(SPI_Conf,pBuffer, WriteAddr, count);
        WriteAddr +=  count;
        pBuffer += count;

        SPI_FLASH_PageWrite(SPI_Conf,pBuffer, WriteAddr, temp);
      }
      else
      {
        SPI_FLASH_PageWrite(SPI_Conf,pBuffer, WriteAddr, NumByteToWrite);
      }
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
      NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

      SPI_FLASH_PageWrite(SPI_Conf,pBuffer, WriteAddr, count);
      WriteAddr +=  count;
      pBuffer += count;

      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(SPI_Conf,pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      if (NumOfSingle != 0)
      {
        SPI_FLASH_PageWrite(SPI_Conf,pBuffer, WriteAddr, NumOfSingle);
      }
    }
  }
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BufferRead
* Description    : Reads a block of data from the FLASH.
* Input          : - pBuffer : pointer to the buffer that receives the data read
*                    from the FLASH.
*                  - ReadAddr : FLASH's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BufferRead(
													SPI_FLASH_TypeDef *SPI_Conf,
													u8* pBuffer, 					//数据的数据存储缓冲区
													u32 ReadAddr, 				//读取的起始地址
													u16 NumByteToRead			//待读取字节数
)
{
//	while(SPI_FLASH_ReadStatus(&FLASH_Conf)&WIP_Flag==WIP_Flag);		//读Flash状态寄存器
	SPI_FLASH_WaitForWriteEnd(SPI_Conf);	//等待FLASH写完成
	
	SPI_FLASH_ENALBE(SPI_Conf);								//SPI_FLASH_使能
	
  /* Send "Read from Memory " instruction */
  SPI_FLASH_WriteReadByte(SPI_Conf,Flash_READ);

  /* Send ReadAddr high nibble address byte to read from */
  SPI_FLASH_WriteReadByte(SPI_Conf,(ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte to read from */
  SPI_FLASH_WriteReadByte(SPI_Conf,(ReadAddr& 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte to read from */
  SPI_FLASH_WriteReadByte(SPI_Conf,ReadAddr & 0xFF);

  while (NumByteToRead--) /* while there is data to be read */
  {
    /* Read a byte from the FLASH */
    *pBuffer = SPI_FLASH_WriteReadByte(SPI_Conf,Dummy_Byte);
    /* Point to the next location where the byte read will be saved */
    pBuffer++;
  }
	SPI_FLASH_DISALBE(SPI_Conf);											//SPI_FLASH_关闭
}


/*******************************************************************************
* Function Name  : SPI_FLASH_StartReadSequence
* Description    : Initiates a read data byte (READ) sequence from the Flash.
*                  This is done by driving the /CS line low to select the device,
*                  then the READ instruction is transmitted followed by 3 bytes
*                  address. This function exit and keep the /CS line low, so the
*                  Flash still being selected. With this technique the whole
*                  content of the Flash is read with a single READ instruction.
* Input          : - ReadAddr : FLASH's internal address to read from.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_StartReadSequence(SPI_FLASH_TypeDef *SPI_Conf,u32 ReadAddr)
{
//  /* Select the FLASH: Chip Select low */
//  SPI_FLASH_CS_LOW(SPI_Conf);

  /* Send "Read from Memory " instruction */
  SPI_FLASH_WriteReadByte(SPI_Conf,Flash_READ);

  /* Send the 24-bit address of the address to read from -----------------------*/
  /* Send ReadAddr high nibble address byte */
  SPI_FLASH_WriteReadByte(SPI_Conf,(ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte */
  SPI_FLASH_WriteReadByte(SPI_Conf,(ReadAddr& 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte */
  SPI_FLASH_WriteReadByte(SPI_Conf,ReadAddr & 0xFF);
}




/*******************************************************************************
* Function Name  : SPI_FLASH_SendHalfWord
* Description    : Sends a Half Word through the SPI interface and return the
*                  Half Word received from the SPI bus.
* Input          : Half Word : Half Word to send.
* Output         : None
* Return         : The value of the received Half Word.
*******************************************************************************/
u16 SPI_FLASH_SendHalfWord(SPI_FLASH_TypeDef *SPI_Conf,u16 HalfWord)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI_Conf->SPIx, SPI_I2S_FLAG_TXE) == RESET);

  /* Send Half Word through the SPI1 peripheral */
  SPI_I2S_SendData(SPI_Conf->SPIx, HalfWord);

  /* Wait to receive a Half Word */
  while (SPI_I2S_GetFlagStatus(SPI_Conf->SPIx, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the Half Word read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI_Conf->SPIx);
}



















/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
void SPI_FLASH_ConfigurationNR(SPI_FLASH_TypeDef *SPI_Conf)
{
		//1)**********定义相关结构体
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	//2)**********相关GPIO配置
	if(SPI_Conf->SPIx==SPI1)
	{
		//PA4-NSS;PA5-SCK;PA6-MISO;PA7-MOSI;
		//2.1)**********打开SPI时钟	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 ,ENABLE);			//开启SPI时钟	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA, ENABLE);	
		
		if((SPI_Conf->SPI_CS_PORT==GPIOA)&&(SPI_Conf->SPI_CS_PIN==GPIO_Pin_4))			//如果SPI_NSS为SPI_NSS_Soft（软件控制方式）
		{
//			SPI_SSOutputCmd(Pinfo->sSPIx, ENABLE);			//如果在主机模式下的片选方式为硬件（SPI_NSS_Hard）方式，此处必须打开，否则NSS无信号
			SPI_Conf->SPI_Flash_NSS_CsFlg=1;		//如果使用纯硬件SPI1（含CS脚），SPI1_CsFlg=1，否则SPI1_CsFlg=0；
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else
		{
//			SPI_SSOutputCmd(Pinfo->sSPIx, DISABLE);			//如果在主机模式下的片选方式为硬件（SPI_NSS_Hard）方式，此处必须打开，否则NSS无信号
			SPI_Conf->SPI_Flash_NSS_CsFlg=0;		//如果使用纯硬件SPI1（含CS脚），SPI1_CsFlg=1，否则SPI1_CsFlg=0；
			//开CS-GPIO时钟
			if(SPI_Conf->SPI_CS_PORT==GPIOA)
			{
				if((SPI_Conf->SPI_CS_PIN==GPIO_Pin_13)||(SPI_Conf->SPI_CS_PIN==GPIO_Pin_14)||(SPI_Conf->SPI_CS_PIN==GPIO_Pin_15))
				{
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
					//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);			//关闭SW功能
					GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);		//关闭JTAG,SW功能开启
				}
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOB)
			{
				if((SPI_Conf->SPI_CS_PIN==GPIO_Pin_3)||(SPI_Conf->SPI_CS_PIN==GPIO_Pin_4))
				{
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
					GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);				//关闭JTAG
				}
				else
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOC)
			{
				if((SPI_Conf->SPI_CS_PIN==GPIO_Pin_14)||(SPI_Conf->SPI_CS_PIN==GPIO_Pin_15))
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_AFIO, ENABLE);
				else
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOD)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOE)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOF)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOG)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
			}
			//SCK,MISO,MOSI配置
			GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
			GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			//CS配置
			GPIO_InitStructure.GPIO_Pin 	= SPI_Conf->SPI_CS_PIN;
			GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(SPI_Conf->SPI_CS_PORT, &GPIO_InitStructure);
		}
	}
	else if(SPI_Conf->SPIx==SPI2)
	{
		//PB12-NSS;PB13-SCK;PB14-MISO;PB15-MOSI;
		//2.2)**********打开SPI时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 ,ENABLE);				//开启SPI时钟			
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB, ENABLE);
		
		if((SPI_Conf->SPI_CS_PORT==GPIOB)&&(SPI_Conf->SPI_CS_PIN==GPIO_Pin_12))			//如果SPI_NSS为SPI_NSS_Soft（软件控制方式）
		{
			SPI_Conf->SPI_Flash_NSS_CsFlg=1;		//如果使用纯硬件SPI2（含CS脚），SPI2_CsFlg=1，否则SPI2_CsFlg=0；
			SPI_SSOutputCmd(SPI_Conf->SPIx, ENABLE);			//如果在主机模式下的片选方式为硬件（SPI_NSS_Hard）方式，此处必须打开，否则NSS无信号
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
			GPIO_Init(GPIOB, &GPIO_InitStructure);	
		}
		else
		{
			
//			SPI_SSOutputCmd(Pinfo->sSPIx, DISABLE);			//如果在主机模式下的片选方式为硬件（SPI_NSS_Hard）方式，此处必须打开，否则NSS无信号
			SPI_Conf->SPI_Flash_NSS_CsFlg=0;		//如果使用纯硬件SPI2（含CS脚），SPI2_CsFlg=1，否则SPI2_CsFlg=0；
			//开CS-GPIO时钟
			if(SPI_Conf->SPI_CS_PORT==GPIOA)
			{
				if((SPI_Conf->SPI_CS_PIN==GPIO_Pin_13)||(SPI_Conf->SPI_CS_PIN==GPIO_Pin_14)||(SPI_Conf->SPI_CS_PIN==GPIO_Pin_15))
				{
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
					//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);			//关闭SW功能
					GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);		//关闭JTAG,SW功能开启
				}
				else
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOB)
			{
				if((SPI_Conf->SPI_CS_PIN==GPIO_Pin_3)||(SPI_Conf->SPI_CS_PIN==GPIO_Pin_4))
				{
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
					GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);				//关闭JTAG
				}
				else
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOC)
			{
				if((SPI_Conf->SPI_CS_PIN==GPIO_Pin_14)||(SPI_Conf->SPI_CS_PIN==GPIO_Pin_15))
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_AFIO, ENABLE);
				else
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOD)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOE)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOF)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOG)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
			}
			//SCK,MISO,MOSI配置
			GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
			GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			//CS配置
			GPIO_InitStructure.GPIO_Pin 	= SPI_Conf->SPI_CS_PIN;
			GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(SPI_Conf->SPI_CS_PORT, &GPIO_InitStructure);
		}
	}
	else if(SPI_Conf->SPIx==SPI3)
	{
		//PA15-NSS;PB3-SCK;PB4-MISO;PB5-MOSI;
		//2.2)**********打开SPI时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3 ,ENABLE);			
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
		
		if((SPI_Conf->SPI_CS_PORT==GPIOA)&&(SPI_Conf->SPI_CS_PIN==GPIO_Pin_15))			//如果SPI_NSS为SPI_NSS_Soft（软件控制方式）
		{
			SPI_Conf->SPI_Flash_NSS_CsFlg=1;		//如果使用纯硬件SPI3（含CS脚），SPI3_CsFlg=1，否则SPI3_CsFlg=0；
			
			GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;  		//复用推挽输出
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			
			//2.2)**********SPI_NSS配置		
			GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_15;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;  		//复用推挽输出
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else																										//如果SPI_NSS为SPI_NSS_Soft（软件控制方式）
		{
			SPI_Conf->SPI_Flash_NSS_CsFlg=0;		//如果使用纯硬件SPI3（含CS脚），UseSPI3_flg=1，否则UseSPI3_flg=0；
			//开CS-GPIO时钟
			if(SPI_Conf->SPI_CS_PORT==GPIOA)
			{
				if((SPI_Conf->SPI_CS_PIN==GPIO_Pin_13)||(SPI_Conf->SPI_CS_PIN==GPIO_Pin_14)||(SPI_Conf->SPI_CS_PIN==GPIO_Pin_15))
				{
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
					//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);			//关闭SW功能
					GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);		//关闭JTAG,SW功能开启
				}
				else
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOB)
			{
				if((SPI_Conf->SPI_CS_PIN==GPIO_Pin_3)||(SPI_Conf->SPI_CS_PIN==GPIO_Pin_4))
				{
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
					GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);				//关闭JTAG
				}
				else
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOC)
			{
				if((SPI_Conf->SPI_CS_PIN==GPIO_Pin_14)||(SPI_Conf->SPI_CS_PIN==GPIO_Pin_15))
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_AFIO, ENABLE);
				else
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOD)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOE)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOF)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
			}
			else if(SPI_Conf->SPI_CS_PORT==GPIOG)
			{
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
			}
			//SCK,MISO,MOSI配置
			GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
			GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			//CS配置
			GPIO_InitStructure.GPIO_Pin 	= SPI_Conf->SPI_CS_PIN;
			GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(SPI_Conf->SPI_CS_PORT, &GPIO_InitStructure);
		}			
	}
	//3)**********SPI配置选项
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;				//设置方向				（2线全双工、2线只接收、一线发送、一线接收）
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;															//模式         	（从或主设备）
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;													//宽度         	（8或16位）
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;																//时钟极性     	（低或高）
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;															//时钟相位     	（第一个或第二个跳变沿）
	if(SPI_Conf->SPI_Flash_NSS_CsFlg==1)																			//如果使用纯硬件SPI1（含CS脚），UseSPI1_flg=1，否则UseSPI1_flg=0；
	{
		SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;																//片选方式     	（硬件或软件方式）
	}
	else
	{
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;															//片选方式     	（硬件或软件方式）
	}	
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_Conf->SPI_BaudRatePrescaler_x;				//波特率预分频 	（从2---256分频）
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;												//最先发送的位 	（最低位，还是最高位在先）
	SPI_InitStructure.SPI_CRCPolynomial = 7;																	//设置crc多项式	（数字）如7
	SPI_Init(SPI_Conf->SPIx,&SPI_InitStructure);

	SPI_FLASH_DISALBE(SPI_Conf);				//关闭SPI接口，使用时再打开
	
	//3)**********使能SPIx_NESS为主输出模式
	if((SPI_Conf->SPIx->CR1&0X0200)!=SPI_NSS_Soft)						//如果在主机模式下的片选方式为硬件（SPI_NSS_Hard）方式，此处必须打开，否则NSS无信号
	{
		SPI_SSOutputCmd(SPI_Conf->SPIx, ENABLE);								//如果在主机模式下的片选方式为硬件（SPI_NSS_Hard）方式，此处必须打开，否则NSS无信号
	}
	else
	{
		SPI_SSOutputCmd(SPI_Conf->SPIx, DISABLE);								//如果在主机模式下的片选方式为硬件（SPI_NSS_Hard）方式，此处必须打开，否则NSS无信号
	}
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
void SPI_FLASH_ENALBE(SPI_FLASH_TypeDef *SPI_Conf)				//SPI_FLASH_使能
{
	if(SPI_Conf->SPI_Flash_NSS_CsFlg==1)														//如果使用纯硬件SPI1（含CS脚），UseSPI1_flg=1，否则UseSPI1_flg=0；
	{
		SPI_Cmd(SPI_Conf->SPIx, ENABLE);															//使能SPI--CS自动拉低
	}
	else
	{
		GPIO_ResetBits(SPI_Conf->SPI_CS_PORT, SPI_Conf->SPI_CS_PIN);	//CS_LOW片选使能
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		SPI_Cmd(SPI_Conf->SPIx, ENABLE);															//使能SPI
	}
	
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
void SPI_FLASH_DISALBE(SPI_FLASH_TypeDef *SPI_Conf)				//SPI_FLASH_关闭
{	
	if(SPI_Conf->SPI_Flash_NSS_CsFlg==1)														//如果使用纯硬件SPI1（含CS脚），UseSPI1_flg=1，否则UseSPI1_flg=0；
	{
		SPI_Cmd(SPI_Conf->SPIx, DISABLE);															//关闭SPI--CS自动拉高
	}
	else
	{
		SPI_Cmd(SPI_Conf->SPIx, DISABLE);															//关闭SPI
		__nop();
		__nop();
		__nop();
		__nop();
		__nop();
		GPIO_SetBits(SPI_Conf->SPI_CS_PORT, SPI_Conf->SPI_CS_PIN);		//CS_HIGH禁止片选
	}	
}
/*******************************************************************************
* Function Name  : SPI_FLASH_SendByte
* Description    : Sends a byte through the SPI interface and return the byte
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
u8 SPI_FLASH_WriteReadByte(SPI_FLASH_TypeDef *SPI_Conf,u8 byte)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI_Conf->SPIx, SPI_I2S_FLAG_TXE) == RESET);
  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(SPI_Conf->SPIx, byte);
  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI_Conf->SPIx, SPI_I2S_FLAG_RXNE) == RESET);
  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI_Conf->SPIx);
}
/*******************************************************************************
* Function Name  : SPI_FLASH_WriteEnable
* Description    : Enables the write access to the FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WriteEnable(SPI_FLASH_TypeDef *SPI_Conf)		//0x06
{
	u8 Status=0;
	SPI_FLASH_WaitForWriteEnd(SPI_Conf);			//等待FLASH写完成	
	
	SPI_FLASH_ENALBE(SPI_Conf);								//SPI_FLASH_使能
	SPI_Cmd(SPI_Conf->SPIx, ENABLE);					//使能SPI
	SPI_FLASH_WriteReadByte(SPI_Conf,Flash_WREN);	//0x06使能写命令
	SPI_FLASH_DISALBE(SPI_Conf);							//SPI_FLASH_关闭
	
	SPI_FLASH_ENALBE(SPI_Conf);								//SPI_FLASH_使能
	SPI_FLASH_WriteReadByte(SPI_Conf,Flash_WRSR);	//0x01写状态寄存器命令
	SPI_FLASH_WriteReadByte(SPI_Conf,0x00);				//使能写状态寄存器参数--清除写保护
	SPI_FLASH_DISALBE(SPI_Conf);							//SPI_FLASH_关闭
	
	SPI_FLASH_ENALBE(SPI_Conf);								//SPI_FLASH_使能
	SPI_Cmd(SPI_Conf->SPIx, ENABLE);					//使能SPI
	SPI_FLASH_WriteReadByte(SPI_Conf,Flash_WREN);	//0x06使能写命令
	SPI_FLASH_DISALBE(SPI_Conf);							//SPI_FLASH_关闭
	
	do																				//等待写允许
	{
		SPI_FLASH_ENALBE(SPI_Conf);													//SPI_FLASH_使能
		
		SPI_FLASH_WriteReadByte(SPI_Conf,Flash_RDSR);						//写入读状态命令 */		
		Status = SPI_FLASH_WriteReadByte(SPI_Conf,Dummy_Byte);	//读状态 */	
		
		SPI_FLASH_DISALBE(SPI_Conf);									//SPI_FLASH_关闭	
	}while((Status&WEL_Flag)!=WEL_Flag);
	
	SPI_FLASH_DISALBE(SPI_Conf);										//SPI_FLASH_关闭	
}
/*******************************************************************************
* Function Name  : SPI_FLASH_WriteDisable
* Description    : Enables the write access to the FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WriteDisable(SPI_FLASH_TypeDef *SPI_Conf)			//0x写保护--写禁止
{
	u32 Status=0x00;
	
	Status=SPI_FLASH_ReadStatus(SPI_Conf);			//读Flash状态寄存器
	Status=Status|0x1C;													//所有的保护
	
  SPI_FLASH_ENALBE(SPI_Conf);										//SPI_FLASH_使能
	
	SPI_FLASH_WriteReadByte(SPI_Conf,Flash_WRDI);	 	//发送写保护命令
	SPI_FLASH_WriteStatus(SPI_Conf,Status);			//写Flash状态寄存器
	
	SPI_FLASH_DISALBE(SPI_Conf);									//SPI_FLASH_关闭
	
}
/*******************************************************************************
* Function Name  : SPI_FLASH_WaitForWriteEnd
* Description    : Polls the status of the Write In Progress (WIP) flag in the
*                  FLASH's status  register  and  loop  until write  opertaion
*                  has completed.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WaitForWriteEnd(SPI_FLASH_TypeDef *SPI_Conf)	//等待FLASH写完成
{
  u8 FLASH_Status = 0;

  SPI_FLASH_ENALBE(SPI_Conf);										//SPI_FLASH_使能
	
  SPI_FLASH_WriteReadByte(SPI_Conf,Flash_RDSR);						//发送读状态寄存器命令
  do			//循环读取状态值，直到芯片退出忙状态
  {
    FLASH_Status = SPI_FLASH_WriteReadByte(SPI_Conf,Dummy_Byte);		//循环读取状态值，
  }
  while ((FLASH_Status & WIP_Flag) == WIP_Flag); 			

	SPI_FLASH_DISALBE(SPI_Conf);									//SPI_FLASH_关闭
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
u8 SPI_FLASH_ReadStatus(SPI_FLASH_TypeDef *SPI_Conf)
{
	u32 Status=0x00;

  SPI_FLASH_ENALBE(SPI_Conf);												//SPI_FLASH_使能
	
	SPI_FLASH_WriteReadByte(SPI_Conf,Flash_RDSR);					//0x05写入读状态命令
	Status = SPI_FLASH_WriteReadByte(SPI_Conf,Dummy_Byte);	//读取状态值
	
	SPI_FLASH_DISALBE(SPI_Conf);											//SPI_FLASH_关闭
	
	return Status;																		//返回状态值
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
void SPI_FLASH_WriteStatus(SPI_FLASH_TypeDef *SPI_Conf,u8 Status)		//写Flash状态寄存器
{
	SPI_FLASH_WriteEnable(SPI_Conf);					//0x06写使能
	
	SPI_FLASH_ENALBE(SPI_Conf);								//SPI_FLASH_使能
	
	SPI_FLASH_WriteReadByte(SPI_Conf,Flash_WRSR);	//0x01写状态寄存器命令
	SPI_FLASH_WriteReadByte(SPI_Conf,Status);			//写状态寄存器参数
	
	SPI_FLASH_DISALBE(SPI_Conf);							//SPI_FLASH_关闭	
}

/*******************************************************************************
* Function Name  : SPI_FLASH_ReadID
* Description    : Reads FLASH identification.
* Input          : None
* Output         : None
* Return         : FLASH identification
*******************************************************************************/
u32 SPI_FLASH_ReadID(SPI_FLASH_TypeDef *SPI_Conf)
{
  u32 Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
	
  SPI_FLASH_ENALBE(SPI_Conf);												//SPI_FLASH_使能
	
  SPI_FLASH_WriteReadByte(SPI_Conf,Flash_RDID);			//0x9F,读设备信息命令

  Temp0 = SPI_FLASH_WriteReadByte(SPI_Conf,Dummy_Byte);
  Temp1 = SPI_FLASH_WriteReadByte(SPI_Conf,Dummy_Byte);
  Temp2 = SPI_FLASH_WriteReadByte(SPI_Conf,Dummy_Byte);


	SPI_FLASH_DISALBE(SPI_Conf);											//SPI_FLASH_关闭	  

  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;	

  return Temp;
}
/*******************************************************************************
* Function Name  : SPI_FLASH_SectorErase
* Description    : Erases the specified FLASH sector.
* Input          : SectorAddr: address of the sector to erase.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_SectorErase(SPI_FLASH_TypeDef *SPI_Conf,u32 SectorAddr)
{	
  /* Send write enable instruction */
	SPI_FLASH_WaitForWriteEnd(SPI_Conf);		//等待FLASH写完成
  SPI_FLASH_WriteEnable(SPI_Conf);				//写使能
	
  SPI_FLASH_ENALBE(SPI_Conf);								//SPI_FLASH_使能
  /* Send Sector Erase instruction */
  SPI_FLASH_WriteReadByte(SPI_Conf,Flash_SE);
  /* Send SectorAddr high nibble address byte */
  SPI_FLASH_WriteReadByte(SPI_Conf,(SectorAddr & 0xFF0000) >> 16);
  /* Send SectorAddr medium nibble address byte */
  SPI_FLASH_WriteReadByte(SPI_Conf,(SectorAddr & 0xFF00) >> 8);
  /* Send SectorAddr low nibble address byte */
  SPI_FLASH_WriteReadByte(SPI_Conf,SectorAddr & 0xFF);
	
	SPI_FLASH_DISALBE(SPI_Conf);							//SPI_FLASH_关闭;
	
  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForWriteEnd(SPI_Conf);	
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BulkErase
* Description    : Erases the entire FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BulkErase(SPI_FLASH_TypeDef *SPI_Conf)
{
  SPI_FLASH_WriteEnable(SPI_Conf);					//FLASH写使能--等待空闲
	SPI_FLASH_WaitForWriteEnd(SPI_Conf);			//等待FLASH写完成--等待空闲
	
	SPI_FLASH_ENALBE(SPI_Conf);								//SPI_FLASH_使能
  SPI_FLASH_WriteReadByte(SPI_Conf,Flash_BE);		//写入块擦除命令
	SPI_FLASH_DISALBE(SPI_Conf);							//SPI_FLASH_关闭	

  SPI_FLASH_WaitForWriteEnd(SPI_Conf);//等待FLASH写完成--等待空闲	
}
/*******************************************************************************
* Function Name  : SPI_FLASH_BulkErase
* Description    : Erases the entire FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_ChipErase(SPI_FLASH_TypeDef *SPI_Conf)
{
  /* Send write enable instruction */
  SPI_FLASH_WriteEnable(SPI_Conf);
	/* 等待FLASH写完成*/
	SPI_FLASH_WaitForWriteEnd(SPI_Conf);		//等待FLASH写完成
	/* Chip Erase */
	/* Select the FLASH: Chip Select low */
  SPI_FLASH_ENALBE(SPI_Conf);												//SPI_FLASH_使能
	
  /* Send Bulk Erase instruction  */
  SPI_FLASH_WriteReadByte(SPI_Conf,Flash_CE);	//(unsigned char)0x60				//芯片擦除; 整片擦除; 也可以用0XC7	
	
	SPI_FLASH_DISALBE(SPI_Conf);											//SPI_FLASH_关闭
	
  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForWriteEnd(SPI_Conf);	
}































#endif
