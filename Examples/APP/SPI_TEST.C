#ifdef SPI_TEST
#include "SPI_TEST.H"



#define SPI_BufferSize 16
u8 SPI_Buffer[SPI_BufferSize]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A};
SPI_InitTypeDef  SPI_InitStructure;
u16 SPI1_NSS=0;
u16 SPI2_NSS=0;
/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void SPI_TEST_Configuration(void)
{
//	TIM_Configuration(TIM3,7200,500);						//定时时间设定
	PWM_Configuration(TIM2,7200,10000,50);
	SPI_StructConf(&SPI_InitStructure);
//	SPI_BASIC_Configuration(SPI1,&SPI_InitStructure);						//SPI配置
	
	SPI_DMA_Configuration(SPI1,&SPI_InitStructure,(u32*)SPI_Buffer,(u32*)SPI_Buffer,SPI_BUFFERSIZE);		//SPI_DMA配置
	SPI_DMA_Configuration(SPI2,&SPI_InitStructure,(u32*)SPI_Buffer,(u32*)SPI_Buffer,SPI_BUFFERSIZE);		//SPI_DMA配置

	if((SPI1->CR1&0X0200)==SPI_NSS_Soft)						//如果在主机模式下的片选方式为硬件（SPI_NSS_Hard）方式，此处必须打开，否则NSS无信号
	{
		SPI1_NSS=SPI_NSS_Soft;
	}
	else
	{
		SPI1_NSS=SPI_NSS_Hard;
	}
	if((SPI2->CR1&0X0200)==SPI_NSS_Soft)						//如果在主机模式下的片选方式为硬件（SPI_NSS_Hard）方式，此处必须打开，否则NSS无信号
	{
		SPI2_NSS=SPI_NSS_Soft;
	}
	else
	{
		SPI2_NSS=SPI_NSS_Hard;
	}
//	SPI1_NSS=SPI_NSS_Soft;

}

/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void SPI_TEST_Server(void)
{
	u16 num=0xFFFE;
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
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
		SPI_Cmd(SPI1, ENABLE);
//		SPI_DMAPrintf(SPI1,"%d",num);		//SPI_DMA发送函数----后边的省略号就是可变参数
		//SPI_DMAPrintf(SPI1,"%s",SPI_Buffer);		//SPI_DMA发送函数----后边的省略号就是可变参数
		SPI_DMA_BufferWrite(SPI1,(u32*)SPI_Buffer,10);		//SPI_DMA发送函数----后边的省略号就是可变参数
		while(SPI_TX_DMAFlagClear(SPI1)==ERROR);
		while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI_Cmd(SPI1, DISABLE);
		free(SPI_BUFFER);						//释放动态空间
		GPIO_SetBits(GPIOA,GPIO_Pin_4);
		
//		GPIO_ResetBits(GPIOB,GPIO_Pin_12);
		SPI_Cmd(SPI2, ENABLE);
//		SPI_DMAPrintf(SPI1,"%d",num);		//SPI_DMA发送函数----后边的省略号就是可变参数
		//SPI_DMAPrintf(SPI1,"%s",SPI_Buffer);		//SPI_DMA发送函数----后边的省略号就是可变参数
		SPI_DMA_BufferWrite(SPI2,(u32*)SPI_Buffer,10);		//SPI_DMA发送函数----后边的省略号就是可变参数
		while(SPI_TX_DMAFlagClear(SPI2)==ERROR);
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		SPI_Cmd(SPI2, DISABLE);
		free(SPI_BUFFER);						//释放动态空间
//		GPIO_SetBits(GPIOB,GPIO_Pin_12);


//	free(SPI_BUFFER);						//释放动态空间
}
/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void SPI_StructConf(SPI_InitTypeDef* SPI_InitStruct)
{
//	SPI4->GPIO_NSS	=	GPIOA;
//	SPI4->SPI_NSSP	=	GPIO_Pin_4;
//	SPI4->GPIO_SCK	=	GPIOA;
//	SPI4->SPI_SCKP	=	GPIO_Pin_5;
//	SPI4->GPIO_MISO	=	GPIOA;
//	SPI4->SPI_MISOP	=	GPIO_Pin_6;
//	SPI4->GPIO_MOSI	=	GPIOA;
//	SPI4->SPI_MOSIP	=	GPIO_Pin_71;	
	
	SPI_InitStruct->SPI_Direction=SPI_Direction_2Lines_FullDuplex;			//设置方向				（2线全双工、2线只接收、一线发送、一线接收）
	SPI_InitStruct->SPI_Mode=SPI_Mode_Master;														//模式         	（从或主设备）
	SPI_InitStruct->SPI_DataSize=SPI_DataSize_8b;												//宽度         	（8或16位）
	SPI_InitStruct->SPI_CPOL=SPI_CPOL_High;															//时钟极性     	（低或高）
	SPI_InitStruct->SPI_CPHA=SPI_CPHA_2Edge;														//时钟相位     	（第一个或第二个跳变沿）
//	SPI_InitStruct->SPI_NSS=SPI_NSS_Soft;																//片选方式     	（硬件或软件方式）
	SPI_InitStruct->SPI_NSS=SPI_NSS_Hard;																//片选方式     	（硬件或软件方式）
	SPI_InitStruct->SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_16;			//波特率预分频 	（从2---256分频）
	SPI_InitStruct->SPI_FirstBit=SPI_FirstBit_MSB;											//最先发送的位 	（最低位，还是最高位在先）
	SPI_InitStruct->SPI_CRCPolynomial=0X07;															//设置crc多项式	（其复位值为0x0007，根据应用可以设置其他数值。）
}




#endif
