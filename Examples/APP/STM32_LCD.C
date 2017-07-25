#ifdef STM32_LCD

#include "STM32_LCD.H"

#include "string.h"				//串和内存操作函数头文件
#include "stm32f10x_dma.h"




#include "STM32F10x_BitBand.H"
#include "STM32_GPIO.H"
#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"
#include "STM32_WDG.H"
#include "STM32_PWM.H"
#include "STM32_PWM.H"
#include "STM32_GPIO.H"
#include "STM32_USART.H"
#include "STM32_DMA.H"


#include "TM1618.H"
#include "SSD1963.H"
#include "STM32_SDCard.H"
#include "GT32L32M0180.H"

//#define SDCardTest
//#define GT32L32M0180Test	
//


GT32L32_Init_TypeDef 	GT32L32_Init;
GT32L32_Info_TypeDef	GT32L32_Info;
SSD1963_Pindef 				SSD1963_Pinfo;
TM1618_Pindef					TM1618_1,TM1618_2;

u16 millisecond=0;
u8 hour=19,min=55,second=00;



u32	dspdata=0;
u16 us=0;
u16	mm=0;
u8	ss=0;
u8	hh=0;
u8 GTBuffer[512]={0};

void GT32L32_PinSet(void);
	
//=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>
//->函数名		:	
//->功能描述	:	 
//->输入		:	
//->输出		:
//->返回 		:
//<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=
void STM32_LCD_Configuration(void)
{	
	SYS_Configuration();					//系统配置---打开系统时钟 STM32_SYS.H	
	GPIO_DeInitAll();							//将所有的GPIO关闭----V20170605	
	STM32_LCD_PinConf();
	
	SSD1963_Configuration();					//LCD接口配置
//	SSD1963_PinConf(&SSD1963_Pinfo);
//	GT32L32_Configuration();
	SSD1963_PrintfString(0,16,"SD_TYPE IS SD_TYPE_V2HC");		//后边的省略号就是可变参数
//	LCD_PrintfStringGT(0,32,32,"后边的省略号就是可变参数~!@#$%^&*()_+{}:<>?|");		//后边的省略号就是可变参数
	LCD_PrintfStringGT(10,120,24,"我后边的省略号就是可变参数啊");							//后边的省略号就是可变参数
	LCD_PrintfStringGT(10,144,24,"我后边的省略号就是可变参数啊");							//后边的省略号就是可变参数
	LCD_PrintfStringGT(10,168,32,"我后边的省略号就是可变参数啊");							//后边的省略号就是可变参数
//	LCD_PrintfStringGT(10,120,32,"A");							//后边的省略号就是可变参数
//	LCD_PrintfStringGT(0,260,32,"1234567890SD_TYPE IS SD_TYPE_V2HC54");							//后边的省略号就是可变参数
	LCD_PrintfStringGT(0,260,32,"TYPE");							//后边的省略号就是可变参数
	SSD1963_DrawCircle(400,240,100);
//	SSD1963_PrintfString(0,300,"SD_TYPE IS SD_TYPE_V2HC");		//后边的省略号就是可变参数
//	LCD_PrintfStringGT(300,230,32,"%02d-%02d-%02d-%03d",hour,min,second,millisecond);			//后边的省略号就是可变参数
//	LCD_PrintfStringGT(0,256,32,"运行计时：");		//后边的省略号就是可变参数
//	LCD_PrintfStringGT(300,256,32,"%02d:%02d:%02d:%02d",hour,min,second,millisecond);		//后边的省略号就是可变参数
//	TM1618_PinSet();
	
	SysTick_Configuration(1000);											//系统嘀嗒时钟配置72MHz,单位为uS
	
	IWDG_Configuration(1000);													//独立看门狗配置---参数单位ms
	
	PWM_OUT(TIM2,PWM_OUTChannel1,5000,10);						//PWM设定-20161127版本
	
	SSD1963_DrawRectangle(10,10,790,470);
	
	SSD1963_DrawLine(30, 50,500, 50);						//画直线
	
}
//=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>=>
//->函数名		:	
//->功能描述	:	 
//->输入		:	
//->输出		:
//->返回 		:
//<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=<=
void STM32_LCD_Server(void)
{
	u16 delat=1000;
	IWDG_Feed();								//独立看门狗喂狗
	
	millisecond++;
	if(millisecond>999)
	{
		millisecond=0;
		second++;
		if(second>59)
		{
			second=0;
			min++;			
			if(min>59)
			{
				min=0;
				hour++;
				if(hour>23)
				{
					hour=0;
				}
				LCD_PrintfStringGT(300,230,32,"%02d",hour);		//后边的省略号就是可变参数
				
			}
			LCD_PrintfStringGT(348,230,32,"%02d",min);		//后边的省略号就是可变参数
		}
		LCD_PrintfStringGT(396,230,32,"%02d",second);		//后边的省略号就是可变参数
		while(delat--);
//		LCD_PrintfStringGT(50,400,24,"%02d-秒",second);		//后边的省略号就是可变参数
//		TM1618_DIS();
	}
	TM1618_DIS();
//	SSD1963_DrawRectangle(10,10,790,470);
//	LCD_PrintfStringGT(444,230,32,"%03d",millisecond);		//后边的省略号就是可变参数
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
void STM32_LCD_PinConf(void)
{
	TM1618_PinSet();
	SSD1963_PinSet();
	GT32L32_PinSet();
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
void TM1618_PinSet(void)
{
	TM1618_1.TM1618_CLK_PORT	=GPIOC;
	TM1618_1.TM1618_CLK_Pin		=GPIO_Pin_8;
	
	TM1618_1.TM1618_DIO_PORT	=GPIOC;
	TM1618_1.TM1618_DIO_Pin		=GPIO_Pin_9;	
	
	TM1618_1.TM1618_STB_PORT	=GPIOC;
	TM1618_1.TM1618_STB_Pin		=GPIO_Pin_10;
	
	TM1618_2.TM1618_CLK_PORT	=GPIOC;
	TM1618_2.TM1618_CLK_Pin		=GPIO_Pin_8;
	
	TM1618_2.TM1618_DIO_PORT	=GPIOC;
	TM1618_2.TM1618_DIO_Pin		=GPIO_Pin_9;	
	
	TM1618_2.TM1618_STB_PORT	=GPIOC;
	TM1618_2.TM1618_STB_Pin		=GPIO_Pin_11;
	
	TM1618_PinConf(&TM1618_1);
	TM1618_PinConf(&TM1618_2);
	
	GPIO_Configuration_OPP50	(GPIOC,	GPIO_Pin_12);			//将GPIO相应管脚配置为PP(推挽)输出模式，最大速度50MHz----V20170605
	
	PC12=1;
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
void SSD1963_PinSet(void)
{
	SSD1963_Pinfo.SSD1963_sBACKLIGHT_PORT	=	GPIOC;
	SSD1963_Pinfo.SSD1963_sBACKLIGHT_Pin	=	GPIO_Pin_0;
	
	SSD1963_Pinfo.SSD1963_sRD_PORT				=	GPIOC;
	SSD1963_Pinfo.SSD1963_sRD_Pin					=	GPIO_Pin_1;	
	
	SSD1963_Pinfo.SSD1963_sREST_PORT			=	GPIOC;
	SSD1963_Pinfo.SSD1963_sREST_Pin				=	GPIO_Pin_2;
	
	SSD1963_Pinfo.SSD1963_sDC_PORT				=	GPIOC;
	SSD1963_Pinfo.SSD1963_sDC_Pin					=	GPIO_Pin_3;
	
	SSD1963_Pinfo.SSD1963_sWR_PORT				=	GPIOC;
	SSD1963_Pinfo.SSD1963_sWR_Pin					=	GPIO_Pin_4;	
	
	SSD1963_Pinfo.SSD1963_sCS_PORT				=	GPIOC;
	SSD1963_Pinfo.SSD1963_sCS_Pin					=	GPIO_Pin_5;
	
	SSD1963_Pinfo.SSD1963_sDATABUS_PORT		=	GPIOE;
	SSD1963_Pinfo.SSD1963_sDATABUS_Pin		=	GPIO_Pin_All;
	
	SSD1963_PinConf(&SSD1963_Pinfo);
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
void GT32L32_PinSet(void)
{
	GT32L32_Init.sSPIx=SPI1;
	GT32L32_Init.sGT32L32_CS_PORT=GPIOB;
	GT32L32_Init.sGT32L32_CS_PIN=GPIO_Pin_14;
	GT32L32_Init.SPI_BaudRatePrescaler_x=SPI_BaudRatePrescaler_128;
	
	GT32L32_ConfigurationNR(&GT32L32_Init);				//普通SPI通讯方式配置
//	GT32L32_PinConf(&GT32L32_Init);
}
/*******************************************************************************
*函数名			:	function
*功能描述		:	函数功能说明
*输入				: 
*返回值			:	无
*******************************************************************************/
void TM1618_DIS(void)
{
	if(us++>=1000)
	{
		us=0;
		mm++;
		if(mm>=60)
		{
			mm=0;
			ss++;
		}
		if(ss>=60)
		{
			ss=0;
			hh++;
		}
		if(hh>=24)
		{
			hh=0;
		}
		
		dspdata=(u32)mm+(u32)ss*100+(u32)hh*10000;
		TM1618_WriteDataN(&TM1618_1,dspdata/10000);
		TM1618_WriteDataN(&TM1618_2,dspdata%10000);	
	}
}
/*******************************************************************************
*函数名		:	LCD_ShowString
*功能描述	:	显示字符串高通字库
*输入			: x,y:起点坐标
						*p:字符串起始地址
						用16字体
*输出			:	无
*返回值		:	无
*例程			:
*******************************************************************************/
unsigned int LCD_PrintfStringGT(u16 x,u16 y,u8 font,const char *format,...)				//后边的省略号就是可变参数
{ 	
//		va_list ap; 										//VA_LIST 是在C语言中解决变参问题的一组宏，所在头文件：#include <stdarg.h>,用于获取不确定个数的参数
//		static char string[ 256 ];			//定义数组，
//    	va_start( ap, format );
//    	vsprintf( string , format, ap );    
//		va_end( ap );
	
	char	*Char_Buffer;

	//1)**********获取数据宽度
	u16 num=strlen((const char*)format);		//获取数据宽度
	//2)**********定义缓冲区大小变量
	unsigned int BufferSize;
	//3)**********args为定义的一个指向可变参数的变量，va_list以及下边要用到的va_start,va_end都是是在定义，可变参数函数中必须要用到宏， 在stdarg.h头文件中定义
	va_list args;                                        
	//4)**********申请动态空间
	Char_Buffer = (char*)malloc(sizeof(char) * num);
	if(Char_Buffer==NULL)
	{
		Char_Buffer=NULL;
		return 0;
	}
	//5)**********初始化args的函数，使其指向可变参数的第一个参数，format是可变参数的前一个参数
	va_start(args, format);
	//6)**********正常情况下返回生成字串的长度(除去\0),错误情况返回负值
	BufferSize = vsprintf(Char_Buffer, format, args);
	num=BufferSize;
	//7)**********结束可变参数的获取
	va_end(args);                                      		
	//8)**********将等发送缓冲区大小（数据个数）及缓冲区地址发给DMA开启发送
//	while(*Char_Buffer!='\0')
	while(num>0)
	{ 
		unsigned char	*char_GT32L32;
		unsigned char dst=*Char_Buffer;
		
		if(dst>0x80)		//双字节--汉字
		{
			u32 Address=0;	//汉字点阵的字节地址(Address)
			u32 lengh=0;			//汉字点阵的数据长度
			u16 word=dst<<8;
			Char_Buffer++;
			dst=*Char_Buffer;
			word=word|dst;
			
			//显示超限判断
			if(x>LCD_W-16)
			{
				x=0;
				y+=32;
			}
			if(y>LCD_H-32)
			{
				y=x=0;
				SSD1963_Clear(SSD1963_RED);
			}
			//字体大小判断
//			if(font==12)
//			{
//				GT32L32_Info.GT32L32_BaseAddr=GB18030_BaseAddr_hz12x12;
//			}
//			else if(font==16)
//			{
//				GT32L32_Info.GT32L32_BaseAddr=GB18030_BaseAddr_hz16x16;
//			}
//			else if(font==24)
//			{
//				GT32L32_Info.GT32L32_BaseAddr=GB18030_BaseAddr_hz24x24;
//			}
//			else if(font==32)
//			{
//				GT32L32_Info.GT32L32_BaseAddr=GB18030_BaseAddr_hz32x32;
//			}
			
//			GT32L32_Get_Info(word,&GT32L32_Info);		//获取字符地址和点阵长度
			
//			char_GT32L32 = (unsigned char*)malloc(GT32L32_Info.GT32L32_BufferSize);
//			
//			GT32L32_ReadBuffer(&GT32L32_Init,GT32L32_Info.GT32L32_Address,GT32L32_Info.GT32L32_BufferSize,GTBuffer);	//从字库中读数据函数
//			SSD1963_ShowCharGT(x,y,font,GT32L32_Info.GT32L32_BufferSize,GTBuffer);
				//获取汉字点阵地址
				Address	=	GT32L32_GetAddress(font, word>>8, word, 0, 0);//获取地址
				//获取汉字点阵数据长度
				lengh= GT32L32_GetBufferLen(font, word>>8, word, 0, 0);//获取长度
				//获取点阵数据
				GT32L32_ReadBuffer(&GT32L32_Init,Address,lengh,GTBuffer);	//从字库中读数据函数
				SSD1963_ShowCharGT(x,y,font,lengh,GTBuffer);
				
			if(font==12)
			{
				x+=12;
			}
			else if(font==16)
			{
				x+=16;
			}
			else if(font==24)
			{
				x+=24;
			}
			else if(font==32)
			{
				x+=32;
			}
			Char_Buffer++;
			num--;num--;		//双字节，减两次
						
//			free(char_GT32L32);		//发送完成后注意应该释放缓冲区：free(Char_Buffer); 
		}
		else		//单字节
		{
			u32 Address=0;	//汉字点阵的字节地址(Address)
			u32 lengh=0;			//汉字点阵的数据长度
			
			if(x>LCD_W-16)
			{
				x=0;
				y+=32;
			}
			if(y>LCD_H-32)
			{
				y=x=0;
				SSD1963_Clear(SSD1963_RED);
			}
		
			if(font==12)
			{
				GT32L32_Info.GT32L32_BaseAddr=ASCII_BaseAddr_zf6x12;		//(u32)0x100900,			//6x12点阵ASCII字符
			}
			else if(font==16)
			{
				GT32L32_Info.GT32L32_BaseAddr=ASCII_BaseAddr_zfs8x16;		//(u32)0x100D80,			//8x16点阵ASCII标准字符
			}
			else if(font==24)
			{
				GT32L32_Info.GT32L32_BaseAddr=ASCII_BaseAddr_zf12x24;			//(u32)0x101B80,			//12x24点阵ASCII标准字符
			}
			else if(font==32)
			{
				GT32L32_Info.GT32L32_BaseAddr=ASCII_BaseAddr_zfs16x32;		//(u32)0x102D80,			//16x32点阵ASCII标准字符
			}
			
			
			
//			GT32L32_Get_Info(dst,&GT32L32_Info);	//获取字符地址和点阵长度			
//			char_GT32L32 = (unsigned char*)malloc(GT32L32_Info.GT32L32_BufferSize);			
//			GT32L32_ReadBuffer(&GT32L32_Init,GT32L32_Info.GT32L32_Address,GT32L32_Info.GT32L32_BufferSize,GTBuffer);	//从字库中读数据函数
//			SSD1963_ShowCharGT(x,y,font,GT32L32_Info.GT32L32_BufferSize,GTBuffer);
//			free(char_GT32L32);		//发送完成后注意应该释放缓冲区：free(Char_Buffer);
			
			//获取汉字点阵地址
			Address	=	GT32L32_GetAddress(font, dst, 0, 0, 0);//获取地址
			//获取汉字点阵数据长度
			lengh= GT32L32_GetBufferLen(font, dst, 0, 0, 0);//获取长度
			//获取点阵数据
			GT32L32_ReadBuffer(&GT32L32_Init,Address,lengh,GTBuffer);	//从字库中读数据函数
			SSD1963_ShowCharGT(x,y,font,lengh,GTBuffer);
			
			if(font==12)
			{
				x+=6;
			}
			else if(font==16)
			{
				x+=8;
			}
			else if(font==24)
			{
				x+=12;
			}
			else if(font==32)
			{
				x+=16;
			}
			
			Char_Buffer++;
			num--;
//			free(char_GT32L32);		//发送完成后注意应该释放缓冲区：free(Char_Buffer);
		}
		
//		LCD_ShowCharGT(x,y,1,STM32_DSP_Buffer);
//		GT32L32_Get_Info(0xB0A1,&GT32L32_Info);
//		GT32L32_ReadBuffer(u32 address,u16 len,unsigned char *Buffer);	//从字库中读数据函数
//		LCD_ShowCharGT(x,y,1,STM32_DSP_Buffer);
//		LCD_ShowCharGT(664,300,1,STM32_DSP_Buffer);
//		GT32L32_GetGB18030_16(0xb0,0xa1,0,0,Char_Buffer);
//		LCD_ShowCharGT(x,y,*Char_Buffer,0);		
	}
	//9)**********DMA发送完成后注意应该释放缓冲区：free(USART_BUFFER);
	Char_Buffer-=BufferSize;
//	free(Char_Buffer);		//发送完成后注意应该释放缓冲区：free(Char_Buffer); 
	return BufferSize;
}

#endif
