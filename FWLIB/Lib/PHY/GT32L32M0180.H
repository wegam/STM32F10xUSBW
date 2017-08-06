/******************************** User_library *********************************
* �ļ��� 	: STM32_GT32L32M0180.H
* ����   	: wegam@sina.com
* �汾   	: V
* ����   	: 2017/01/15
* ˵��   	: 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#ifndef __GT32L32M0180_H
#define __GT32L32M0180_H

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_nvic.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_dma.h"


//#include "STM32_WOW.H"		//ʹ��λ��

#include	"stdio.h"				//����printf
#include	"string.h"			//����printf
#include	"stdarg.h"			//���ڻ�ȡ��ȷ�������Ĳ���
#include	"stdlib.h"			//malloc��̬�����ڴ�ռ�


/* Private define ------------------------------------------------------------*/
//____________��Ŀ����
#define	STM32_GT32L32M0180
//#define	GT32L32_BITBAND				//λ��
#define	GT32L32_SPIPORT_EN		//ʹ��SPI�ӿ�
#define	STM32_LCD_GT32L32M0180
#define	HC_PL010V11			//LCD���ذ�


//____________SPI�ٶȶ���
#define GT32L32_CLK_SpeedL	0
#define GT32L32_CLK_SpeedH	1

//#ifdef STM32_LCD_GT32L32M0180

	//____________�˿ڶ���
//	#define GT32L32_SPI_PORT			SPI1

//	#define GT32L32_SPI_Periph		RCC_APB2Periph_SPI1
//	//____________SPI_CS����
//	#define GT32L32_CS_PORT				GPIOB
//	#define GT32L32_CS_PIN				GPIO_Pin_14
//	#define GT32L32_CS_Periph			RCC_APB2Periph_GPIOB
//	//____________SPI_CLK����
//	#define GT32L32_CLK_PORT			GPIOA
//	#define GT32L32_CLK_PIN				GPIO_Pin_5
//	#define GT32L32_CLK_Periph		RCC_APB2Periph_GPIOA
//	//____________SPI_MISO����
//	#define GT32L32_MISO_PORT			GPIOA
//	#define GT32L32_MISO_PIN			GPIO_Pin_6
//	#define GT32L32_MISO_Periph		RCC_APB2Periph_GPIOA
//	//____________SPI_MOSI����
//	#define GT32L32_MOSI_PORT			GPIOA
//	#define GT32L32_MOSI_PIN			GPIO_Pin_7
//	#define GT32L32_MOSI_Periph		RCC_APB2Periph_GPIOA

	//____________GPIO���ƶ���


		
//		//____________SPI_CS���ƶ���
//		#define GT32L32_CS_DisSelect				GPIO_SetBits(GT32L32_CS_PORT,				GT32L32_CS_PIN)			//ȡ��Ƭѡ
//		#define GT32L32_CS_Select						GPIO_ResetBits(GT32L32_CS_PORT,			GT32L32_CS_PIN)		//ʹ��Ƭѡ
//	//____________SPI_CLK/SCLK���ƶ���
//		#define GT32L32_CLK_L								GPIO_ResetBits(GT32L32_CLK_PORT,	GT32L32_CLK_PIN)		//�͵�ƽ
//		#define GT32L32_CLK_H								GPIO_SetBits(GT32L32_CLK_PORT,		GT32L32_CLK_PIN)			//�ߵ�ƽ
//	//____________SPI_MISO/SO���ƶ���
//		#define GT32L32_SO									GPIO_ReadOutputDataBit(GT32L32_MISO_PORT,GT32L32_MISO_PIN)
//	//____________SPI_MOSI/SI���ƶ���
//		#define GT32L32_SI_L								GPIO_ResetBits(GT32L32_MOSI_PORT,	GT32L32_MOSI_PIN)		//�͵�ƽ
//		#define GT32L32_SI_H								GPIO_SetBits(GT32L32_MOSI_PORT,		GT32L32_MOSI_PIN)			//�ߵ�ƽ



//#else
//	//____________�˿ڶ���
//	#define GT32L32_SPI_PORT			SPI1
//	#define GT32L32_SPI_Periph		RCC_APB2Periph_SPI1
//	//____________SPI_CS����
//	#define GT32L32_CS_PORT				GPIOA
//	#define GT32L32_CS_PIN				GPIO_Pin_4
//	#define GT32L32_CS_Periph			RCC_APB2Periph_GPIOB
//	//____________SPI_CLK����
//	#define GT32L32_CLK_PORT			GPIOA
//	#define GT32L32_CLK_PIN				GPIO_Pin_5
//	#define GT32L32_CLK_Periph		RCC_APB2Periph_GPIOA
//	//____________SPI_MISO����
//	#define GT32L32_MISO_PORT			GPIOA
//	#define GT32L32_MISO_PIN			GPIO_Pin_6
//	#define GT32L32_MISO_Periph		RCC_APB2Periph_GPIOA
//	//____________SPI_MOSI����
//	#define GT32L32_MOSI_PORT			GPIOA
//	#define GT32L32_MOSI_PIN			GPIO_Pin_7
//	#define GT32L32_MOSI_Periph		RCC_APB2Periph_GPIOA

//	//____________GPIO���ƶ���
//	#ifdef		GT32L32_BITBAND		//*************λ��
//		//____________SPI_CS���ƶ���
//			#define GT32L32_CS_DisSelect				PA4=1			//ȡ��Ƭѡ
//			#define GT32L32_CS_Select						PA4=0			//Ƭѡ
//		//____________SPI_CLK/SCLK���ƶ���
//			#define GT32L32_CLK_L								PA5=0			//�͵�ƽ
//			#define GT32L32_CLK_H								PA5=1			//�ߵ�ƽ
//		//____________SPI_MISO/SO���ƶ���
//			#define GT32L32_SO									PA6in
//		//____________SPI_MOSI/SI���ƶ���
//			#define GT32L32_SI_L								PA7=0			//�͵�ƽ
//			#define GT32L32_SI_H								PA7=1			//�ߵ�ƽ
//	#else		//*************δ����λ��
//		//____________SPI_CS���ƶ���
//			#define GT32L32_CS_DisSelect				GPIO_SetBits(GT32L32_CS_PORT,				GT32L32_CS_PIN)			//ȡ��Ƭѡ
//			#define GT32L32_CS_Select						GPIO_ResetBits(GT32L32_CS_PORT,			GT32L32_CS_PIN)		//ʹ��Ƭѡ
//		//____________SPI_CLK/SCLK���ƶ���
//			#define GT32L32_CLK_L								GPIO_ResetBits(GT32L32_CLK_PORT,	GT32L32_CLK_PIN)		//�͵�ƽ
//			#define GT32L32_CLK_H								GPIO_SetBits(GT32L32_CLK_PORT,		GT32L32_CLK_PIN)			//�ߵ�ƽ
//		//____________SPI_MISO/SO���ƶ���
//			#define GT32L32_SO									GPIO_ReadOutputDataBit(GT32L32_MISO_PORT,GT32L32_MISO_PIN)
//		//____________SPI_MOSI/SI���ƶ���
//			#define GT32L32_SI_L								GPIO_ResetBits(GT32L32_MOSI_PORT,	GT32L32_MOSI_PIN)		//�͵�ƽ
//			#define GT32L32_SI_H								GPIO_SetBits(GT32L32_MOSI_PORT,		GT32L32_MOSI_PIN)			//�ߵ�ƽ
//	#endif	
//#endif

/* Private typedef -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
//----GPIO�ܽ�����--------------------------------------------------------------
typedef struct	_GT32L32_Pin
{
	//____________�˿ڶ���
	SPI_TypeDef *			sSPIx;				//SPI�ӿ����� x=0��1��2��3��

	//____________SPI_CS����
	GPIO_TypeDef* 	sGT32L32_CS_PORT;
	unsigned short 	sGT32L32_CS_PIN;

	//____________SPI_CLK����
	GPIO_TypeDef* 	sGT32L32_CLK_PORT;
	unsigned short 	sGT32L32_CLK_PIN;

	//____________SPI_MISO����
	GPIO_TypeDef* 	sGT32L32_MISO_PORT;
	unsigned short 	sGT32L32_MISO_PIN;

	//____________SPI_MOSI����
	GPIO_TypeDef* 	sGT32L32_MOSI_PORT;
	unsigned short 	sGT32L32_MOSI_PIN;
	
	//____________SPI_������
	unsigned short 	SPI_BaudRatePrescaler_x;		//x=2/4/8/16/32/64/128/256
	
}GT32L32_Init_TypeDef;

//----�ַ������ֿ����ַ��------------------------------------------------------------
typedef enum
{
	ASCII_BaseAddr_None					=	(u32)0x000000,			//δ���õ�ַ
	
	ASCII_BaseAddr_zf5x7				=	(u32)0x100000,			//5x7����ASCII��׼�ַ�
	ASCII_BaseAddr_zfs7x8				=	(u32)0x100300,			//7x8����ASCII��׼�ַ�
	ASCII_BaseAddr_zfm7x8				=	(u32)0x100600,			//7x8����ASCII�����ַ�
	ASCII_BaseAddr_zf6x12				=	(u32)0x100900,			//6x12����ASCII�ַ�
	ASCII_BaseAddr_zfs8x16			=	(u32)0x100D80,			//8x16����ASCII��׼�ַ�
	ASCII_BaseAddr_zfm8x16			=	(u32)0x101580,			//8x16����ASCII�����ַ�
	ASCII_BaseAddr_zf12x24			=	(u32)0x101B80,			//12x24����ASCII��׼�ַ�
	ASCII_BaseAddr_zfs16x32			=	(u32)0x102D80,			//16x32����ASCII��׼�ַ�
	ASCII_BaseAddr_zfm16x32			=	(u32)0x104580,			//16x32����ASCII�����ַ�
	
	ASCII_BaseAddr_ar12					=	(u32)0x105D80,			//12���󲻵ȿ�ASCII��ͷ��Arial���ַ�
	ASCII_BaseAddr_ar16					=	(u32)0x106740,			//16���󲻵ȿ�ASCII��ͷ��Arial���ַ�
	ASCII_BaseAddr_ar24					=	(u32)0x107400,			//24���󲻵ȿ�ASCII��ͷ��Arial���ַ�
	ASCII_BaseAddr_ar32					=	(u32)0x108FC0,			//32���󲻵ȿ�ASCII��ͷ��Arial���ַ�
	
	ASCII_BaseAddr_TNR12				=	(u32)0x10C080,			//12���󲻵ȿ�ASCII������Times New Roman���ַ�
	ASCII_BaseAddr_TNR16				=	(u32)0x10CA50,			//16���󲻵ȿ�ASCII������Times New Roman���ַ�
	ASCII_BaseAddr_TNR24				=	(u32)0x10D740,			//24���󲻵ȿ�ASCII������Times New Roman���ַ�
	ASCII_BaseAddr_TNR32				=	(u32)0x10F340,			//32���󲻵ȿ�ASCII������Times New Roman���ַ�
	
	Digital_BaseAddr_sz14x28		=	(u32)0x112400,			//14x28���ַ����ַ�
	Digital_BaseAddr_sz20x40		=	(u32)0x112748,			//20x40���ַ����ַ�
	Digital_BaseAddr_sz28				=	(u32)0x112CE8,			//28���󲻵ȿ����ַ����ַ�
	Digital_BaseAddr_sz40				=	(u32)0x113396,			//40���󲻵ȿ����ַ����ַ�
	
	GB18030_BaseAddr_zf12x12		=	(u32)0x113D0E,			//12x12����GB18030�ַ�
	GB18030_BaseAddr_zf16x16		=	(u32)0x194FDE,			//16x16����GB18030�ַ�
	GB18030_BaseAddr_zf24x24		=	(u32)0x2743DE,			//24x24����GB18030�ַ�
	GB18030_BaseAddr_zf32x32		=	(u32)0x47AE10,			//32x32����GB18030�ַ�
	
	GB18030_BaseAddr_hz12x12		=	(u32)0x113D0E,			//12x12����GB18030����
	GB18030_BaseAddr_hz16x16		=	(u32)0x194FDE,			//16x16����GB18030����
	GB18030_BaseAddr_hz24x24		=	(u32)0x2743DE,			//24x24����GB18030����
	GB18030_BaseAddr_hz32x32		=	(u32)0x47AE10,			//32x32����GB18030����
	
	Unicode_To_GBK_BaseAddr			=	(u32)0x46A90E,			//Unicode->GBKת���
	BIG5_To_GBK_BaseAddr				=	(u32)0x7F1E10,			//BIG5->GBKת���
	
	BarCode_BaseAddr_12x27			=	(u32)0x478FD2,			//12x27�������ַ� EAN13
	BarCode_BaseAddr_16x20			=	(u32)0x479C7A,			//16x20�������ַ� CODE128
	
	AntennaCode_BaseAddr_12X12	=(u32)0x47AD32,				//12x12���߷���
	BatteryCode_BaseAddr_12X12	=(u32)0x47ADAA,				//12x12��ط���
	
	Reserved_BaseAddr_LSB				=(u32)0x7F8760,				//�������͵�ַ
	Reserved_BaseAddr_MSB				=(u32)0x7FFFFF,				//�������ߵ�ַ		
}	GT32L32_BaseAddr_TypeDef;		//�ַ������ֿ����ַ��
//----�����б�------------------------------------------------------------
typedef enum
{
	GT32L32_Error_None					=	(u8)0x00,			//�޴���
	GT32L32_Error_Addr					=	(u8)0x01,			//��ַ����
	GT32L32_Error_ASCIICode			=	(u8)0x02,			//ASCII�����
	GT32L32_Error_HzEmp					=	(u8)0x03,			//���������8λΪ0x7F,Ϊ������
	
}	GT32L32_Error_TypeDef;		//��������б�
//----��Ϣ�ṹ��------------------------------------------------------------
typedef struct      /*Card Identification Data*/
{
	GT32L32_BaseAddr_TypeDef	GT32L32_BaseAddr;		//�ַ������ֿ����ַ��
	GT32L32_Error_TypeDef			GT32L32_Error;			//�����б�
	u32	GT32L32_Address;													//�ַ���ַ
	u8	GT32L32_BufferSize;												//�������ݳ���
} GT32L32_Info_TypeDef;
/* Exported macro ------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */

void 	GT32L32_ConfigurationNR(GT32L32_Init_TypeDef *GT32L32_Init);				//��ͨSPIͨѶ��ʽ����
void 	GT32L32_DMA_ConfigurationNR(GT32L32_Init_TypeDef *GT32L32_Init);		//SPI-DMAͨѶ��ʽ����---20170722δ���
u8 GT32L32_ReadStatus(GT32L32_Init_TypeDef *GT32L32_Init);								//��ȡоƬ״̬


void 	GT32L32_PinConf(GT32L32_Init_TypeDef *GT32L32_Init);
void	GT32L32_Server(void);


u8  GT32L32_GetDig_Info(u8 Sequence,GT32L32_Info_TypeDef *GT32L32_Info);			//�������ַ����ַ�
u8  GT32L32_GetASCII_Info(u8 ASCIICode,GT32L32_Info_TypeDef *GT32L32_Info);		//ASCII�ַ���Ϣ
u8	GT32L32_GetGB18030_Info(u16 word,GT32L32_Info_TypeDef *GT32L32_Info);			//GB18030����&�ַ���Ϣ
u8 	GT32L32_Get_Info(u16 word,GT32L32_Info_TypeDef *GT32L32_Info);						//��ȡ�ַ���ַ�͵��󳤶�



u32 GT32L32_GetAddress(u8 font, u8 c1, u8 c2, u8 c3, u8 c4);//��ȡ��ַ
u32 GT32L32_GetBufferLen(u8 font, u8 c1, u8 c2, u8 c3, u8 c4);//��ȡ����
u16  GT32L32_ReadBuffer(GT32L32_Init_TypeDef *GT32L32_Init,u8 font,u16 word,unsigned char *Buffer);		//���ֿ��ж����ݺ���


u32 GT32L32_GetGB18030_12(u8 c1, u8 c2, u8 c3, u8 c4);		//12x12����GB18030����&�ַ���ַ����
u32 GT32L32_GetGB18030_16(u8 c1, u8 c2, u8 c3, u8 c4);		//16x16����GB18030����&�ַ���ַ����
u32 GT32L32_GetGB18030_24(u8 c1, u8 c2, u8 c3, u8 c4);		//24x24����GB18030����&�ַ���ַ����
u32 GT32L32_GetGB18030_32(u8 c1, u8 c2, u8 c3, u8 c4);		//32x32����GB18030����&�ַ���ַ����
u32 GT32L32_GetBarCode_13(u8 * BAR_NUM);															//EAN13��������ó���
u32 GT32L32_GetBarCode_128(u8	*BAR_NUM,u8	flag);											//GB/T 18347-2001(CODE128)��������ó���
u32 GT32L32_GetAntennaCode_12X12(u8	NUM,u8 *GetBuffer);								//����ͼ����ó���
u32 GT32L32_GetBatteryCode_12X12(u8	NUM,u8 *GetBuffer);								//���ͼ����ó���

u16	GT32L32_U2G_13(u16 Unicode);																			//UNICODEתGBK���ӳ���㷨,��1&3�ַ���
u32 GT32L32_UNICODE_To_GBK(u16 unicode,u8 *GetBuffer);								//UNICODEתGBK���ӳ���㷨������������,ת��ȫ��˫�ֽ����� 
u32 GT32L32_BIG5_To_GBK(u16 BIG5_Code,u8 *GetBuffer);									//BIG5תGBKת���㷨




u8 GT32L32_ReadStatus(GT32L32_Init_TypeDef *GT32L32_Init);																					//��ȡоƬ״̬
void GT32L32_ChipErase(GT32L32_Init_TypeDef *GT32L32_Init);																					//̽���û�������
u8  GT32L32_ReadBufferbac(GT32L32_Init_TypeDef *GT32L32_Init,u32 address,u16 len,unsigned char *Buffer);		//���ֿ��ж����ݺ���
u8  GT32L32_ReadWriteByte(u8 Data);		//��дһ���ֽ�
void GT32L32_SendByte(GT32L32_Init_TypeDef *GT32L32_Init,u8 ByteData);		//����һ���ֽ�����,�ȷ��͸�λ
u8 GT32L32_ReadByte(GT32L32_Init_TypeDef *GT32L32_Init);		//��һ���ֽ�,�ȶ���λ



#endif

/*******************	wegam@sina.com	*******************/
/*********************	2017/01/21	*********************/
/**********************	END OF FILE	*********************/

