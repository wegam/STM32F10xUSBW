#include "STM32F10x_ADS1230H20160808.H"


#define	ADS1230_CLKP GPIO_Pin_12	//ʱ��
#define	GPIO_ADSCLK GPIOA

#define ADS1230_DATP GPIO_Pin_11	//����
#define GPIO_ADSDAT GPIOA

#define ADS1230_SPWNP GPIO_Pin_10	//�͵�ƽ����
#define GPIO_ADSSPWN GPIOA

#define ADS1230_GAINP GPIO_Pin_8	//����
#define GPIO_ADSGAIN GPIOA

#define ADS1230_SPEENP GPIO_Pin_9	//ת����
#define GPIO_ADSSPEEN GPIOA

#define ADS1230_GAIN	64		//�͵�ƽ-����64,�ߵ�ƽ-����128
#define ADS1230_SPEED	10		//�͵�ƽ-10SPS,�ߵ�ƽ-80SPS

u32 ADS1230_ADC=0;

/*******************************************************************************
*������		:ADS1230_Delay
*��������	:ADS1230��ʱ����
*����			:j-��ʱʱ��					
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void ADS1230_Delay(u16 j)
{
	u8 i=36;
	for(;j>0;j--)
		for(i=36;i>0;i--);
}
/*******************************************************************************
*������		:ADS1230_Configuration
*��������	:ADS1230�ܽų�ʼ��
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void ADS1230_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef	EXTI_InitStructure;		
	NVIC_InitTypeDef	NVIC_InitStructure;	
	
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);				//�ر�AFIOʱ��,Ϊ�ر�JTAG����
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  //�ر�JTAG����
	
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   //��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//�������ж�
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource11);
	EXTI_InitStructure.EXTI_Line=EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;			//�½����ж�
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;		//�ж���ʹ��
	EXTI_Init(&EXTI_InitStructure);			
	EXTI_GenerateSWInterrupt(EXTI_Line11);		//ʹ���ж�
//	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQChannel;//�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	//?????
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;			//?????
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//??
	NVIC_Init(&NVIC_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line11);
}

void ADSREAD_SET(u8 EN)
{
	EXTI->PR=1<<11;  //���LINE11�ϵ��жϱ�־
	if(EN)
		EXTI->IMR|=1<<11;//������line11�ϵ��ж�
	else
		EXTI->IMR&=~(1<<11);//����line11���ж�
}

/*******************************************************************************
*������		:ADS1230_INIT
*��������	:ADS1230�ܽų�ʼ��
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void ADS1230_INIT(void)
{
#ifdef	ADS1230_GAIN		//�͵�ƽ-����64,�ߵ�ƽ-����128
		GPIO_WriteBit(GPIO_ADSGAIN,ADS1230_GAINP,Bit_RESET);		//PGA==64
#else
		GPIO_WriteBit(GPIO_ADSGAIN,ADS1230_GAINP,Bit_SET);			//PGA==128
#endif
	
#ifdef	ADS1230_SPEED		//�͵�ƽ-10SPS,�ߵ�ƽ-80SPS
		GPIO_WriteBit(GPIO_ADSSPEEN,ADS1230_SPEENP,Bit_RESET);	//10SPS
#else
		GPIO_WriteBit(GPIO_ADSSPEEN,ADS1230_SPEENP,Bit_SET);		//80SPS
#endif
	
		GPIO_WriteBit(GPIO_ADSSPWN,ADS1230_SPWNP,Bit_SET);			//����ת��
}
/*******************************************************************************
*������		:ADS1230_INIT
*��������	:ADS1230�ܽų�ʼ��
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void ADS1230_START(void)
{
//	GPIO_ResetBits(GPIOB,GPIO_Pin_1);			//���			
//	GPIO_ResetBits(GPIOB,GPIO_Pin_10);		//����		
//	GPIO_ResetBits(GPIOB,GPIO_Pin_11);		//�Ƶ�

//	GPIO_ResetBits(GPIOB,GPIO_Pin_3);		//���
//	GPIO_ResetBits(GPIOB,GPIO_Pin_5);		//����
//	GPIO_ResetBits(GPIOB,GPIO_Pin_4);		//�Ƶ�
//	
////		ADSREAD_SET(1);	//ʹ���ж�
//	GPIO_SetBits(GPIOB,GPIO_Pin_10);		//����
//	GPIO_SetBits(GPIOB,GPIO_Pin_5);			//����
	
	GPIO_WriteBit(GPIO_ADSSPWN,ADS1230_SPWNP,Bit_SET);			//����
	GPIO_WriteBit(GPIO_ADSCLK,ADS1230_CLKP,Bit_RESET);			//ʱ���ź�����
}
/*******************************************************************************
*������		:ADS1230_INIT
*��������	:ADS1230�ܽų�ʼ��
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void ADS1230_STOP(void)
{
//	GPIO_ResetBits(GPIOB,GPIO_Pin_1);			//���			
//	GPIO_ResetBits(GPIOB,GPIO_Pin_10);		//����		
//	GPIO_ResetBits(GPIOB,GPIO_Pin_11);		//�Ƶ�

//	GPIO_ResetBits(GPIOB,GPIO_Pin_3);		//���
//	GPIO_ResetBits(GPIOB,GPIO_Pin_5);		//����
//	GPIO_ResetBits(GPIOB,GPIO_Pin_4);		//�Ƶ�

//	GPIO_SetBits(GPIOB,GPIO_Pin_1);	//���
//	GPIO_SetBits(GPIOB,GPIO_Pin_3);	//���	
	GPIO_WriteBit(GPIO_ADSSPWN,ADS1230_SPWNP,Bit_RESET);			//ֹͣ
}
/*******************************************************************************
*������		:ADS1230_INIT
*��������	:ADS1230�ܽų�ʼ��
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
u32 ADS1230_READ(void)
{
	u8 i=0;
//	u32 ADS1230_ADC=0;
		ADS1230_ADC=0;
//	GPIO_ResetBits(GPIOB,GPIO_Pin_1);			//���			
//	GPIO_ResetBits(GPIOB,GPIO_Pin_10);		//����		
//	GPIO_ResetBits(GPIOB,GPIO_Pin_11);		//�Ƶ�

//	GPIO_ResetBits(GPIOB,GPIO_Pin_3);		//���
//	GPIO_ResetBits(GPIOB,GPIO_Pin_5);		//����
//	GPIO_ResetBits(GPIOB,GPIO_Pin_4);		//�Ƶ�
	
////	ADS1230_Delay(10);

//	GPIO_SetBits(GPIOB,GPIO_Pin_11);		//�Ƶ�
//	GPIO_SetBits(GPIOB,GPIO_Pin_4);			//�Ƶ�
	ADS1230_Delay(50);
	for(i=0;i<20;i++)
	{
		GPIO_SetBits(GPIO_ADSCLK,ADS1230_CLKP);			//ʱ�Ӹ�
		GPIO_SetBits(GPIOB,GPIO_Pin_11);		//�Ƶ�
		GPIO_SetBits(GPIOB,GPIO_Pin_4);			//�Ƶ�
		ADS1230_Delay(5);
		if(GPIO_ReadInputDataBit(GPIO_ADSDAT,ADS1230_DATP))
		{
			ADS1230_ADC|=0x01;
			ADS1230_ADC<<=1;
		}
		else
		{
			ADS1230_ADC<<=1;
		}
		GPIO_ResetBits(GPIO_ADSCLK,ADS1230_CLKP);			//ʱ�ӵ�
		GPIO_ResetBits(GPIOB,GPIO_Pin_11);		//�Ƶ�
		GPIO_ResetBits(GPIOB,GPIO_Pin_4);			//�Ƶ�
		ADS1230_Delay(5);
	}
	
	GPIO_ResetBits(GPIO_ADSDAT,ADS1230_DATP);			//ʱ�ӵ�
//	ADS1230_ADC>>=2;
	return ADS1230_ADC;
	
//	GPIO_WriteBit(GPIO_ADSSPWN,ADS1230_SPWNP,Bit_RESET);			//ֹͣ
}
/*******************************************************************************
*������		:ADS1230_INIT
*��������	:ADS1230�ܽų�ʼ��
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void ADS1230(void)
{
	EXTI_ClearITPendingBit(EXTI_Line11);
//	ADSREAD_SET(0);
	ADS1230_Configuration();
	GPIO_Write(GPIOB,0XFFFF);			
	GPIO_Write(GPIOB,0XFF00);	
	
	GPIO_SetBits(GPIOB,GPIO_Pin_1);			//���			
	GPIO_SetBits(GPIOB,GPIO_Pin_10);		//����		
	GPIO_SetBits(GPIOB,GPIO_Pin_11);		//�Ƶ�

	GPIO_SetBits(GPIOB,GPIO_Pin_3);		//���
	GPIO_SetBits(GPIOB,GPIO_Pin_5);		//����
	GPIO_SetBits(GPIOB,GPIO_Pin_4);		//�Ƶ�
	
//	ADS1230_START();
//	ADS1230_READ();
//	GPIO_WriteBit(GPIO_ADSSPWN,ADS1230_SPWNP,Bit_RESET);			//����
}
/*******************************************************************************
*������		:ADS1230_LED_DISPLAY
*��������	:ADS1230�ܽų�ʼ��
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void ADS1230_LED_DISPLAY(void)
{
	if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_10))
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_1);			//���			
		GPIO_ResetBits(GPIOB,GPIO_Pin_10);		//����		
		GPIO_ResetBits(GPIOB,GPIO_Pin_11);		//�Ƶ�

		GPIO_SetBits(GPIOB,GPIO_Pin_3);		//���
		GPIO_SetBits(GPIOB,GPIO_Pin_5);		//����
		GPIO_SetBits(GPIOB,GPIO_Pin_4);		//�Ƶ�
		
	}
	else
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_1);			//���			
		GPIO_SetBits(GPIOB,GPIO_Pin_10);		//����		
		GPIO_SetBits(GPIOB,GPIO_Pin_11);		//�Ƶ�
		
		GPIO_ResetBits(GPIOB,GPIO_Pin_3);		//���
		GPIO_ResetBits(GPIOB,GPIO_Pin_5);		//����
		GPIO_ResetBits(GPIOB,GPIO_Pin_4);		//�Ƶ�
		
	}
}

