
#include "I2CH20160808.H"
//#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	

////////////////////////////////////////////////////////////////////////////////// 	
//#define IIC_SCL	GPIO_Pin_10
//#define IIC_SDA	GPIO_Pin_11
//#define GPIO_SCL	GPIOB
//#define GPIO_SDA	GPIOB
//GPIO_ResetBits(GPIO_TypeDef* GPIOx, u16 GPIO_Pin);
//GPIO_SetBits(GPIO_TypeDef* GPIOx, u16 GPIO_Pin);
//GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, u16 GPIO_Pin);
//GPIO_WriteBit(GPIO_TypeDef* GPIOx, u16 GPIO_Pin, BitAction BitVal);
//#define	(IIC_SCL=1) 0
u16 IIC_SCL=0;
GPIO_TypeDef* GPIO_SCL=0;

GPIO_TypeDef* GPIO_SDA=0;
u16 IIC_SDA=0;

/*******************************************************************************
*������		:IIC_Delay
*��������	:595��ʱ����
*����			:j-��ʱʱ��					
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void IIC_Delay(u16 j)
{
	for(;j>0;j--);
}

/*******************************************************************************
*������		:IIC_INII
*��������	:IIC�ܽų�ʼ��
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void IIC_PinSet(GPIO_TypeDef* GPIOP_SCL,u16 IIC_SCLP,GPIO_TypeDef* GPIOP_SDA,u16 IIC_SDAP)
{
	GPIO_SCL=GPIOP_SCL;
	IIC_SCL=IIC_SCLP;
	
	GPIO_SDA=GPIOP_SDA;
	IIC_SDA=IIC_SDAP;
}
/*******************************************************************************
*������		:IIC_Configuration
*��������	:IIC�ܽų�ʼ��
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void IIC_Configuration(void)
{					
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<4;//��ʹ������IO PORTCʱ��
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	//SCL�ܽ�����
	GPIO_InitStructure.GPIO_Pin = IIC_SCL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_SCL, &GPIO_InitStructure);
	//SDA�ܽ�����
	GPIO_InitStructure.GPIO_Pin = IIC_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_SDA, &GPIO_InitStructure);

}
/*******************************************************************************
*������		:IIC_IN
*��������	:IIC�����ݹܽ�����
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void IIC_IN(void)
{					
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<4;//��ʹ������IO PORTCʱ��
//	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	
	GPIO_InitStructure.GPIO_Pin = IIC_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_SDA, &GPIO_InitStructure);

//	IIC_SCL=1;
//	IIC_SDA=1;
}
/*******************************************************************************
*������		:IIC_OUT
*��������	:IICд���ݹܽ�����
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void IIC_OUT(void)
{					
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<4;//��ʹ������IO PORTCʱ��
//	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	
	GPIO_InitStructure.GPIO_Pin = IIC_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_SDA, &GPIO_InitStructure);

}
/*******************************************************************************
*������		:IIC_Start
*��������	:IIC����
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void IIC_Start(void)
{
	//����������Ϊ���ģʽ
	IIC_OUT();     //sda�����
	GPIO_WriteBit(GPIO_SDA,IIC_SDA,Bit_SET);
//	IIC_SDA=1;
	GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_SET);
//	IIC_SCL=1;
	IIC_Delay(5);
	GPIO_WriteBit(GPIO_SDA,IIC_SDA,Bit_RESET);
// 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	IIC_Delay(5);
	GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_RESET);
//	IIC_SCL=0;//ǯסIIC���ߣ�׼�����ͻ��������
}
/*******************************************************************************
*������		:IIC_Stop
*��������	:IICֹͣ
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void IIC_Stop(void)
{
	IIC_OUT();//sda�����
	GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_RESET);
	GPIO_WriteBit(GPIO_SDA,IIC_SDA,Bit_RESET);
//	IIC_SCL=0;
//	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	IIC_Delay(5);
	GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_SET);
	GPIO_WriteBit(GPIO_SDA,IIC_SDA,Bit_SET);
//	IIC_SCL=1; 
//	IIC_SDA=1;//����IIC���߽����ź�
	IIC_Delay(5);								
}
/*******************************************************************************
*������		:IIC_Wait_Ack
*��������	:���IICӦ��
*����			:��
*���			:��
*����ֵ		:1������Ӧ��ʧ��
					 0������Ӧ��ɹ�
*����			��
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	IIC_IN();//SDA����Ϊ����
	GPIO_WriteBit(GPIO_SDA,IIC_SDA,Bit_SET);  
//	IIC_SDA=1;
	IIC_Delay(5);
	GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_SET);	
//	IIC_SCL=1;
	IIC_Delay(5);	
	while(GPIO_ReadInputDataBit(GPIO_SDA,IIC_SDA))
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_RESET);
//	IIC_SCL=0;//ʱ�����0 	
	return 0;  
}
/*******************************************************************************
*������		:IIC_Ack
*��������	:����Ӧ���ź�
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
void IIC_Ack(void)
{
	GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_RESET);
//	IIC_SCL=0;
	IIC_OUT();
	GPIO_WriteBit(GPIO_SDA,IIC_SDA,Bit_RESET);
//	IIC_SDA=0;
	IIC_Delay(5);
	GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_SET);
//	IIC_SCL=1;
	IIC_Delay(5);
	GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_RESET);
//	IIC_SCL=0;
}
/*******************************************************************************
*������		:IIC_NAck
*��������	:��Ӧ��
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
//������ACKӦ��		
void IIC_NAck(void)
{
	GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_RESET);
//	IIC_SCL=0;
	IIC_OUT();
	GPIO_WriteBit(GPIO_SDA,IIC_SDA,Bit_SET); 
//	IIC_SDA=1;
	IIC_Delay(5);
	GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_SET);
//	IIC_SCL=1;
	IIC_Delay(5);
	GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_RESET);
//	IIC_SCL=0;
}
/*******************************************************************************
*������		:IIC_Send_Byte
*��������	:IIC����һ���ֽ�
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	IIC_OUT(); 
	GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_RESET);
//	IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
	IIC_Delay(5);
	for(t=0;t<8;t++)
	{     
		if((txd&0x80) == 0x00)
			GPIO_WriteBit(GPIO_SDA,IIC_SDA,Bit_RESET);
		else
			GPIO_WriteBit(GPIO_SDA,IIC_SDA,Bit_SET);
//        IIC_SDA=(txd&0x80)>>7;
		txd<<=1; 	
		IIC_Delay(5);   //��TEA5767��������ʱ���Ǳ����
		GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_SET);
//		IIC_SCL=1;
		IIC_Delay(5);
		GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_RESET);
//		IIC_SCL=0;	
		IIC_Delay(5);
	}	
}
/*******************************************************************************
*������		:
*��������	:
*����			:��
*���			:��
*����ֵ		:��
*����			��
*******************************************************************************/
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC_IN();//SDA����Ϊ����
	for(i=0;i<8;i++ )
	{
		GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_RESET);
//	IIC_SCL=0; 
		IIC_Delay(5);
		GPIO_WriteBit(GPIO_SCL,IIC_SCL,Bit_SET);
//	IIC_SCL=1;
		IIC_Delay(5);
		receive<<=1;
		if(GPIO_ReadInputDataBit(GPIO_SDA,IIC_SDA))
			receive++;   
		IIC_Delay(5); 
    }					
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}



