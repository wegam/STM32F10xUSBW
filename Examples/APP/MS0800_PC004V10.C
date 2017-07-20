#ifdef MS0800_PC004V10

#include "MS0800_PC004V10.H"

#include "string.h"				//�����ڴ��������ͷ�ļ�

/*##############################################################################
################################################################################
# ��Ŀ��		:	MS0800_PC004V10	
# ��������	:	����ҩ�ܵ�Ԫ��
-------------------------------------------------------------------------------	
********************************����˵��***************************************
-------------------------------------------------------------------------------
�� �����ذ�ͨѶ������ָ��������ʾ�壬
�� ��ʾ��ַ�������
�� ���뷽ʽ��
�� CANͨѶ˵����
	�����ʣ�	100K
	���е�ַ��0X000
	
�� 485ͨѶ˵����485�����ذ�ͨѶ
	�����ʣ�	9600
	���ںţ�	USART2
	���ƽţ�	PA1
	ͷ�룺		0XFA,0XFB
	˽�е�ַ�����뿪�ص�ַ
-------------------------------------------------------------------------------
********************************��������***************************************
-------------------------------------------------------------------------------
1��������Ŀ
2���ϵ��������ȡ��Ԫ��ַ
3��	
4��	
5��	
-------------------------------------------------------------------------------
*****************************ָ��/Э��˵��*************************************
-------------------------------------------------------------------------------

--------------------------------CANЭ��----------------------------------------
1.1��	CANЭ�飺				������ܰ�ͨѶ
			CAN��ID��					0X3FF(StdId=0X3FF;)
			���ȣ�						3	(DLC=1;)
			��ʱ��ͬ����			0XAA;(Data[0]=0XAA;)
			��ȡ/��ʾIDָ�	0X01;(Data[0]=0X01;)
			�Լ�ָ�				0X02;(Data[0]=0X02;)
			Ϩ��ָ�				0X03;(Data[0]=0X02;)
-------------------------------------------------------------------------------		
1.2��	����ܰ��ϱ���ַ���趨������ܰ�ID������Ԫ��������ַ
			CAN��ID��			SW_ID�����뿪�ص�ַ��
			���ȣ�				1	(DLC=1;)
			��ʱ��ͬ����	0XAA;(Data[0]=0XAA;)
		
1.3��	����ܰ�������ݣ������ϼ���Ԫ�����ݣ���ʾ���߹ر�
			CAN��ID��			SW_ID�����뿪�ص�ַ��
			���ȣ�					3	(DLC=3;)
			������ʾ��ʶ��	0X08;(Data[0]=0X08;)
			��ʾ�������ݣ�	�����ʾ999
										��λ8λ��Data[1]
										��λ8λ��Data[2]
									
1.4��	����ܰ�Ӧ�����������յ���ȷ�����ݺ�Ļ�ִ
			CAN��ID��			SW_ID�����뿪�ص�ַ��
			���ȣ�					1	(DLC=1;)
			��ʱ��ͬ����		0XA0;(Data[0]=0XA0;)
--------------------------------485Э��----------------------------------------
2.1��	485Э�飺			�����ذ�ͨѶ
			���ݳ��ȣ�	13Byte
			RS232ͨѶ˵����			FA F5 01 01 BCC8 00 06 01 01 01 00 01 25------->FA F5 01 01 27 00 06 01 01 01 00 01 25
//			��ʽ��			HEAD(2)�������ţ���Ԫ�ţ���ֵ��λ����ֵ��λ�����У��
			˵����
			HEAD:				0XFA,0XFB
			���			0X00--�ر���ʾ��0X01--��ʾ��ֵ��0x02--��ȡ�����豸��ַ����ź͵�Ԫ�ţ�,0x03--��ʾ��ԪID��
			��ţ�			��Ԫ��ţ�1~0XFF
			��Ԫ�ţ�		�����ID�ţ�10~77
			��ֵ��λ��	���λ����
			��ֵ��λ��	���λ���ӣ����Ӻ�ȡֵ��Χ��0~999�������ֻ����λ
			���У�飺	Byte0~Byte6У��
			�������ݵ�Ԫ���󣬻�ִλΪ0X80|��������
			�������ݵ�Ԫ��ַ���󣺻�ִλ0XE0|��������
-------------------------------------------------------------------------------


********************************************************************************

################################################################################
###############################################################################*/

#define RS485TX	0
#define RS485RX	1
#define PC004V10_RS4851_CTL(n)	if(n==RS485RX){GPIO_ResetBits(GPIOA,GPIO_Pin_1);}else{GPIO_SetBits(GPIOA,GPIO_Pin_1);}

//************CANͨѶ����
#define	CAN_ID				PC004V10_Num[8]*10+PC004V10_Num[9]		//��Ԫ��ַ	����λ���ϲ���ʮλΪ�㣬��λΪ���
#define	CAN_Command		PC004V10_Num[5]												//��������----��λ���û���
#define	Data_H				PC004V10_Num[11]											//���ݸ�λ
#define	Data_L				PC004V10_Num[12]											//���ݵ�λ
//************485ͨѶ����
//#define	

#define	PC004V10_BufferSize 32															//DMA1�����С

u8 PC004V10_Buffer[PC004V10_BufferSize]={0};								//RS485����
u8 PC004V10_Num[13]={0};																		//���յ�������

u8 ID_ARR[8][8]={0,0};			//�洢�����������ID�б�
u8 ON_line[8][8]={0,0};			//��ӦID�б���־λ��0--�޴�ID��1--�ж�ӦID

u8 PC004V10_ID=0;						//��Ԫ��ID----���ҵ�������Ϊ���ߣ����ұ�Ϊ���λ

u16	SYSTime=0;							//ѭ����ʱ����
u8 Buzzer_time=0;						//������������Ӧ����

u8 Self_Dsp=0;							//���뿪��Ϊ0ʱ���Լ������ʾ��ʶ����

CanRxMsg RxMessage;

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void MS0800_PC004V10_Configuration(void)
{
	CAN_Configuration();	//CAN����
	
	CAN_FilterInitConfiguration(0,0X000,0X000);					//CAN�˲�������
	
	USART_DMA_Configuration(USART2,9600,1,1,(u32*)PC004V10_Buffer,(u32*)PC004V10_Buffer,PC004V10_BufferSize);	//USART_DMA����
	GPIO_Configuration(GPIOA,GPIO_Pin_1,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);			//RS485_CTL
	
	GPIO_Configuration(GPIOB,GPIO_Pin_14,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);			//GPIO����--BUZZER
	GPIO_Configuration(GPIOB,GPIO_Pin_15,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);			//GPIO����--WORK_LED
	
	//********************���뿪������
	GPIO_Configuration(GPIOC,GPIO_Pin_0,GPIO_Mode_IPU,GPIO_Speed_50MHz);			//GPIO����---���뿪��
	GPIO_Configuration(GPIOC,GPIO_Pin_1,GPIO_Mode_IPU,GPIO_Speed_50MHz);			//GPIO����
	GPIO_Configuration(GPIOC,GPIO_Pin_2,GPIO_Mode_IPU,GPIO_Speed_50MHz);			//GPIO����
	GPIO_Configuration(GPIOC,GPIO_Pin_3,GPIO_Mode_IPU,GPIO_Speed_50MHz);			//GPIO����
	GPIO_Configuration(GPIOC,GPIO_Pin_4,GPIO_Mode_IPU,GPIO_Speed_50MHz);			//GPIO����
	GPIO_Configuration(GPIOC,GPIO_Pin_5,GPIO_Mode_IPU,GPIO_Speed_50MHz);			//GPIO����
	GPIO_Configuration(GPIOC,GPIO_Pin_6,GPIO_Mode_IPU,GPIO_Speed_50MHz);			//GPIO����
	GPIO_Configuration(GPIOC,GPIO_Pin_7,GPIO_Mode_IPU,GPIO_Speed_50MHz);			//GPIO����	

	GPIO_ResetBits(GPIOB,GPIO_Pin_14);
	PC004V10_RS4851_CTL(RS485RX);
	
	PC004V10_Num[8]=1;
	PC004V10_Num[9]=0;
	PC004V10_Num[11]=0;
	PC004V10_Num[12]=0;

}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void MS0800_PC004V10_Server(void)
{
	//ѭ������1mS
	u8 status=0;
	SYSTime++;
		
	if(SYSTime>=10000)
		SYSTime=0;
	//**************���ͨѶ��־λ���鿴�Ƿ�ΪͨѶ�жϣ�����ǣ���˴�ʱ��������Ч
	status=PC004V10_485_TR();				//485�շ�����//ͨѶ�ӿ�
	if(status&&(SYSTime>0))
		SYSTime--;	
	
	PC004V10_BUZZER(SYSTime);				//������
	
	//**************����豸ID���ޱ仯���о͸���
	if(SYSTime%500==0)
	{
		if(PC004V10_ID!=PC004V10_Get_ID())
		{
			PC004V10_ID=PC004V10_Get_ID();
			PC004V10_Buffer[5]=0x01;							//��ȡIDָ��0x01
			PC004V10_CAN_COMMAD();					//CAN�������������ַ���������ͣ�����--ʱ��ͬ��--���ͻ�ȡD����
		}
	}
	if(PC004V10_ID==0)
	{
		Self_Test(SYSTime);
	}
	WORK_LED(SYSTime);				//WORK_LED
}
/*******************************************************************************
* ������		:
* ��������	: 
* ����			: ��
* ���			: ��
* ����			: ��
*******************************************************************************/
u8 PC004V10_485_TR(void)				//485�շ�����//ͨѶ�ӿ�
{
	//***********����״̬������485��Ϊ���շ�ʽ
	u16 num=0;
//	u8 temp=0;
	num=USART_RX_FlagClear(USART2);						//�������DMA����ȫ���жϱ�־
	//1)**********�Խ��յ������ݽ��й����
	if(num)
	{	
		//1.1)**********RS485ת������ģʽ����ֹ�����ݽ�����������
		PC004V10_RS4851_CTL(RS485TX);						//485�ڷ���ģʽ������������
		PC004V10_delay(2000);										//��ʱ��ʹ��485�շ��������ŵ�ƽ��ȷת��
		//1.3)**********��������Ϣ
		if((PC004V10_Buffer[0]==0xFA)&&(PC004V10_Buffer[1]==0xF5)&&(PC004V10_Buffer[2]==0x05)&&(PC004V10_Buffer[3]==0x0A))
		{
			if(PC004V10_Buffer[5]==0x04)					//����IDָ��
			{
				memset(ID_ARR,0,sizeof(ID_ARR));		//��յ�ַ�洢�ռ�
				memset(ON_line,0,sizeof(ON_line));	//��յ�ַ���߱�־
			}
			PC004V10_CAN_COMMAD();								//CAN�������������ַ���������ͣ�����--ʱ��ͬ��
			PC004V10_RS4851_CTL(RS485RX);					//485תΪ����ģʽ
		}
		//1.2)**********���������ȷ��
		else if((PC004V10_Buffer[0]==0xFA)&&(PC004V10_Buffer[1]==0xF5)&&(PC004V10_Buffer[7]==PC004V10_ID)&&((PC004V10_Buffer[4]==BCC8(&PC004V10_Buffer[7],6))))			//���ͷ��ʶ������ź����У��λ
		{
//			/********************************************/
//			//***************�жϵ�Ԫ��ַ��ȷ���
			u8 temp=0;
			temp=PC004V10_Buffer[8]*10+PC004V10_Buffer[9];						//����Ԫ��ַ�Ĳ�ź���źϳ�
			//1.2.1��***************�жϵ�Ԫ��ַ��ȷ���
			if(memchr(ID_ARR,temp,sizeof(ID_ARR)))										//��Ԫ��ַ��ȷ	//�жϵ�Ԫ��ַ��û���ڣ�������x��ǰn���ֽ��������ַ�ch������ֵΪָ��s���״γ���ch��ָ��λ�á����chû����s�����г��֡�����NULL			
			{
				memcpy(PC004V10_Num,PC004V10_Buffer,13);								//��������						

				PC004V10_Buffer[3]|=0x80;						//�޸Ļ�ִλ
				USART_DMASend(USART2,(u32*)PC004V10_Buffer,(u32)num);		//���ػ�ִ�����ذ�
				
				PC004V10_CAN_TX();					//CAN�������ݣ���ַ���������ͣ�����
			}
			else		//��Ԫ��ַ����
			{
				PC004V10_Buffer[3]|=0xE0;																//�޸Ļ�ִλ
				USART_DMASend(USART2,(u32*)PC004V10_Buffer,(u32)num);		//���ػ�ִ�����ذ�
			}
			/**********************************************/
		}
		
		else			//����������Ч���ָ�485Ϊ��������״̬
		{
			memset(PC004V10_Buffer,0,PC004V10_BufferSize);	//��ʼ������
			PC004V10_RS4851_CTL(RS485RX);										//485תΪ����ģʽ			
		}
		return 1;		
	}
	//2)**********���ݴ���2DMA��������ж�
	else if(USART_TX_DMAFlagClear(USART2))
	{
		memset(PC004V10_Buffer,0,PC004V10_BufferSize);			//��ʼ������		
		return 1;
	}
	//3)**********���ݷ�����ɺ�485���ƽ�����תΪ����ģʽ
	else	if(USART_GetFlagStatus(USART2,USART_FLAG_TC)==SET)
	{
		PC004V10_RS4851_CTL(RS485RX);
		USART_ClearFlag(USART2,USART_FLAG_TC);
		return 0;
	}
	else
	{
		return 0;
	}
}
/*******************************************************************************
* ������		:
* ��������	: 
* ����			: ��
* ���			: ��
* ����			: ��
*******************************************************************************/
void PC004V10_CAN_RX(void)									//��Ҫ��������ܰ��ַ��
{
	u8 ID_Temp=0;
	if(SET == CAN_GetITStatus(CAN_IT_FF0))
	{
		CAN_ClearITPendingBit(CAN_IT_FF0);
	}
	else if(SET == CAN_GetITStatus(CAN_IT_FOV0))
	{
		CAN_ClearITPendingBit(CAN_IT_FOV0);
	}
	else if(SET == CAN_GetITStatus(CAN_IT_FMP0))
	{
		CAN_ClearITPendingBit(CAN_IT_FMP0);
	}
	else
	{
		CAN_Receive(CAN_FIFO0,&RxMessage);
		if(RxMessage.DLC==1&&RxMessage.Data[0]==0XAA)
		{
			ID_Temp=RxMessage.StdId;
			ID_ARR[ID_Temp/10][ID_Temp%10]=ID_Temp;
			ON_line[ID_Temp/10][ID_Temp%10]=1;
		}
	}
}
/*******************************************************************************
* ������		:
* ��������	: 
* ����			: ��
* ���			: ��
* ����			: ��
*******************************************************************************/
void PC004V10_CAN_TX(void)					//CAN�������ݣ���ַ���������ͣ�����
{
	CanTxMsg TxMessage;
	
	u8	TransmitMailbox = 0;
	u32	i;

	TxMessage.StdId=CAN_ID;					//��Ԫ��ַ
//	TxMessage.ExtId=0XFF;
	TxMessage.RTR=CAN_RTR_DATA;			//����֡
	TxMessage.IDE=CAN_ID_STD;				//ʹ�ñ�׼��ʶ��
	TxMessage.DLC=3;
	TxMessage.Data[0]=CAN_Command;	//��������
	TxMessage.Data[1]=Data_H;				//���ݸ�λ
	TxMessage.Data[2]=Data_L;				//���ݵ�λ
//	TxMessage.Data[3]=0x00;
//	TxMessage.Data[4]=0x00;
//	TxMessage.Data[5]=0x00;
//	TxMessage.Data[6]=0x00;
//	TxMessage.Data[7]=0x00;
	TransmitMailbox = CAN_Transmit(&TxMessage);
	i = 0;
	// ���ڼ����Ϣ�����Ƿ�����
	while((CAN_TransmitStatus(TransmitMailbox) != CANTXOK) && (i != 0xFF))
	{
		i++;
	}
	i = 0;
	// ��鷵�صĹҺŵ���Ϣ��Ŀ
	while((CAN_MessagePending(CAN_FIFO0) < 1) && (i != 0xFF))
	{
		i++;
	}

}
/*******************************************************************************
* ������		:
* ��������	: 
* ����			: ��
* ���			: ��
* ����			: ��
*******************************************************************************/
void PC004V10_CAN_COMMAD(void)					//CAN�������������ַ���������ͣ�����--ʱ��ͬ��
{
	CanTxMsg TxMessage;
	
	u8	TransmitMailbox = 0;
	u32	i;

	TxMessage.StdId=0X3FF;								//��Ԫ��ַ
//	TxMessage.ExtId=0XFF;
	TxMessage.RTR=CAN_RTR_DATA;						//����֡
	TxMessage.IDE=CAN_ID_STD;							//ʹ�ñ�׼��ʶ��
	TxMessage.DLC=3;
	TxMessage.Data[0]=PC004V10_Buffer[5];	//��������
	TxMessage.Data[1]=0x00;						//���ݸ�λ
	TxMessage.Data[2]=0x00;						//���ݵ�λ
//	TxMessage.Data[3]=0x00;
//	TxMessage.Data[4]=0x00;
//	TxMessage.Data[5]=0x00;
//	TxMessage.Data[6]=0x00;
//	TxMessage.Data[7]=0x00;
	TransmitMailbox = CAN_Transmit(&TxMessage);
	i = 0;
	// ���ڼ����Ϣ�����Ƿ�����
	while((CAN_TransmitStatus(TransmitMailbox) != CANTXOK) && (i != 0xFF))
	{
		i++;
	}
	i = 0;
	// ��鷵�صĹҺŵ���Ϣ��Ŀ
	while((CAN_MessagePending(CAN_FIFO0) < 1) && (i != 0xFF))
	{
		i++;
	}
	memset(PC004V10_Buffer,0,PC004V10_BufferSize);	//��ʼ������
}
/*******************************************************************************
* ������		:
* ��������	: 
* ����			: ��
* ���			: ��
* ����			: ��
*******************************************************************************/
void WORK_LED(u16 time)				//WORK_LED
{
	if(time%1000<500)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_15);				//WORK_LED		
	}
	else
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);			//WORK_LED
	}
}
/*******************************************************************************
* ������		:
* ��������	: 
* ����			: ��
* ���			: ��
* ����			: ��
*******************************************************************************/
void PC004V10_BUZZER(u16 time)				//WORK_LED
{
	if(Buzzer_time<6)
	{
		if(time%1000<500)
		{
			GPIO_SetBits(GPIOB,GPIO_Pin_14);				//Buzzer_ON		
		}
		else
		{
			GPIO_ResetBits(GPIOB,GPIO_Pin_14);			//Buzzer_OFF
		}
		if(time%500==0)
		{
			Buzzer_time++;
		}
	}
	else
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);			//Buzzer_OFF
	}
}
/*******************************************************************************
*������		:	function
*��������	:	��������˵��
*����			: 
*���			:	��
*����ֵ		:	��
*����			:
*******************************************************************************/
u8 PC004V10_Get_ID(void)
{
	u8	SW_ID=0;
	//1)***************��ȡH���뿪�ص�ַ�����ҵ���Ϊ�ӵ͵���
	if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4))		//SWH-1
	{
		SW_ID|=1;
	}
	SW_ID<<=1;
	if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5))		//SWH-2
	{
		SW_ID|=1;
	}
	SW_ID<<=1;
	if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6))		//SWH-3
	{
		SW_ID|=1;
	}
	SW_ID<<=1;
	if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7))		//SWH-4
	{
		SW_ID=1;
	}
	
	//2)***************��ȡL���뿪�ص�ַ�����ҵ���Ϊ�ӵ͵���
	SW_ID<<=1;
	if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0))		//SWL-1
	{
		SW_ID|=1;
	}
	SW_ID<<=1;
	if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1))		//SWL-2
	{
		SW_ID|=1;
	}
	SW_ID<<=1;
	if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2))		//SWL-3
	{
		SW_ID|=1;
	}
	SW_ID<<=1;
	if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3))		//SWL-4
	{
		SW_ID|=1;
	}
	
	//3)***************���ػ�ȡ��IDֵ
	return (SW_ID);

}
void Self_Test(u16 time)			//���뿪��Ϊ0ʱ��������ʾ����
{
//	//************CANͨѶ����
//#define	CAN_ID				PC004V10_Num[8]*10+PC004V10_Num[9]		//��Ԫ��ַ	����λ���ϲ���ʮλΪ�㣬��λΪ���
//#define	CAN_Command		PC004V10_Num[5]												//��������----��λ���û���
//#define	Data_H				PC004V10_Num[11]											//���ݸ�λ
//#define	Data_L				PC004V10_Num[12]											//���ݵ�λ
//	Self_Dsp=0;							//���뿪��Ϊ0ʱ���Լ������ʾ��ʶ����
	if((time%10==0)&&(Self_Dsp==0))
	{
		PC004V10_Num[5]=0x05;				//��������----��λ���û���
		
//		PC004V10_Num[12]++;
		if(PC004V10_Num[12]++>=99)
		{
			PC004V10_Num[12]=0;
			if(PC004V10_Num[11]++>=9)
			{
				PC004V10_Num[11]=0;
				if(PC004V10_Num[9]++>=3)
				{
					PC004V10_Num[9]=0;
					if(PC004V10_Num[8]++>=5)
					{
						PC004V10_Num[8]=1;
						PC004V10_Num[9]=0;
						PC004V10_Num[11]=0;
						PC004V10_Num[12]=0;
						Self_Dsp=1;							//���뿪��Ϊ0ʱ���Լ������ʾ��ʶ������1--��ʾID
						SYSTime=0;
					}
				}
			}
		}		
		PC004V10_CAN_TX();					//CAN�������ݣ���ַ���������ͣ�����
	}
	else if(Self_Dsp==1)
	{
		if(time<5)
		{
			PC004V10_Num[5]=0x04;				//��������----��ʾID
//			PC004V10_CAN_TX();					//CAN�������ݣ���ַ���������ͣ�����
			PC004V10_CAN_COMMAD();			//CAN�������������ַ���������ͣ�����--ʱ��ͬ��
		}
		else if(time>=9999)
		{
			PC004V10_Num[5]=0x05;				//��������----��λ���û���
			PC004V10_Num[8]=1;
			PC004V10_Num[9]=0;
			PC004V10_Num[11]=0;
			PC004V10_Num[12]=0;
			PC004V10_CAN_COMMAD();			//CAN�������������ַ���������ͣ�����--ʱ��ͬ��
			Self_Dsp=0;							//���뿪��Ϊ0ʱ���Լ������ʾ��ʶ������0--����
		}
	}
}

/*******************************************************************************
* ������		:
* ��������	: 
* ����			: ��
* ���			: ��
* ����			: ��
*******************************************************************************/
void PC004V10_delay(u16 time)
{
	while(time--);
}



#endif