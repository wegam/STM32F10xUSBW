/******************************** User_library *********************************
* �ļ��� 	: STM32_SDCard.H
* ����   	: wegam@sina.com
* �汾   	: V
* ����   	: 2016/01/01
* ˵��   	: 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


#include "STM32_TOOLS.H"


/*******************************************************************************
* ������		:	8λ���У��
* ��������	: 
* ����			: *P ָ�룬numb���ݳ��ȣ�������
* ���			: ������
* ����			: ��
*******************************************************************************/
u8 BCC8(u8 *Pointer,u16 num)			//���У��
{
	u16 i_Num=0;
	u16 temp=0;
	for(i_Num=0;i_Num<num;i_Num++)
	{
		temp=temp^(*Pointer);
		Pointer++;
	}
	return temp;
}
/*******************************************************************************
* ������		:	���У��
* ��������	: 
* ����			: 
* ���			: ������
* ����			: ��
*******************************************************************************/
u8 CRC8(u8 *Pointer)			//ѭ������У��
{
	return 0;
}
/*******************************************************************************
* ������		:	��������У��
* ��������	: 
* ����			: ��
* ���			: ��
* ����			: ��
*******************************************************************************/
u8 LRC8(u8 *Pointer)		//��������У��
{
	return 0;
}
