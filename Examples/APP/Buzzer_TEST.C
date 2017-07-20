#ifdef Buzzer_TEST
#include "Buzzer_TEST.H"

#include "STM32_SYS.H"
#include "STM32_SYSTICK.H"

unsigned long TheSysClock=0;
// ����������������������ף�� 
const tNote MyScore[ ] = 
{ 
	{L3, T/4}, 
	{L5, T/8+T/16}, 
	{L6, T/16}, 
	{M1, T/8+T/16}, 
	{M2, T/16}, 
	{L6, T/16}, 
	{M1, T/16},
	{L5, T/8}, 
	{M5, T/8+T/16}, 
	{H1, T/16}, 
	{M6, T/16}, 
	{M5, T/16}, 
	{M3, T/16}, 
	{M5, T/16}, 
	{M2, T/2}, 
	// ʡ�Ժ����������ݣ������Ȥ�Ķ��߲�������
	{ 0, 0} // ���� 
}; 

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void Buzzer_TEST_Configuration(void)
{
	SYS_Configuration();				//ϵͳ����
//	SysTick_Configuration(1000000);	//ϵͳ���ʱ������72MHz,��λΪuS
	
	PWM_OUT(TIM1,PWM_OUTChannel1,10,200);		//PWM�趨
	PWM_OUT(TIM2,PWM_OUTChannel1,1,500);		//sys_led
}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void Buzzer_TEST_Server(void)
{
//	TIM_Cmd(TIM1, DISABLE); 									//ʹ��TIM
	TheSysClock++;
	if(TheSysClock>=65536UL)
		TheSysClock=0;
	musicPlay(); 
}


// ����������ָ��Ƶ�ʵ�����
// usFreq�Ƿ���Ƶ�ʣ�ȡֵ (ϵͳʱ��/65536)+1 �� 20000����λ��Hz 
void buzzerSound(unsigned short usFreq) 
{ 
	unsigned long ulVal; 
	 
	if ((usFreq <= TheSysClock / 65536UL) || (usFreq > 20000)) 
	{ 
		buzzerQuiet();// ����������
	} 
	else 
	{ 
//		GPIOPinTypeTimer(CCP3_PORT, CCP3_PIN); // ������عܽ�ΪTimer���� 
		ulVal = TheSysClock / usFreq; 
//		TimerLoadSet(TIM1, TIMER_B, ulVal); // ����TimerB��ֵ 
//		TimerMatchSet(TIM1, TIMER_B, ulVal / 2); // ����TimerBƥ��ֵ 
		PWM_OUT(TIM1,PWM_OUTChannel1,ulVal,ulVal/2);		//PWM�趨
		TIM_Cmd(TIM1, ENABLE); // ʹ��TimerB���� 
	} 
}

// ������ֹͣ����
void buzzerQuiet(void) 
{ 
	TIM_Cmd(TIM1, DISABLE); 									//ʹ��TIM
//	TimerDisable(TIMER1_BASE, TIMER_B); // ��ֹTimerB���� 
//	GPIOPinTypeOut(CCP3_PORT, CCP3_PIN); // ����CCP3�ܽ�ΪGPIO��� 
//	GPIOPinWrite(CCP3_PORT, CCP3_PIN, 0x00); // ʹCCP3�ܽ�����͵�ƽ
}

//��������
void musicPlay(void) 
{ 
	short i = 0; 
	for (;;) 
	{ 
		if (MyScore[i].mTime == 0) break; 
			buzzerSound(MyScore[i].mName); 
//		SysCtlDelay(MyScore[i].mTime * (TheSysClock/ 3000)); 
		i++; 
//		buzzerQuiet( ); 
//		SysCtlDelay(10 * (TheSysClock/ 3000)); 
	} 
}




#endif