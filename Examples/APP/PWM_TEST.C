#ifdef PWM_TEST
#include "PWM_TEST.H"

#include "STM32_SYS.H"

/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PWM_TEST_Configuration(void)
{
	SYS_Configuration();				//ϵͳ����
//	SysTick_Configuration(10000);	//ϵͳ���ʱ������72MHz,��λΪuS
	
//	GPIO_Configuration0();
//	TIM_Configuration(TIM1,7200,3000);	//��ʱʱ���趨
//	PWM_Configuration(TIM2,7200,10000,51);
//	PWM_OUT(TIM1,PWM_OUTChannel1,20000,50);		//PWM�趨
//	PWM_OUT(TIM2,PWM_OUTChannel1,5,100);	//PWM�趨
//	PWM_OUT(TIM2,PWM_OUTChannel1,5,500);		//sys_led
	PWM_OUT(TIM1,PWM_OUTChannel1,10,100);		//PWM�趨
	PWM_OUT(TIM2,PWM_OUTChannel1,5,500);		//sys_led
//	PWM_OUT(TIM3,PWM_OUTChannel1,20000,30);		//PWM�趨
//	PWM_OUT(TIM4,PWM_OUTChannel1,20000,40);		//PWM�趨
//	
//	PWM_OUT(TIM1,PWM_OUTChannel2,20000,50);		//PWM�趨
//	PWM_OUT(TIM2,PWM_OUTChannel2,20000,500);	//PWM�趨

}
/*******************************************************************************
* ������		:	
* ��������	:	 
* ����		:	
* ���		:
* ���� 		:
*******************************************************************************/
void PWM_TEST_Server(void)
{
	

}





#endif