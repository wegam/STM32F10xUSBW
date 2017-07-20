#ifdef PWM_TEST
#include "PWM_TEST.H"

#include "STM32_SYS.H"

/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void PWM_TEST_Configuration(void)
{
	SYS_Configuration();				//系统配置
//	SysTick_Configuration(10000);	//系统嘀嗒时钟配置72MHz,单位为uS
	
//	GPIO_Configuration0();
//	TIM_Configuration(TIM1,7200,3000);	//定时时间设定
//	PWM_Configuration(TIM2,7200,10000,51);
//	PWM_OUT(TIM1,PWM_OUTChannel1,20000,50);		//PWM设定
//	PWM_OUT(TIM2,PWM_OUTChannel1,5,100);	//PWM设定
//	PWM_OUT(TIM2,PWM_OUTChannel1,5,500);		//sys_led
	PWM_OUT(TIM1,PWM_OUTChannel1,10,100);		//PWM设定
	PWM_OUT(TIM2,PWM_OUTChannel1,5,500);		//sys_led
//	PWM_OUT(TIM3,PWM_OUTChannel1,20000,30);		//PWM设定
//	PWM_OUT(TIM4,PWM_OUTChannel1,20000,40);		//PWM设定
//	
//	PWM_OUT(TIM1,PWM_OUTChannel2,20000,50);		//PWM设定
//	PWM_OUT(TIM2,PWM_OUTChannel2,20000,500);	//PWM设定

}
/*******************************************************************************
* 函数名		:	
* 功能描述	:	 
* 输入		:	
* 输出		:
* 返回 		:
*******************************************************************************/
void PWM_TEST_Server(void)
{
	

}





#endif
