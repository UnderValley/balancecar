#ifndef _motor_h_
#define _motor_h_

#include "main.h"
#include "tim.h"

#define ENCODERA htim1
#define ENCODERB htim3
#define PWMIT htim2
#define BASEIT htim4

#define RR 131u    //电机减速比
#define RELOADVALUEA __HAL_TIM_GetAutoreload(&ENCODERA)    //获取自动装载值,本例中为20000
#define COUNTERNUMA __HAL_TIM_GetCounter(&ENCODERA)        //获取编码器定时器中的计数值
#define RELOADVALUEB __HAL_TIM_GetAutoreload(&ENCODERB)    //获取自动装载值,本例中为20000
#define COUNTERNUMB __HAL_TIM_GetCounter(&ENCODERB)        //获取编码器定时器中的计数值
#define IN1(state) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,(GPIO_PinState)(state))    //M1
#define IN2(state) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,(GPIO_PinState)(state))    //M2
 
//电机结构体
typedef struct _Motor
{
	int32_t lastAngle;        //上10ms转过的角度
	int32_t totalAngle;       //总的角度
	int16_t loopNum;          //溢出次数计数值
	float speed;              //电机输出轴目前转速,单位为RPM
}Motor;

extern Motor motorA, motorB;

void MOTOR_Init();
void MOTOR_ControlA(uint16_t direction, uint16_t PWM);
void MOTOR_ControlB(uint16_t direction, uint16_t PWM);

void MOTOR_Start();

void MOTOR_STOP();
void MOTOR_Shutdown();

#endif

/*
//编码器测速的中断函数    修改自动重载值的时候需更改数值

__weak void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==BASEIT.Instance)		              //10ms中断
	{
		
		int16_t pluseA = COUNTERNUMA - RELOADVALUEA/2;
		int16_t pluseB = COUNTERNUMB - RELOADVALUEB/2;
		pluseA = pluseA > 0 ? pluseA : -pluseA;
		pluseB = pluseB > 0 ? pluseB : -pluseB;
        //从开始到现在当前10ms的总脉冲数							               									
		motorA.totalAngle = pluseA + (motorA.loopNum > 0 ? motorA.loopNum : -motorA.loopNum) * RELOADVALUEA/2; 
		motorB.totalAngle = pluseB + (motorB.loopNum > 0 ? motorB.loopNum : -motorB.loopNum) * RELOADVALUEB/2;
        //进行速度计算,根据前文所说的,4倍频,编码器13位,减速比30,再乘以6000即为每分钟输出轴多少转
        //motor.totalAngle - motor.lastAngle为当前10ms内的增量，即脉冲数
		motorA.speed = (float)(motorA.totalAngle - motorA.lastAngle)/(4*11*RR)*6000;
		motorB.speed = (float)(motorB.totalAngle - motorB.lastAngle)/(4*11*RR)*6000;
		if (motorA.speed < 0) motorA.speed = -motorA.speed;
		if (motorB.speed < 0) motorB.speed = -motorB.speed;
		motorA.lastAngle = motorA.totalAngle;              //更新转过的圈数
		motorB.lastAngle = motorB.totalAngle; 
	}
 
    //如果是编码器更新中断,即10ms内,脉冲数超过了计数范围,需要进行处理
	else if(htim->Instance == ENCODERA.Instance)            
	{
		if(COUNTERNUMA < 10000)	motorA.loopNum++;         //向上计数超过10000，正溢出+1
		else if(COUNTERNUMA > 10000)	motorA.loopNum--;     //向下计数小于0，负溢出+1
		__HAL_TIM_SetCounter(&ENCODERA, 10000);             //重新设定初始值		
	}
	
	else if(htim->Instance == ENCODERB.Instance)            
	{
		if(COUNTERNUMB < 10000)	motorB.loopNum++;         //向上计数超过10000，正溢出+1
		else if(COUNTERNUMB > 10000)	motorB.loopNum--;     //向下计数小于0，负溢出+1
		__HAL_TIM_SetCounter(&ENCODERB, 10000);             //重新设定初始值		
	}
}

*/