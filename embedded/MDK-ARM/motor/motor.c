#include "motor.h"
#include "oled.h"
Motor motorA, motorB;
void MOTOR_Init()
{
	HAL_TIM_Encoder_Start(&ENCODERA, TIM_CHANNEL_ALL);      //开启编码器定时器
    __HAL_TIM_ENABLE_IT(&ENCODERA,TIM_IT_UPDATE);           //开启编码器定时器更新中断,防溢出处理
	HAL_TIM_Encoder_Start(&ENCODERB, TIM_CHANNEL_ALL);      //开启编码器定时器
    __HAL_TIM_ENABLE_IT(&ENCODERB,TIM_IT_UPDATE);           //开启编码器定时器更新中断,防溢出处理
	HAL_TIM_Base_Start_IT(&BASEIT);                       //开启10ms定时器中断
	HAL_TIM_PWM_Start(&PWMIT, TIM_CHANNEL_1);            //开启PWM
	HAL_TIM_PWM_Start(&PWMIT, TIM_CHANNEL_2); 
	__HAL_TIM_SET_COUNTER(&ENCODERA, 10000);                //编码器定时器初始值设定为10000
	__HAL_TIM_SET_COUNTER(&ENCODERB, 10000);                //编码器定时器初始值设定为10000
	motorA.loopNum = 0;                                   //溢出计数
	motorA.lastAngle = 0;
	motorA.totalAngle = 0;
	motorB.loopNum = 0;                                   //溢出计数
	motorB.lastAngle = 0;
	motorB.totalAngle = 0;
}

void MOTOR_Start() {
	HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

void MOTOR_PosA() {
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}

void MOTOR_NegA() {
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
	
}

void MOTOR_PosB() {
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

void MOTOR_NegB() {
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
	
}

void MOTOR_STOP() {
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}

void MOTOR_Shutdown() {
	HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
}

void MOTOR_ControlA(uint16_t direction, uint16_t PWM) {
	switch(direction) {
		case 0:{MOTOR_PosA();
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM);
			break;
			}
		case 1:{MOTOR_NegA();
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM);
			break;
			}
		default: break;
	}
	
}

void MOTOR_ControlB(uint16_t direction, uint16_t PWM) {
	switch(direction) {
		case 0:{MOTOR_PosB();
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM);
			break;
			}
		case 1:{MOTOR_NegB();
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM);
			break;
			}
		default: break;
	}
}