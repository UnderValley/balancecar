/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdio.h"
#include "math.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "motor.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFFERSIZE  256     //最大接收字节数

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char RxBuffer[RXBUFFERSIZE];   //接收数据
uint8_t aRxBuffer;			//接收中断缓冲
uint8_t Uart1_Rx_Cnt = 0;		//接收缓冲计数

float pitch,roll,yaw; 		    //欧拉角
short aacx,aacy,aacz;			//加速度传感器原始数据
short gyrox,gyroy,gyroz;		//陀螺仪原始数据
float temp;					    //温度

float_t Vertical_Kp, Vertical_Kd;//直立环p,d参数
double_t Velocity_Kp, Velocity_Ki;//速度环p,i参数

int16_t ENC_CNTA, ENC_CNTB;
int16_t ENC_CNTA_Last, ENC_CNTB_Last;
int16_t errorA, errorB;

int8_t usart_flag = 0;			//usart中使用的标
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int Vertical(float Med,float Angle,float gyro_X);
int Velocity(int Target,int encoder_left,int encoder_right);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	uint8_t flag;
	flag = 0;
	int16_t PWM_out, Velocity_out, Vertical_out;
	Vertical_Kp = 105 * 0.6;	// 	105 * 0.6
	Vertical_Kd = 0.4 * 0.6;		//0.4 * 0.6
	Velocity_Kp = 5500/100000.0;    //    5500/100000.0
	Velocity_Ki = Velocity_Kp / 200;
	ENC_CNTA_Last = ENC_CNTB_Last = 0;
	usart_flag = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  MPU_Init();			//MPU6050初始化
  mpu_dmp_init();		//dmp初始化
  MOTOR_Init();
  MOTOR_Start();
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
	/*mpu6050读数*/
	while(mpu_dmp_get_data(&pitch, &roll, &yaw));	//必须要用while等待，才能读取成功
        MPU_Get_Accelerometer(&aacx,&aacy, &aacz);		//得到加速度传感器数据
        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);		//得到陀螺仪数据
//        temp=MPU_Get_Temperature();						//得到温度信息

	  /*显示动画*/
//	  if(fabs(pitch) > 10 && flag != 0) {
//		  flag = 0;
//		  OLED_Clear();
//		  OLED_ShowCHinese(30, 3, 0);
//		  OLED_ShowCHinese(30+16, 3, 1);
//		  OLED_ShowCHinese(30+32, 3, 2);
//		  OLED_ShowCHinese(30+48, 3, 3);
//		  OLED_ShowCHinese(30+64, 3, 4);
//	  }else if (fabs(pitch) < 10 && flag != 1) {
//		  flag = 1;
//		  OLED_Clear();
//		  OLED_ShowCHinese(40, 3, 6);
//		  OLED_ShowCHinese(40+16, 3, 7);
//		  OLED_ShowCHinese(40+32, 3, 8);
//	  }


	Velocity_out=Velocity(0, errorA, errorB);	//速度环
	PWM_out = Vertical(Velocity_out + 0, pitch, gyroy);// 直立环
	if (fabs(PWM_out) > 1000) PWM_out = 1000 * (PWM_out > 0 ? 1 : -1);
	if (PWM_out < 0) {
		MOTOR_ControlA(1, -PWM_out * 1.25);
		MOTOR_ControlB(1, -PWM_out);
	}else {
		MOTOR_ControlA(0, PWM_out * 1.25);
		MOTOR_ControlB(0, PWM_out);
	}
	
	OLED_ShowSighedNum(8, 0, (int32_t) errorA, 5, 16);
	OLED_ShowSighedNum(69, 0, (int32_t) errorB, 5, 16);
	OLED_ShowSighedNum(8, 2, (int32_t) Vertical_Kp, 5, 16);
	OLED_ShowSighedNum(69, 2, (int32_t) (Vertical_Kd * 10), 5, 16);
	OLED_ShowSighedNum(8, 4, (int32_t) (Velocity_Kp * 100000), 5, 16);
	OLED_ShowSighedNum(69, 4, (int32_t) (Velocity_Ki * 10000000), 5, 16);
	OLED_ShowSighedNum(8, 6, (int32_t) PWM_out, 5, 16);
	OLED_ShowSighedNum(69, 6, (int32_t) pitch, 5, 16);
	
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*********************
直立环PD控制器：Kp*Ek+Kd*Ek_D

入口：期望角度、真实角度、真实角速度
出口：直立环输出
*********************/
int Vertical(float Med,float Angle,float gyro_X)
{
	int PWM_out;
	
	PWM_out=Vertical_Kp*(Angle-Med)+Vertical_Kd*(gyro_X-0);
	return PWM_out;
}

/*********************
速度环PI：Kp*Ek+Ki*Ek_S
*********************/
int Velocity(int Target,int encoder_left,int encoder_right)
{
	static int Encoder_S,EnC_Err_Lowout_last,PWM_out,Encoder_Err,EnC_Err_Lowout;
	float a=0.7;
	
	//1.计算速度偏差
	Encoder_Err=((encoder_left+encoder_right)-Target);//舍去误差--我的理解：能够让速度为"0"的角度，就是机械中值。
	//2.对速度偏差进行低通滤波
	//low_out=(1-a)*Ek+a*low_out_last;
	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//使得波形更加平滑，滤除高频干扰，防止速度突变。
	EnC_Err_Lowout_last=EnC_Err_Lowout;//防止速度过大的影响直立环的正常工作。
	//3.对速度偏差积分，积分出位移
	Encoder_S+=EnC_Err_Lowout;
	//4.积分限幅
	Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
	
	//5.速度环控制输出计算
	PWM_out=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;
	return PWM_out;
}



/*串口中断回调函数*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	
	
	if(Uart1_Rx_Cnt >= 255)  //溢出判断
	{
		Uart1_Rx_Cnt = 0;
		memset(RxBuffer,0x00,sizeof(RxBuffer));
		HAL_UART_Transmit(&huart2, (uint8_t *)"数据溢出", 10,0xFFFF); 	
        
	}
	else
	{
		RxBuffer[Uart1_Rx_Cnt++] = aRxBuffer;   //接收数据转存
		if((Uart1_Rx_Cnt >= 2) && (RxBuffer[Uart1_Rx_Cnt-1] == 0x0A)&&(RxBuffer[Uart1_Rx_Cnt-2] == 0x0D)) //判断结束位
		{
			Uart1_Rx_Cnt = 0;
			memset(RxBuffer,0x00,sizeof(RxBuffer));
			if (usart_flag == 1) {
				Velocity_Kp /= 100000.0;
//				Velocity_Kp = -Velocity_Kp;
				Velocity_Ki = Velocity_Kp / 200.0;
			}
			if (usart_flag == 2) {
				Vertical_Kd /= 10.0;
			}
			usart_flag = 0;
		}else if (usart_flag == 0 && strcmp(RxBuffer, "kp") == 0) {
			usart_flag = 1;
			Velocity_Kp = 0;
//			Vertical_Kp = 0;
		}else if (usart_flag == 1 && aRxBuffer >= '0' && aRxBuffer <= '9') {
//			Vertical_Kp = 10 * Vertical_Kp + aRxBuffer - '0';
			Velocity_Kp = 10 * Velocity_Kp + aRxBuffer - '0';			
		}
		else if (usart_flag == 0 && strcmp(RxBuffer, "kd") == 0) {
			usart_flag = 2;
			Vertical_Kd = 0;
		}else if (usart_flag == 2 && aRxBuffer >= '0' && aRxBuffer <= '9') {
			Vertical_Kd = 10 * Vertical_Kd + aRxBuffer - '0';			
		}
	
		
	}
	
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer, 1);   //再开启接收中断
}

/*编码器中断回调函数*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==BASEIT.Instance)		              //10ms中断
	{
//		printf("1\n");
		ENC_CNTA = COUNTERNUMA - RELOADVALUEA/2;
		ENC_CNTB = COUNTERNUMB - RELOADVALUEB/2;
		errorA = ENC_CNTA - ENC_CNTA_Last;
		errorB = ENC_CNTB - ENC_CNTB_Last;
		ENC_CNTA_Last = ENC_CNTA;
		ENC_CNTB_Last = ENC_CNTB;
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

/*外部中断回调函数*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
