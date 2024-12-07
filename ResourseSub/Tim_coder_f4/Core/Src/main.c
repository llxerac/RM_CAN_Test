/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "pid.h"
#include "Motor.h"
#include "math.h"
#include "oled.h"
#include "../../../Hardware/MPU6050/delay.h"
#include "../../../Hardware/MPU6050/mpu6050.h"
#include "../../../Hardware/MPU6050/eMPL/inv_mpu.h"
#include "../../../Hardware/MPU6050/eMPL/inv_mpu_dmp_motion_driver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ABS(x) ((x)>=0?(x):-(x))

#define ENCODER_RESOLUTION 11    /*编码器一圈的物理脉冲数*/
#define ENCODER_MULTIPLE 4       /*编码器倍频，通过定时器的编码器模式设置*/
#define MOTOR_REDUCTION_RATIO 9.6 /*电机的减速比*/
/*电机转一圈总的脉冲数(定时器能读到的脉冲数) = 编码器物理脉冲数*编码器倍频*电机减速比 */
#define TOTAL_RESOLUTION ( ENCODER_RESOLUTION*ENCODER_MULTIPLE*MOTOR_REDUCTION_RATIO ) 
#define car_num 0
#define mode_num 0

//#define IO1_read() HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)
//#define IO2_read() HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)
//#define IO3_read() HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0)
//#define IO4_read() HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Rxbuf;
uint8_t Rx_speed[6];
uint8_t Rx_angle[3];
int angle_int;
int sp1, sp2;
uint8_t judge = 0;
uint16_t set_pwm[4];
float speed[4];
int CaptureNumber[4];
PID_InitDefStruct pid[4];

 uint8_t i;
float pitch,roll,yaw; 		//欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据
short temp;			

float rotateSpeed[4] = {0};
int encoderNum[4] = {0};
uint8_t ds_fm = 66;

uint16_t rx1;
uint16_t rx2;
uint16_t rx3;
uint16_t rx4;
uint16_t LorR = 0;
uint16_t sensor_left = 0;
uint16_t sensor_right = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void set_speed(int sp)
{
	for(int i = 0; i < 4; i++)
	{
		set_pwm[i] = Velocity_PID(sp, speed[i], pid + i);
		printf("PWM%d:%d\r\n", i, set_pwm[i]);
	}
	Tim_setCompare(set_pwm[3], set_pwm[2], set_pwm[1], set_pwm[0]);
	Direction_forward();
}

void turn_angle_speed(int sp1, int sp2)
{

		set_pwm[0] = Velocity_PID(sp1, speed[0], pid + 0);
		set_pwm[3] = Velocity_PID(sp1, speed[3], pid + 3);
		set_pwm[1] = Velocity_PID(sp2, speed[1], pid + 1);
		set_pwm[2] = Velocity_PID(sp2, speed[2], pid + 2);
}
//
void Delay_us(uint32_t t)//延时一微秒
{   
    int n  =  11;
    while(--t)
    {
        while(--n);
        n = 11;
    }
}


void gases_start_signal(void)//Trig发送触发信号
{  
 
 
  HAL_GPIO_WritePin(GPIOE , GPIO_PIN_4 , GPIO_PIN_SET);
  
  Delay_us(20);//延时20微秒
	
  HAL_GPIO_WritePin(GPIOE , GPIO_PIN_4, GPIO_PIN_RESET);
}

void distance_judge(void)
{
	if(ds_fm < 15)
	{
		Tim_setCompare(0, 0, 0, 0);
		HAL_Delay(2000);
	}
}

static int* read_encoder(void)
{
	
	encoderNum[0] = (int)((int16_t)(TIM3->CNT)); 
	encoderNum[1] = (int)((int16_t)(TIM4->CNT)); 
	encoderNum[2] = (int)((int16_t)(TIM5->CNT)); 
	encoderNum[3] = (int)((int16_t)(TIM8->CNT)); /*CNT为uint32, 转为int16*/
	__HAL_TIM_SetCounter(&htim3, 0);
	__HAL_TIM_SetCounter(&htim4, 0);
	__HAL_TIM_SetCounter(&htim5, 0);
	__HAL_TIM_SetCounter(&htim8, 0);/*CNT设初值*/
	return encoderNum;
}

//计算电机转速（被另一个定时器100ms调用1次）
void calc_motor_rotate_speed()
{
	int* encoderNum;
	/*读取编码器的值，正负代表旋转方向*/
	encoderNum = read_encoder();
	/* 转速(1秒钟转多少圈)=单位时间内的计数值/总分辨率*时间系数 */
	rotateSpeed[0] = ABS((float)encoderNum[0] * 3.14f * 6.8f * 1.2f/TOTAL_RESOLUTION*5);
	rotateSpeed[1] = ABS((float)encoderNum[1] * 3.14f * 6.8f * 1.2f/TOTAL_RESOLUTION*5);
	rotateSpeed[2] = ABS((float)encoderNum[2] * 3.14f * 6.8f * 1.3f/TOTAL_RESOLUTION*5);
	rotateSpeed[3] = ABS((float)encoderNum[3] * 3.14f * 6.8f * 1.3f/TOTAL_RESOLUTION*5);
	printf("encoder: %d\t speed:%.2f cm/s\r\n",encoderNum[0],rotateSpeed[0]);
	printf("encoder1: %d\t speed:%.2f cm/s\r\n",encoderNum[1],rotateSpeed[1]);
	printf("encoder2: %d\t speed:%.2f cm/s\r\n",encoderNum[2],rotateSpeed[2]);
	printf("encoder3: %d\t speed:%.2f cm/s\r\n",encoderNum[3],rotateSpeed[3]);

}

void tx_delay_ms(uint16_t nms)	//??0-8191ms
{
		if(__HAL_TIM_GetCounter(&htim7) > nms)
		{
			calc_motor_rotate_speed();
			__HAL_TIM_SetCounter(&htim7, 0);
		}//????8KHz,8???1ms
		/* Disable the Peripheral */
		
//			CaptureNumber[0]= (short)TIM3 -> CNT;
//			CaptureNumber[1]= (short)TIM4 -> CNT;
//			CaptureNumber[2]= (short)TIM5 -> CNT;
//			CaptureNumber[3]= (short)TIM8 -> CNT;
//			//uint32->int16可以有正负了，表正传和反转
//			printf("count1:%d count2:%d count3:%d count4:%d\r\n",CaptureNumber[0],CaptureNumber[1],CaptureNumber[2],CaptureNumber[3]);
//			TIM3 -> CNT=0;
//			TIM4 -> CNT=0;
//			TIM5 -> CNT=0;
//			TIM8 -> CNT=0;
//	
//			for(int i = 0; i < 4; i++)
//			{
//				speed[i] = ABS(CaptureNumber[i] * 1.0 * 3.14 * 6.8 / (11 * 4 * 34) * 100.0 );
//				printf("speed%d:%.1f\r\n", i, speed[i]);
//			}
}

void OLED_Show(void)
{
			OLED_ShowString(20,8,"speed1:", 8, 1);
			OLED_ShowNum(60,8,rotateSpeed[0],3,8,1);
			OLED_ShowString(20,20,"speed2:", 8, 1);
			OLED_ShowNum(60,20,rotateSpeed[1],3,8,1);
			OLED_ShowString(20,32,"speed3:", 8, 1);
			OLED_ShowNum(60,32,rotateSpeed[2],3,8,1);
			OLED_ShowString(20,44,"speed4:", 8, 1);
			OLED_ShowNum(60,44,rotateSpeed[3],3,8,1);
}

void MPU_Show(void)
{
	if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
	{
	//			temp=MPU_Get_Temperature();								//得到温度值
				MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
				MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
				printf("三轴角度：%f-%f-%f\r\n",pitch,roll,yaw);
	//			printf("三轴加速度：%d-%d-%d\r\n",aacx,aacy,aacz);
	//			printf("三轴角角度：%d-%d-%d\r\n",gyrox,gyroy,gyroz);
				OLED_DisPlay_On();
//				OLED_ShowString(20,8,"aacx:", 12, 1);
//				OLED_ShowNum(60,8,pitch,2,12,1);
//				OLED_ShowString(20,20,"aacy:", 12, 1);
//				OLED_ShowNum(60,20,roll,2,12,1);
//				OLED_ShowString(20,32,"aacz:", 12, 1);
//				OLED_ShowNum(60,32,yaw,2,12,1);
//				OLED_Refresh();
	}
}

void Rx_control(uint16_t forward_speed)
{
	if(Rxbuf == 'a')
		{
//			for(int i = 0; i < 4; i++)
//			{
//				set_pwm[i] = Velocity_PID(5, speed[i], pid + i);
//				printf("PWM%d:%d\r\n", i, set_pwm[i]);
//			}
//			Tim_setCompare(set_pwm[3], set_pwm[2], set_pwm[1], set_pwm[0]);
			Tim_setCompare(forward_speed, forward_speed, forward_speed, forward_speed);
			Direction_forward();
		}
		else if(Rxbuf == 'c' )
		{
			Tim_setCompare(1000, 2000, 2000, 1000); 
			Direction_left();
		}
		else if(Rxbuf == 'd')
		{
				Tim_setCompare(2000, 1000, 1000, 2000);
				Direction_right();
		}
		if(Rxbuf == 'e')
		{
			Tim_setCompare(0, 0, 0, 0);
			Direction_forward();
			HAL_Delay(3000);
		}
}

void sensor_control(uint16_t forward_speed)
{
		rx1 = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0);
		rx2 = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_1);
		rx3 = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2);
		rx4 = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3);
	if(rx2 == 1 && rx1 == 0 && rx3 == 0 && rx4 == 1)
	{
		Tim_setCompare(forward_speed, forward_speed, forward_speed, forward_speed);
		Direction_forward();
		//LorR = 0;
		sensor_left = 0;
		sensor_right = 0;
	}
	else if(rx2 == 1 && rx1 == 0 && rx3 == 1 && rx4 == 1)
	{
		Tim_setCompare(600, 2000, 2000, 600); 
		Direction_left();
		sensor_left = 0;
		//LorR = 1;
	}
	else if(rx2 == 0 && rx1 == 1 && rx3 == 1 && rx4 == 1)
	{
		Tim_setCompare(1000, 2000, 2000, 1000); 
		Direction_left();
		sensor_left = 1;
	}
	else if(rx2 == 1 && rx1 == 1 && rx3 == 0 && rx4 == 1)
	{
		Tim_setCompare(2000, 600, 600, 2000);
		Direction_right();
		sensor_right = 0;
		//LorR = 2;
	}
	else if(rx2 == 1 && rx1 == 1 && rx3 == 1 && rx4 == 0)
	{
		Tim_setCompare(2000, 1000, 1000, 2000);
		Direction_right();
		sensor_right = 1;
	}
	else if(rx2 == 1 && rx1 == 1 && rx3 == 1 && rx4 == 1)
	{
//		if(LorR == 0)
//		{
//			Tim_setCompare(forward_speed, forward_speed, forward_speed, forward_speed);
//			Direction_forward();
//		}
//		else if(LorR == 1)
//		{
//			Tim_setCompare(1000, 2000, 2000, 1000); 
//			Direction_left();
//		}
//		else if(LorR == 2)
//		{
//			Tim_setCompare(2000, 1000, 1000, 2000);
//			Direction_right();
//		}
		if(sensor_left == 1 && sensor_right == 0)
		{
			Tim_setCompare(1000, 2000, 2000, 1000); 
			Direction_left();
		}
		else if(sensor_left == 0 && sensor_right == 1)
		{
			Tim_setCompare(2000, 1000, 1000, 2000);
			Direction_right();
		}
		else
		{
			Tim_setCompare(forward_speed, forward_speed, forward_speed, forward_speed);
			Direction_forward();
		}
	}
	else
	{
		Tim_setCompare(forward_speed, forward_speed, forward_speed, forward_speed);
		Direction_forward();
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_UART5_Init();
  MX_TIM9_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1 | TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	
	HAL_TIM_IC_Start_IT(&htim9,TIM_CHANNEL_1);
	//HAL_TIM_Base_Start_IT(&htim7);
	
	__HAL_TIM_SetCounter(&htim7, 0);//htim1
	__HAL_TIM_ENABLE(&htim7);
//	uint32_t mpu6050_tick=HAL_GetTick();
	printf("start\r\n");
	//while(MPU_Init());
//	{
//		OLED_Refresh();
//		OLED_ShowString(20,8,"init_Failed:", 11, 1);
//	}
	printf("init complete\r\n");
	while(mpu_dmp_init())
	{
		delay_ms(200);
		printf("%s\r\n","Mpu6050 Init Wrong!");
	}
	printf("%s\r\n","Mpu6050 Init OK!");

	for(int i = 0; i < 4; i++)
	{
			PID_Init(pid + i);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		HAL_UART_Transmit(&huart5, (uint8_t *)"h", 1, 0xFFFF);
		OLED_Refresh();
		if(car_num == 0)
		{
			if(mode_num == 0)
			{
				gases_start_signal();
				distance_judge();
				sensor_control(500);
				printf("sensor1:%d,sensor2:%d,sensor3:%d,sensor4:%d\r\n",rx1, rx2, rx3, rx4);
				HAL_UART_Receive(&huart3,&Rxbuf,1, 200);
				//Rx_control(500);
				tx_delay_ms(100);
				MPU_Show();
				OLED_Show();
			}
			else if(mode_num == 1)
			{
				gases_start_signal();
				distance_judge();
				HAL_UART_Receive(&huart3,&Rxbuf,1, 200);
				Rx_control(633);
				tx_delay_ms(100);
				MPU_Show();
				OLED_Show();
			}
		}
		else
		{
			if(mode_num == 0)
			{
				gases_start_signal();
				distance_judge();
				sensor_control(500);
				HAL_UART_Receive(&huart3,&Rxbuf,1, 200);
				Rx_control(750);
				tx_delay_ms(100);
				MPU_Show();
				OLED_Show();
			}
			else if(mode_num == 1)
			{
				int distance_on = 0;
				gases_start_signal();
				distance_judge();
				HAL_UART_Receive(&huart3,&Rxbuf,1, 200);
				if(distance_on == 0)
				{
					Rx_control(900);
					if(ds_fm >= 18 && ds_fm <= 22)
					{
						Rx_control(633);
						distance_on = 1;
					}
				}
				else
				{
					Rx_control(633);
				}
				tx_delay_ms(100);
				MPU_Show();
				OLED_Show();
			}
		}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
