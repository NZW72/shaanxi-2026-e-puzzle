/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "COMMEND.h"
#include "A4988.h"
#include "delay.h"
#include "oled.h"
#include "stepp.h"
#include "key.h"
#include "stdio.h"
#include "switch.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int fputc(int ch, FILE *f)
{
  if (ch == '\n')
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)&"\r", 1, 1000);
  }
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
  return ch;
}

void led_buzzle(uint8_t a)
{
  if(a==0)
  {
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
  }
}

// 电磁铁控制 PB1
void redly_switch(uint8_t a)
{
  if(a==1)
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
  }
}
//LED报警  //根据他的蜂鸣器三极管类型切换逻辑
void LED(uint8_t a)
{
  if(a==1)
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
  }
}



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 每次移动时叠加的偏移量（单位：cm），用于补偿机械误差
#define X_MOVE_OFFSET  0.0f   // X轴每次移动额外偏移（可正可负）
#define Y_MOVE_OFFSET  0.5f   // Y轴每次移动额外偏移


//#define printf  motor_print
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define DEBUG_PRINT_ENABLE  0 // 设为 0 即可关闭所有调试 printf

#if DEBUG_PRINT_ENABLE
    // 开启调试时，正常使用 printf
    #define DEBUG_PRINT(...)   printf(__VA_ARGS__)
#else
    // 关闭调试时，将 printf 替换为空操作（参数不展开，无副作用）
    #define DEBUG_PRINT(...)   ((void)0)
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
motor_flag motor_flag1 = {0};   // 定义并初始化为0
int cm = 0;
uint8_t KEY_NUM = 0;
// 等待两个电机停止（带超时），返回1正常，0超时
uint8_t wait_motors_stop(uint32_t timeout_ms);
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  led_buzzle(0);
  Delay_init();


  A4988_Init();
  A4988_SetMicrostep(MICROSTEP_16);
  A4988_SetDirection(MOTOR_1, DIR_CW);
  A4988_SetDirection(MOTOR_2, DIR_CW);
//  A4988_SetPulsesPerCm(806.45f);
  A4988_res();
  stepper_init();
//  stepper_rotate(-90.0f, 1000);
  redly_switch(0);
  commend_Init();
	Myswitch_init();
	
	KEY_Init();
  My_Switch_down(800);//伸底
	My_Switch_up(800);//伸高
//	stepper_rotate(360, 1000);
	int i=0;
	float actual_y, actual_x;
	OLED_Init();
	OLED_ShowString(0,0,"OK",16);
	My_Switch_down(800);
	redly_switch(1);
	My_Switch_up(800);//伸高
	int flag=1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1) {
	if(flag==1)
	{
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) {
		   	OLED_Clear();
        printf("Y\r\n");              // 替换为 DEBUG_PRINT
        OLED_ShowString(0,0,"YY",16);
        HAL_Delay(1000);
		  	flag=2;
    }
	}
	if(flag==2){
		         
							for (i = 0; i < commend_count; i++) {
								  flag=3;
									DEBUG_PRINT("正在搬运第 %d 个碎片\r\n", i + 1);
									OLED_ShowString(0,0,"RUN",16);
									// ----- 抓取 -----
									DEBUG_PRINT("开始抓取\r\n");
									DEBUG_PRINT("抓取目标 -> MOTOR_2 Y: %.1f cm (原点偏移 %.1f), MOTOR_1 X: %.1f cm (原点偏移 %.1f)\r\n",
															my_data[i].y, Y_CALIB_DIST, my_data[i].x, X_CALIB_DIST);
									actual_y = A4988_MoveDistance(MOTOR_2, my_data[i].y + Y_MOVE_OFFSET);
									actual_x = A4988_MoveDistance(MOTOR_1, my_data[i].x + X_MOVE_OFFSET);
									DEBUG_PRINT("实际移动 -> Y: %.2f cm, X: %.2f cm\r\n", actual_y, actual_x);

									if (!wait_motors_stop(65535)) {
											DEBUG_PRINT("移动到抓取位置超时，跳过该碎片\r\n");
											continue;
									}
									// ----- 旋转 -----
									DEBUG_PRINT("开始旋转，角度: %.1f\r\n", my_data[i].z);

									My_Switch_down(800);
									// ----- 吸合 -----
									DEBUG_PRINT("开启吸取\r\n");
									redly_switch(1);
									HAL_Delay(500);
									DEBUG_PRINT("伸缩向上\r\n");
									My_Switch_up(800);
									HAL_Delay(500);
									stepper_rotate(my_data[i].z, 1000);
									// ----- 返回原点 -----
									DEBUG_PRINT("返回原点\r\n");
									A4988_res();
									if (!wait_motors_stop(65535)) {
											DEBUG_PRINT("复位超时，跳过该碎片\r\n");
											continue;
									}
									// ----- 释放位置 -----
									DEBUG_PRINT("移动到释放位置 -> MOTOR_2 get_y: %.1f (偏移 %.1f), MOTOR_1 get_x: %.1f (偏移 %.1f)\r\n",
															my_data[i].get_y, Y_CALIB_DIST, my_data[i].get_x, X_CALIB_DIST);
									actual_y = A4988_MoveDistance(MOTOR_2, my_data[i].get_y + Y_MOVE_OFFSET);
									actual_x = A4988_MoveDistance(MOTOR_1, my_data[i].get_x + X_MOVE_OFFSET);
									DEBUG_PRINT("实际移动 -> Y: %.2f cm, X: %.2f cm\r\n", actual_y, actual_x);
									if (!wait_motors_stop(65535)) {
											DEBUG_PRINT("移动到释放位置超时，跳过该碎片\r\n");
											continue;
									}
									DEBUG_PRINT("到达指定距离\r\n");
									// ----- 释放 -----
									DEBUG_PRINT("放置碎片\r\n");
									DEBUG_PRINT("伸缩向下\r\n");
									My_Switch_down(800);
									redly_switch(0);
									HAL_Delay(500);
									// ----- 最终归零 -----
									DEBUG_PRINT("返回原点\r\n");
									My_Switch_up(800);
									stepper_rotate(-my_data[i].z, 1000);
									A4988_res();
									if (!wait_motors_stop(65535)) {
											DEBUG_PRINT("最终复位超时\r\n");
									}
							  }

	            }
		
		if(flag==3)
		{
			OLED_Clear();
      OLED_ShowString(0,0,"over",16);
			led_buzzle(1);
			HAL_Delay(500);
			led_buzzle(0);
			led_buzzle(1);
			HAL_Delay(500);
			led_buzzle(0);
			flag=1;
			commend_count=0;
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
// 等待两个电机停止，超时返回0，正常返回1
uint8_t wait_motors_stop(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  while (1) {
    if (motor_flag1.motor1_flag == 0 && motor_flag1.motor2_flag == 0) {
      return 1;   // 正常停止
    }
    if (HAL_GetTick() - start > timeout_ms) {
      // 超时强制停止
      A4988_StopMotor(MOTOR_1);
      A4988_StopMotor(MOTOR_2);
      printf("Motor stop timeout!\r\n");
      return 0;   // 超时
    }
    HAL_Delay(10);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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