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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nop.h"
#include "struct_typedef.h"
#include "OLED.h"
#include "pid.h"
#include "filter.h"
#include "maths.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern vu16 AD_Value[ADC_SAMPLE_PNUM][ADC_SAMPLE_CNUM];
extern vu16  After_filter[ADC_SAMPLE_CNUM];  //用来存放平均值之后的结果
detection_volume_t Vac;
detection_volume_t ACcur;
extern float M;
sliding_mean_filter_type_t acmean_filter;
sliding_mean_filter_type_t vcmean_filter;
float acAtemp;
float acVtemp;
//first_order_filter_type_t low_past;
//fp32 low_num=10;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t test_val;
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
//PidInit(&Vac.current_pid,0.17,0.06,0.2,Integral_Limit|Output_Limit);//电流环PID初始化
//PidInitMode(&Vac.current_pid,Integral_Limit,4.30,0);//电流环PID模式初始化
//PidInitMode(&Vac.current_pid,Output_Limit,0.36,-0.5);//电压环PID模式初始化

//PidInit(&Vac.VIN_pid,0.15,0.012,0.1,Integral_Limit);//电压环PID初始化
//PidInitMode(&Vac.VIN_pid,Integral_Limit,20,0);

PidInit(&Vac.VIN_pid,0.02,0.0038,0.01,Integral_Limit|Output_Limit);//电压环PID初始化
PidInitMode(&Vac.VIN_pid,Integral_Limit,65,0);
PidInitMode(&Vac.VIN_pid,Output_Limit,0.35,-0.5);//电压环PID模式初始化

sliding_mean_filter_init(&acmean_filter);//电流检测滑动均值滤波初始化
sliding_mean_filter_init(&vcmean_filter);//电压检测滑动均值滤波初始化



//first_order_filter_init(&low_past,low_num);

OLED_Init();
HAL_TIM_Base_Start_IT(&htim2);//开启定时器2中断
HAL_TIM_PWM_Start (&htim1,TIM_CHANNEL_1);//开启四路pwm波
HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_1);

HAL_TIM_PWM_Start (&htim1,TIM_CHANNEL_2);
HAL_TIMEx_PWMN_Start (&htim1,TIM_CHANNEL_2);

//__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1799);
HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&AD_Value, ADC_SAMPLE_PNUM*ADC_SAMPLE_CNUM);
OLED_ShowString(1,1,"START:");
//OLED_ShowString(2,1,"O1:0.00");
OLED_ShowString(2,1,"M:0.0");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    //交流电流采样
		acAtemp=(float)ADC_Filter(2)* (3.3 / 4096);
		Vac.curtemp=sliding_mean_filter(&acmean_filter,acAtemp,10);
		Vac.ACcurrent=(Vac.curtemp-1.84)/0.6058;
		//if(Vac.ACcurrent>2.5) HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_RESET);
		
		//交流电压采样
		acVtemp=(float)ADC_Filter(0)*(3.3/4096);
		Vac.temp=sliding_mean_filter(&vcmean_filter,acVtemp,10);
		
		
		if(test_val==1)
		{
			
			if(Vac.ACcurrent<0.4) 
			{
				M=0.8;
			}
			else 
			{	Vac.ACvoltage=102.19*(Vac.temp*Vac.temp)-524.18*Vac.temp+692.24;
				M=0.6 + single_loop(&Vac.VIN_pid,24,Vac.ACvoltage);
			}
		}
		else 
		{
			Vac.VIN_pid.Ierror=0;
			//Vac.current_pid.Ierror=0;
			M=0.7;
		}
		
		
    
		OLED_ShowNum(2,3,M,1);//调制比显示
		OLED_ShowNum(2,5,(uint16_t)(M * 1000) % 1000,3);//
		
		

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
