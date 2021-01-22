/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DSP_Kechkin_CM4.h"
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

extern Speed_structure speed;
    
unsigned short Device_ADC_Buf[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
Device_DAC_ON();

Device_ADC_Start(Device_ADC_Buf,3);

/*                                    ENCODER SPI-DMA INIT                          */
  HAL_SPI_Init(&hspi2);
  HAL_SPI_MspInit(&hspi2);
  HAL_NVIC_DisableIRQ(DMA1_Stream3_IRQn);
  HAL_NVIC_DisableIRQ(DMA1_Stream4_IRQn);
  HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)&speed.angle_f[0], 1);


HAL_Delay(600);

Device_MainTimerStart();
Device_PWM_Channels_ON();




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  //Device_DAC_Out_PA4_CH1(enc_angle/8200.0f);
  Device_On_Led(O_D_Led1_GPIO_Port,O_D_Led1_Pin);
  HAL_Delay(500);
  Device_Off_Led(O_D_Led1_GPIO_Port,O_D_Led1_Pin);
  HAL_Delay(500);
    
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Device_MainTimerStart(void)
{
HAL_TIM_Base_Start_IT(&htim1); 
}

void Device_MainTimerStop(void)
{
HAL_TIM_Base_Stop_IT(&htim1); 
}

void Device_DAC_ON(void)
{
  HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac,DAC_CHANNEL_2);
}
void Device_ADC_Start(unsigned short *Device_ADC_Buf,unsigned short Lenght)
{
  /*  x        1        x        x
ADC1  x,       iU,      x,       x.
ADC2  x,       iV,      x,       x.
ADC3  x,       iW,      x,       x.
*/
        __HAL_ADC_ENABLE(&hadc2);
	__HAL_ADC_ENABLE(&hadc3);
HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)Device_ADC_Buf, Lenght);
__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC);
__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);
__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TE);
__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_FE);
__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_DME);
HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);  
}

void Device_PWM_Channels_ON(void)
{
TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_ENABLE | TIM_CCxN_ENABLE);
TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_ENABLE | TIM_CCxN_ENABLE);
TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_3, TIM_CCx_ENABLE | TIM_CCxN_ENABLE);
__HAL_TIM_MOE_ENABLE(&htim1);
}

void Device_PWM_Channels_OFF(void)
{
TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_DISABLE | TIM_CCxN_DISABLE);
TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_DISABLE | TIM_CCxN_DISABLE);
TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_3, TIM_CCx_DISABLE | TIM_CCxN_DISABLE);  
__HAL_TIM_MOE_DISABLE(&htim1);
}



void Device_Toggle_Led(GPIO_TypeDef *LED_GPIO_Port, uint16_t LED_Pin)
{
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

void Device_On_Led(GPIO_TypeDef *LED_GPIO_Port, uint16_t LED_Pin)
{
  LED_GPIO_Port->BSRR = LED_Pin;
}

void Device_Off_Led(GPIO_TypeDef *LED_GPIO_Port, uint16_t LED_Pin)
{
  LED_GPIO_Port->BSRR = LED_Pin<<16;
}

void Device_DAC_Out_PA4_CH1(float PA4_ch1)
{
    HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(int)(PA4_ch1*4096.0f));
}
void Device_DAC_Out_PA5_CH2(float PA5_ch2)
{
    HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(int)(PA5_ch2*4096.0f));
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
