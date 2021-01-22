/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IV_Pin GPIO_PIN_1
#define IV_GPIO_Port GPIOC
#define IW_Pin GPIO_PIN_2
#define IW_GPIO_Port GPIOC
#define IU_Pin GPIO_PIN_1
#define IU_GPIO_Port GPIOA
#define ID_Pin GPIO_PIN_3
#define ID_GPIO_Port GPIOA
#define FAULT_3_Pin GPIO_PIN_6
#define FAULT_3_GPIO_Port GPIOA
#define RESET_3_Pin GPIO_PIN_5
#define RESET_3_GPIO_Port GPIOC
#define VL_Pin GPIO_PIN_0
#define VL_GPIO_Port GPIOB
#define WL_Pin GPIO_PIN_1
#define WL_GPIO_Port GPIOB
#define FAULT_2_Pin GPIO_PIN_7
#define FAULT_2_GPIO_Port GPIOE
#define UL_Pin GPIO_PIN_8
#define UL_GPIO_Port GPIOE
#define UH_Pin GPIO_PIN_9
#define UH_GPIO_Port GPIOE
#define VH_Pin GPIO_PIN_11
#define VH_GPIO_Port GPIOE
#define WH_Pin GPIO_PIN_13
#define WH_GPIO_Port GPIOE
#define RESET_2_Pin GPIO_PIN_15
#define RESET_2_GPIO_Port GPIOE
#define FAULT_1_Pin GPIO_PIN_11
#define FAULT_1_GPIO_Port GPIOB
#define RESET_1_Pin GPIO_PIN_13
#define RESET_1_GPIO_Port GPIOB
#define O_D_Led1_Pin GPIO_PIN_12
#define O_D_Led1_GPIO_Port GPIOD
#define O_D_Led2_Pin GPIO_PIN_13
#define O_D_Led2_GPIO_Port GPIOD
#define O_D_Led3_Pin GPIO_PIN_14
#define O_D_Led3_GPIO_Port GPIOD
#define O_D_Led4_Pin GPIO_PIN_15
#define O_D_Led4_GPIO_Port GPIOD
#define LED_4_Pin GPIO_PIN_6
#define LED_4_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_8
#define LED_3_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_9
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_8
#define LED_2_GPIO_Port GPIOA
#define KM_Pin GPIO_PIN_10
#define KM_GPIO_Port GPIOC
#define QF_Pin GPIO_PIN_12
#define QF_GPIO_Port GPIOC
#define SB_1_Pin GPIO_PIN_0
#define SB_1_GPIO_Port GPIOD
#define SB_2_Pin GPIO_PIN_2
#define SB_2_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */
void Device_MainTimerStart(void);
void Device_MainTimerStop(void);
void Device_ADC_Start(unsigned short *Device_ADC_Buf,unsigned short Lenght);
void Device_SPI_ADC128_Start(unsigned short *Device_SPI_Buf);
void Device_PWM_Channels_ON(void);
void Device_PWM_Channels_OFF(void);
unsigned short Device_Invertor_Fault_Detect(void);
void Device_Rele_ON(void);
void Device_Rele_OFF(void);
void Device_Toggle_Led(GPIO_TypeDef *LED_GPIO_Port, uint16_t LED_Pin);
void Device_On_Led(GPIO_TypeDef *LED_GPIO_Port, uint16_t LED_Pin);
void Device_Off_Led(GPIO_TypeDef *LED_GPIO_Port, uint16_t LED_Pin);
void Device_DAC_Out_PA4_CH1(float PA4_ch1);
void Device_DAC_Out_PA5_CH2(float PA5_ch2);
void Device_DAC_ON(void);
void Device_CAN_Start(void);
void Device_CAN_Start_Send(void);

extern unsigned short Device_ADC_Buf[3];
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
