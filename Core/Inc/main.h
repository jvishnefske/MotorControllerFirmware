/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
struct p_HW{
    ADC_HandleTypeDef *adc1;
    ADC_HandleTypeDef *adc2;
    ADC_HandleTypeDef *adc3;
    CAN_HandleTypeDef *can1;
    I2C_HandleTypeDef *i2c1;
    I2C_HandleTypeDef *i2c2;
    IWDG_HandleTypeDef *iwdg;
    SPI_HandleTypeDef *spi3;
    TIM_HandleTypeDef *tim1;
    TIM_HandleTypeDef *tim2;
    TIM_HandleTypeDef *tim3;
    TIM_HandleTypeDef *tim4;
    TIM_HandleTypeDef *tim8;
    TIM_HandleTypeDef *tim10;
    TIM_HandleTypeDef *tim11;
    UART_HandleTypeDef *uart1;
    UART_HandleTypeDef *uart2;
    UART_HandleTypeDef *uart3;
    UART_HandleTypeDef *uart6;
    uint32_t * pTickCounter;
};
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MPU_Fsync_Pin GPIO_PIN_14
#define MPU_Fsync_GPIO_Port GPIOC
#define MPU_INT_Pin GPIO_PIN_15
#define MPU_INT_GPIO_Port GPIOC
#define mpu_int_Pin GPIO_PIN_11
#define mpu_int_GPIO_Port GPIOF
#define mpu_fsync_Pin GPIO_PIN_12
#define mpu_fsync_GPIO_Port GPIOF
#define lcd_command_Pin GPIO_PIN_13
#define lcd_command_GPIO_Port GPIOF
#define lcd_cs_Pin GPIO_PIN_14
#define lcd_cs_GPIO_Port GPIOF
#define rfm_cs_Pin GPIO_PIN_15
#define rfm_cs_GPIO_Port GPIOF
#define rfm_irq_Pin GPIO_PIN_0
#define rfm_irq_GPIO_Port GPIOG
#define lcd_reset_Pin GPIO_PIN_1
#define lcd_reset_GPIO_Port GPIOG
#define gps_uart1_tx_Pin GPIO_PIN_9
#define gps_uart1_tx_GPIO_Port GPIOA
#define gps_uart1_rx_Pin GPIO_PIN_10
#define gps_uart1_rx_GPIO_Port GPIOA
#define rs485_de_Pin GPIO_PIN_4
#define rs485_de_GPIO_Port GPIOD
#define servo1_Pin GPIO_PIN_8
#define servo1_GPIO_Port GPIOB
#define servo2_Pin GPIO_PIN_9
#define servo2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
