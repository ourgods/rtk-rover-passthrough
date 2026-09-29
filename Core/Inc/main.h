/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
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
#define UM980_RST_Pin GPIO_PIN_1
#define UM980_RST_GPIO_Port GPIOB
#define AIR780_RST_Pin GPIO_PIN_1
#define AIR780_RST_GPIO_Port GPIOA
#define L33_M0_Pin GPIO_PIN_9
#define L33_M0_GPIO_Port GPIOC
#define L33_M1_Pin GPIO_PIN_8
#define L33_M1_GPIO_Port GPIOA
#define L33_AUX_Pin GPIO_PIN_11
#define L33_AUX_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOB
#define BEEP_Pin GPIO_PIN_9
#define BEEP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;



extern osSemaphoreId UART1_tx_semaphoreHandle;
extern osSemaphoreId UART2_tx_semaphoreHandle;
extern osSemaphoreId UART3_tx_semaphoreHandle;
extern osSemaphoreId UART4_tx_semaphoreHandle;
extern osSemaphoreId UART1_rx_semaphoreHandle;
extern osSemaphoreId UART2_rx_semaphoreHandle;
extern osSemaphoreId UART3_rx_semaphoreHandle;
extern osSemaphoreId UART4_rx_semaphoreHandle;
extern osSemaphoreId UART5_tx_semaphoreHandle;
extern osSemaphoreId UART5_rx_semaphoreHandle;

#define _KHAL_UART_ENABLE_RX(__HANDLE__)   ((__HANDLE__)->Instance->CR1 |=  USART_CR1_RE)
#define _KHAL_UART_DISABLE_RX(__HANDLE__)  ((__HANDLE__)->Instance->CR1 &=  (~USART_CR1_RE))

#define _KHAL_UART_ENABLE_TX(__HANDLE__)   ((__HANDLE__)->Instance->CR1 |=  USART_CR1_TE)
#define _KHAL_UART_DISABLE_TX(__HANDLE__)  ((__HANDLE__)->Instance->CR1 &=  (~USART_CR1_TE))

#define Periph_BASE         0x40000000  //Peripheral 
#define Periph_BB_BASE      0x42000000  //Peripheral bitband

#define Periph_BB(PeriphAddr, BitNumber)    \
          (*(__IO uint32_t *) (Periph_BB_BASE | ((PeriphAddr - Periph_BASE) << 5) | ((BitNumber) << 2)))
	 
#define Periph_GetBit_BB(PeriphAddr, BitNumber)       \
          (*(__IO uint32_t *) (Periph_BB_BASE | ((PeriphAddr - Periph_BASE) << 5) | ((BitNumber) << 2)))

#define PAOutBit(n)     Periph_BB((uint32_t)&GPIOA->ODR,n)  // 
#define PAInBit(n)      Periph_BB((uint32_t)&GPIOA->IDR,n)  //

#define PBOutBit(n)     Periph_BB((uint32_t)&GPIOB->ODR,n)  // 
#define PBInBit(n)      Periph_BB((uint32_t)&GPIOB->IDR,n)  //

#define PCOutBit(n)     Periph_BB((uint32_t)&GPIOC->ODR,n)  // 
#define PCInBit(n)      Periph_BB((uint32_t)&GPIOC->IDR,n)  //

#define PDOutBit(n)     Periph_BB((uint32_t)&GPIOD->ODR,n)  // 
#define PDInBit(n)      Periph_BB((uint32_t)&GPIOD->IDR,n)  //

#define PEOutBit(n)     Periph_BB((uint32_t)&GPIOE->ODR,n)  // 
#define PEInBit(n)      Periph_BB((uint32_t)&GPIOE->IDR,n)  //

#define PFOutBit(n)     Periph_BB((uint32_t)&GPIOF->ODR,n)  // 
#define PFInBit(n)      Periph_BB((uint32_t)&GPIOF->IDR,n)  //

#define PGOutBit(n)     Periph_BB((uint32_t)&GPIOG->ODR,n)  // 
#define PGInBit(n)      Periph_BB((uint32_t)&GPIOG->IDR,n)  //

#define LED1        PBOutBit(3)     
#define LED2        PBOutBit(4)                   
#define LED3        PBOutBit(5)  
              
#define UM980_RST   PBOutBit(1)
#define AIR780_RST  PAOutBit(1)
#define L33_M0      PCOutBit(9)    
#define L33_M1      PAOutBit(8)    
#define L33_AUX     PAInBit(11) 
#define BEEP        PBOutBit(9) 
#define RUN_LED     LED3  
#define RTK_LED     LED2
#define LORA_LED    LED1              
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
