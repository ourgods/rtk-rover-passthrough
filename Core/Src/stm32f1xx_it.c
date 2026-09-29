/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "passthrough.h"
#include "L33_driver.h"
#include "lora_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
//    uint32_t temp;
//    BaseType_t tHigherPriorityTaskWoken;
//    
//    if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET))                    
//    { 
//        __HAL_UART_CLEAR_IDLEFLAG(&huart1);     
//        HAL_UART_DMAStop(&huart1);              
//        temp = hdma_usart1_rx.Instance->CNDTR;
//        L33_uart_rx_len = SIZE_OF_L33RCV_BUF - temp;                                  
//        //g_UART1_rx_buffer[g_UART1_rx_len] = 0;                                        
//        xSemaphoreGiveFromISR(UART1_rx_semaphoreHandle,&tHigherPriorityTaskWoken);    
//    }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_PE) != RESET ||   
       __HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE)  != RESET ||   
       __HAL_UART_GET_FLAG(&huart1, UART_FLAG_NE)  != RESET ||  
       __HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET)    
    {
        uint32_t temp = huart1.Instance->SR;
                 temp = huart1.Instance->DR;
        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE);
        g_rover_diag.uart1_err++;   /* L33 串口误码计数 (watch g_rover_diag.uart1_err) */
    } 
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
//   uint32_t temp;
//    BaseType_t tHigherPriorityTaskWoken;
//    
//    if((__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET))                    
//    { 
//        __HAL_UART_CLEAR_IDLEFLAG(&huart2);     
//        HAL_UART_DMAStop(&huart2);              
//        temp = hdma_usart2_rx.Instance->CNDTR;
//       // g_UART2_rx_len = SIZE_OF_UART2_RX_BUF - temp;                                  
//        //g_UART1_rx_buffer[g_UART1_rx_len] = 0;                                        
//        xSemaphoreGiveFromISR(UART2_rx_semaphoreHandle,&tHigherPriorityTaskWoken);    
//    }
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
    if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_PE) != RESET ||   
       __HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE)  != RESET ||   
       __HAL_UART_GET_FLAG(&huart2, UART_FLAG_NE)  != RESET ||  
       __HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) != RESET)    
    {
        uint32_t temp = huart2.Instance->SR;
                 temp = huart2.Instance->DR;
        __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE);
    } 
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
//    uint32_t temp;
//    BaseType_t tHigherPriorityTaskWoken;
//    
//    if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE) != RESET))                    
//    { 
//        __HAL_UART_CLEAR_IDLEFLAG(&huart3);     
//        HAL_UART_DMAStop(&huart3);              
//        temp = hdma_usart3_rx.Instance->CNDTR;
//        g_UART3_rx_len = SIZE_OF_UART3_RX_BUF - temp;                                  
//        //g_UART1_rx_buffer[g_UART1_rx_len] = 0;                                        
//        xSemaphoreGiveFromISR(UART3_rx_semaphoreHandle,&tHigherPriorityTaskWoken);    
//    }
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
    if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_PE) != RESET ||   
       __HAL_UART_GET_FLAG(&huart3, UART_FLAG_FE)  != RESET ||   
       __HAL_UART_GET_FLAG(&huart3, UART_FLAG_NE)  != RESET ||  
       __HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE) != RESET)    
    {
        uint32_t temp = huart3.Instance->SR;
                 temp = huart3.Instance->DR;
        __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE);
    }
  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */
    if(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_PE) != RESET ||   
       __HAL_UART_GET_FLAG(&huart4, UART_FLAG_FE)  != RESET ||   
       __HAL_UART_GET_FLAG(&huart4, UART_FLAG_NE)  != RESET ||  
       __HAL_UART_GET_FLAG(&huart4, UART_FLAG_ORE) != RESET)    
    {
        uint32_t temp = huart4.Instance->SR;
                 temp = huart4.Instance->DR;
        __HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE);
    }
  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */
    if(__HAL_UART_GET_FLAG(&huart5, UART_FLAG_PE) != RESET ||   
       __HAL_UART_GET_FLAG(&huart5, UART_FLAG_FE)  != RESET ||   
       __HAL_UART_GET_FLAG(&huart5, UART_FLAG_NE)  != RESET ||  
       __HAL_UART_GET_FLAG(&huart5, UART_FLAG_ORE) != RESET)    
    {
         uint32_t temp = huart5.Instance->SR;
                  temp = huart5.Instance->DR;
        __HAL_UART_CLEAR_FLAG(&huart5, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE);
    }
  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel3 global interrupt.
  */
void DMA2_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel3_IRQn 0 */

  /* USER CODE END DMA2_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA2_Channel3_IRQn 1 */

  /* USER CODE END DMA2_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA2 channel4 and channel5 global interrupts.
  */
void DMA2_Channel4_5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel4_5_IRQn 0 */

  /* USER CODE END DMA2_Channel4_5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA2_Channel4_5_IRQn 1 */

  /* USER CODE END DMA2_Channel4_5_IRQn 1 */
}

/* USER CODE BEGIN 1 */
volatile uint32_t dbg_uart1_txcplt_cnt = 0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        dbg_uart1_txcplt_cnt++;
        osSemaphoreRelease(UART1_tx_semaphoreHandle);   
    }
    else if (huart == &huart2)
    {
        osSemaphoreRelease(UART2_tx_semaphoreHandle); 
    }
    else if (huart == &huart3)
    {
        osSemaphoreRelease(UART3_tx_semaphoreHandle); 
    }
    else if (huart == &huart4)
    {
        osSemaphoreRelease(UART4_tx_semaphoreHandle); 
    }
    else if (huart == &huart5)
    {
        osSemaphoreRelease(UART5_tx_semaphoreHandle); 
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)
    {
        L33_uart_rx_len = Size;
        g_passthrough_uart1_rx_len = Size;
        osSemaphoreRelease(UART1_rx_semaphoreHandle);
    }
    else if (huart == &huart2)
    {
        osSemaphoreRelease(UART2_rx_semaphoreHandle);
    }
    else if (huart == &huart3)
    {
        osSemaphoreRelease(UART3_rx_semaphoreHandle);
    }
    else if (huart == &huart4)
    {
        passthrough_uart4_rx_event_callback(Size);
    }
    else if (huart == &huart5)
    {
        g_passthrough_uart5_rx_len = Size;
        osSemaphoreRelease(UART5_rx_semaphoreHandle);
    }
}

volatile uint8_t g_uart1_err_flag = 0;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        g_uart1_err_flag = 1;
        L33_uart_rx_len = 0;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, L33_uart_rx_buf, sizeof(L33_uart_rx_buf));
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
    else if (huart == &huart4)
    {
        passthrough_uart4_error_callback();
    }
    else if (huart == &huart5)
    {
        HAL_UARTEx_ReceiveToIdle_IT(&huart5, uart5_rx_buf, PASSTHROUGH_UART5_BUF_SIZE);
    }
}

/* USER CODE END 1 */
