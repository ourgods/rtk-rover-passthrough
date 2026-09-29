#include "main.h"
#include "cmsis_os.h"
#include "JY6X.h"
#include <string.h>

int16_t g_Pitch_int,g_Roll_int;
uint8_t g_UART3_tx_buffer[SIZE_OF_UART3_TX_BUF];
uint8_t g_UART3_rx_buffer[SIZE_OF_UART3_RX_BUF];
volatile uint32_t g_UART3_rx_len = 0;               //接收数据长度


static uint8_t CaliSum(uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint8_t ucCheck = 0;
    for(i=0; i<len; i++) ucCheck += *(data + i);
    return ucCheck;
}

void get_imu_data()
{
    
    taskENTER_CRITICAL();
    for (int i=0;i<SIZE_OF_UART3_RX_BUF-10;i++)
    {
        if ((g_UART3_rx_buffer[i]==0x55) && (g_UART3_rx_buffer[i+1]==0x53))
        {
            if (CaliSum(&g_UART3_rx_buffer[i],10) == g_UART3_rx_buffer[i + 10])
            {
                g_Pitch_int = (int16_t)((((int32_t)g_UART3_rx_buffer[i+3]) << 8) | g_UART3_rx_buffer[i+2]);
                g_Roll_int = (int16_t)((((int32_t)g_UART3_rx_buffer[i+5]) << 8) | g_UART3_rx_buffer[i+4]);
            }
        }
    }
    taskEXIT_CRITICAL();
}


void imu_task(void const * argument)
{
    osDelay(1000);
    __HAL_UART_CLEAR_IDLEFLAG(&huart3); 
    __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3,g_UART3_rx_buffer,SIZE_OF_UART3_RX_BUF);

    for(;;)
    {
            if (osSemaphoreAcquire(UART3_rx_semaphoreHandle,100) == osOK)  
            {  
                get_imu_data();
                memset(g_UART3_rx_buffer,0,SIZE_OF_UART3_RX_BUF);
                HAL_UARTEx_ReceiveToIdle_DMA(&huart3,g_UART3_rx_buffer,SIZE_OF_UART3_TX_BUF);//数据处理完毕，重新启动接收，防止未处理完毕，数据被破坏
            }
            osDelay(1);

    }
}
