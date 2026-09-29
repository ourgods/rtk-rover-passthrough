#include "main.h"
#include "cmsis_os.h"
#include "L33_driver.h"
#include "console.h"
#include <string.h>
#include <stdio.h>

// 全局变量
L33_ReceiveTypeDef L33_rx_data = {0};
uint8_t  L33_uart_rx_buf[L33_RCV_BUF_SIZE];  // UART接收缓冲区
uint8_t  L33_uart_tx_buf[L33_SEND_BUF_SIZE];  // UART发送缓冲区
uint16_t L33_uart_rx_len = 0;

volatile uint32_t dbg_l33_data_tx_try = 0;
volatile uint32_t dbg_l33_data_tx_ok = 0;
volatile uint32_t dbg_l33_data_tx_fail = 0;
volatile uint32_t dbg_l33_data_tx_timeout = 0;
volatile uint32_t dbg_l33_data_tx_last_ret = 0;
volatile uint32_t dbg_l33_data_tx_last_len = 0;

static uint8_t L33_WaitAuxReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (L33_AUX == 0)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return 0;
        }
        osDelay(1);
    }

    return 1;
}

static uint32_t L33_TxWaitTimeoutMs(uint16_t length)
{
    uint32_t tx_time_ms = ((uint32_t)length * 10U * 1000U + L33_BAUDRATE - 1U) / L33_BAUDRATE;

    return tx_time_ms + 300U;
}

void restart_L33_DMA_RCV()
{
    L33_uart_rx_len = 0;
    HAL_UART_AbortReceive(&huart1);
    HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(&huart1,L33_uart_rx_buf,sizeof(L33_uart_rx_buf));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    if (ret != HAL_OK)
    {
        L33_printf("DMA RCV restart FAIL: %d\r\n", ret);
    }
}

static uint8_t L33_SendData(uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef dma_ret;

    dbg_l33_data_tx_try++;
    dbg_l33_data_tx_last_len = length;
    if (length > L33_SEND_BUF_SIZE)
    {
        dbg_l33_data_tx_last_ret = HAL_ERROR;
        dbg_l33_data_tx_fail++;
        return 0;
    }
    if (osSemaphoreAcquire(UART1_tx_semaphoreHandle, L33_TxWaitTimeoutMs(length)) == osOK)
    {
        if (L33_WaitAuxReady(3000U) == 0)
        {
            dbg_l33_data_tx_last_ret = HAL_BUSY;
            dbg_l33_data_tx_timeout++;
            osSemaphoreRelease(UART1_tx_semaphoreHandle);
            return 0;
        }
        memcpy(L33_uart_tx_buf, data, length);
        dma_ret = HAL_UART_Transmit_DMA(&huart1, L33_uart_tx_buf, length);
        dbg_l33_data_tx_last_ret = dma_ret;
        if (dma_ret != HAL_OK)
        {
            dbg_l33_data_tx_fail++;
            osSemaphoreRelease(UART1_tx_semaphoreHandle);
            return 0;
        }
        else
        {
            dbg_l33_data_tx_ok++;
            return 1;
        }
    }
    else
    {
        dbg_l33_data_tx_timeout++;
        dbg_l33_data_tx_last_ret = HAL_TIMEOUT;
        return 0;
    }
}

static void L33_goto_ATMode()
{
    HAL_StatusTypeDef dma_ret;
    if (osSemaphoreAcquire(UART1_tx_semaphoreHandle, 100) == osOK)
    {
        strncpy((char *)L33_uart_tx_buf, "+++", 3);
        dma_ret = HAL_UART_Transmit_DMA(&huart1, (const uint8_t *)L33_uart_tx_buf, 3);
        if (dma_ret != HAL_OK)
        {
            osSemaphoreRelease(UART1_tx_semaphoreHandle);
        }
    }
}

static void L33_SendATCmd(uint8_t *cmd)
{
    HAL_StatusTypeDef dma_ret;
    uint32_t t_len = strlen((const char *)cmd);
    if (t_len > L33_SEND_BUF_SIZE - 3) t_len = L33_SEND_BUF_SIZE - 3;
    if (osSemaphoreAcquire(UART1_tx_semaphoreHandle, 100) == osOK)
    {
        memset(L33_uart_tx_buf, 0, sizeof(L33_uart_tx_buf));
        memcpy(L33_uart_tx_buf, cmd, t_len);
        L33_uart_tx_buf[t_len++] = '\r';
        L33_uart_tx_buf[t_len++] = '\n';
        dma_ret = HAL_UART_Transmit_DMA(&huart1, (const uint8_t *)L33_uart_tx_buf, t_len);
        if (dma_ret != HAL_OK)
        {
            osSemaphoreRelease(UART1_tx_semaphoreHandle);
        }
    }
    osDelay(50);
}

// L33初始化（配置主从定点模式、地址等参数）
void L33_Init() 
{
    uint8_t cmd[64];
    uint8_t at_retry;

    for (at_retry = 0; at_retry < 3; at_retry++)
    {
        osDelay(1200);
        memset(L33_uart_rx_buf, 0, sizeof(L33_uart_rx_buf));
        while (osSemaphoreAcquire(UART1_rx_semaphoreHandle, 0) == osOK) {}
        L33_goto_ATMode();
        if (osSemaphoreAcquire(UART1_rx_semaphoreHandle, 1000) == osOK)  
        {
            if (strstr((char*)L33_uart_rx_buf, "OK") || strstr((char*)L33_uart_rx_buf, "a"))
            {
                L33_printf("1 Go setting mode\r\n");
                restart_L33_DMA_RCV();
                break;
            }
        }
        L33_printf("1 +++ retry %d\r\n", at_retry + 1);
        restart_L33_DMA_RCV();
    }
    if (at_retry >= 3)
    {
        L33_printf("1 +++ FAILED, skip config\r\n");
        restart_L33_DMA_RCV();
        return;
    }
        // 2. 配置串口参数（115200, 无校验）
        sprintf((char*)cmd, "AT+UART=%d,0", L33_BAUDRATE);
        L33_SendATCmd(cmd);
        if (osSemaphoreAcquire(UART1_rx_semaphoreHandle,300) == osOK)  
        {
            if (strstr((char const *)L33_uart_rx_buf, "OK"))
            {
                L33_printf("2 AT+UART=%d,0\r\n",L33_BAUDRATE);
            }
            restart_L33_DMA_RCV();
        }
        
        // 3. 配置空中速率（9600bps，L33支持）
        sprintf((char*)cmd, "AT+AIRSPEED=9600");
        L33_SendATCmd(cmd);
        if (osSemaphoreAcquire(UART1_rx_semaphoreHandle,300) == osOK)  
        {
            if (strstr((char const *)L33_uart_rx_buf, "OK"))
            {
                L33_printf("3 AT+AIRSPEED=9600\r\n");
            }
            restart_L33_DMA_RCV();
        }
        
        // 4. 配置频率（435MHz=22） AA BB 16 ?? ?? ?? ?? ??
        sprintf((char*)cmd, "AT+TXFREQ=22");
        L33_SendATCmd(cmd);
        if (osSemaphoreAcquire(UART1_rx_semaphoreHandle,300) == osOK)  
        {
            if (strstr((char const *)L33_uart_rx_buf, "OK"))
            {
                L33_printf("4.1 AT+TXFREQ=22,432MHZ\r\n");
            }
            restart_L33_DMA_RCV();
        }
        
        
        sprintf((char*)cmd, "AT+RXFREQ=22");
        L33_SendATCmd(cmd);
        if (osSemaphoreAcquire(UART1_rx_semaphoreHandle,300) == osOK)  
        {
            if (strstr((char const *)L33_uart_rx_buf, "OK"))
            {
                L33_printf("4.2 AT+RXFREQ=22,432MHZ\r\n");
            }
            restart_L33_DMA_RCV();
        }
        
        
        // 5. 配置定点模式
        sprintf((char*)cmd, "AT+WORKMODE=0");  //0：透传模式1：定点模式2：主从模式3：自组网模式
        L33_SendATCmd(cmd);
        if (osSemaphoreAcquire(UART1_rx_semaphoreHandle,300) == osOK)  
        {
            if (strstr((char const *)L33_uart_rx_buf, "OK"))
            {
                L33_printf("5 AT+WORKMODE=0,透传模式\r\n");
            }
            restart_L33_DMA_RCV();
        }
        
        /*
        // 6. 配置主从角色与地址
        sprintf((char*)cmd, "AT+ADDRESS=%d", L33_BASE_STATION_ADDR);  // 主机地址
        L33_SendATCmd(cmd);
        if (osSemaphoreAcquire(UART1_rx_semaphoreHandle,300) == osOK)  
        {
            if (strstr((char const *)L33_uart_rx_buf, "OK"))
            {
                L33_printf("6 AT+ADDRESS=%d\r\n",L33_BASE_STATION_ADDR);
            }
            restart_L33_DMA_RCV();
        }*/

        // 6.1 配置网络ID（必须与接收端一致）
        L33_SendATCmd("AT+NETID=0");
        if (osSemaphoreAcquire(UART1_rx_semaphoreHandle,300) == osOK)
        {
            if (strstr((char const *)L33_uart_rx_buf, "OK"))
            {
                L33_printf("6.1 AT+NETID=0\r\n");
            }
            restart_L33_DMA_RCV();
        }
        
        //if (role == L33_ROLE_MASTER) 
//        {
//            sprintf((char*)cmd, "AT+MSMODE=0");  // 主机模式
//            L33_SendATCmd(cmd);
//            restart_L33_DMA_RCV();
//            L33_printf("MASTER MODE\r\n");
//            
//            sprintf((char*)cmd, "AT+ADDRESS=%d", L33_MASTER_ADDR);  // 主机地址
//            L33_SendATCmd(cmd);
//            restart_L33_DMA_RCV();
//        } 
//        else 
//        {
//            sprintf((char*)cmd, "AT+MSMODE=1");  // 从机模式
//            L33_SendATCmd(cmd);
//            restart_L33_DMA_RCV();
//            L33_printf("SLAVER MODE\r\n");
//            
//            sprintf((char*)cmd, "AT+ADDRESS=%d", slave_addr);  // 从机唯一地址
//            L33_SendATCmd(cmd);
//            restart_L33_DMA_RCV();
//        }
        
        // 7. 配置速率优化、信道过滤、信道避让
        sprintf((char*)cmd, "AT+LDREN=1");
        L33_SendATCmd(cmd);
        if (osSemaphoreAcquire(UART1_rx_semaphoreHandle,300) == osOK)  
        {
            if (strstr((char const *)L33_uart_rx_buf, "OK"))
            {
                L33_printf("7.1 AT+LDREN=1,速率优化\r\n");
            }
            restart_L33_DMA_RCV();
        }
        
        sprintf((char*)cmd, "AT+CHCHECK=1");
        L33_SendATCmd(cmd);
        if (osSemaphoreAcquire(UART1_rx_semaphoreHandle,300) == osOK)  
        {
            if (strstr((char const *)L33_uart_rx_buf, "OK"))
            {
                L33_printf("7.2 AT+CHCHECK=1,信道过滤\r\n");
            }
            restart_L33_DMA_RCV();
        }
        
        sprintf((char*)cmd, "AT+ACTIVITIES=1");
        L33_SendATCmd(cmd);
        if (osSemaphoreAcquire(UART1_rx_semaphoreHandle,300) == osOK)  
        {
            if (strstr((char const *)L33_uart_rx_buf, "OK"))
            {
                L33_printf("7.3 AT+ACTIVITIES=1,信道避让\r\n");
            }
            restart_L33_DMA_RCV();
        }
        
        // 8. 配置业务保活
        sprintf((char*)cmd, "AT+KEEPALIVE=65535,1800,1800");
        L33_SendATCmd(cmd);
        if (osSemaphoreAcquire(UART1_rx_semaphoreHandle,300) == osOK)  
        {
            if (strstr((char const *)L33_uart_rx_buf, "OK"))
            {
                L33_printf("8 AT+KEEPALIVE=65535,1800,1800,业务保活\r\n");
            }
            restart_L33_DMA_RCV();
        }
        
        // 9. 保存配置
        L33_SendATCmd("AT&W");
        if (osSemaphoreAcquire(UART1_rx_semaphoreHandle,300) == osOK)  
        {
            if (strstr((char const *)L33_uart_rx_buf, "OK"))
            {
                L33_printf("9 AT&W,保存配置\r\n");
            }
            restart_L33_DMA_RCV();
        }
        
        // 10. 重启模块使配置生效（重启后模块进入正常通信模式，不能再发AT指令）
        L33_SendATCmd("AT&R");
        L33_printf("10 AT&R,等待重启...\r\n");
        osDelay(3000);
        restart_L33_DMA_RCV();
        L33_printf("10 L33重启完成,进入正常模式\r\n");
}


/**
 * @brief  透传模式发送数据
 * @param  channel: 目标信道
 * @param  data: 数据指针
 * @param  len: 数据长度(1~64)
 * @retval 1-已送入模块 0-失败或模块忙
 */
uint8_t LoRa_PassThrough_Send(uint8_t channel, uint8_t *data, uint8_t len)
{
    uint8_t buf[LORA_MAX_PACKET_LEN ]; // 数据
    
    if(len == 0 || len > LORA_MAX_PACKET_LEN) return 0;
    // 帧格式：纯数据
    memcpy(&buf[0], data, len);        // 数据内容
    
    // 串口发送
    return L33_SendData(buf,len);
}


void L33_ParseData(uint8_t *data, uint16_t len)
{
    uint16_t addr;
    uint8_t channel;

    if (len == 0) return;
    if (len >= 3)
    {
        addr = (data[0] << 8) | data[1];
        channel = data[2];

        if (channel != 22 ||
            (addr != L33_BASE_STATION_ADDR && addr != 0x0002))
        {
            return;
        }

        L33_rx_data.src_addr = addr;
        L33_rx_data.len = len - 3;
        if (L33_rx_data.len > L33_MAX_DATA_LEN)
        {
            L33_rx_data.len = L33_MAX_DATA_LEN;
        }
        memcpy(L33_rx_data.buf, &data[3], L33_rx_data.len);
        L33_rx_data.is_ready = 1;
    }
}
