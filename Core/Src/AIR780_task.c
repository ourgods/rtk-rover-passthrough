#include "main.h"
#include "cmsis_os.h"
#include "air780_driver.h"
#include "AIR780_TASK.H"

#include "console.h"
#include <string.h>



// -------------------------- 主循环 --------------------------
void AIR780_task(void const * argument)
{
    osDelay(1000);    // 模块上电启动延时（必加）
    __HAL_UART_CLEAR_IDLEFLAG(&huart2); 
    __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2,(uint8_t *)UART2_RecvBuf,SIZE_OF_UART2_RX_BUF);
    
    UM980_RST = 0;
    osDelay(2000);    // 模块上电启动延时（必加）
    UM980_RST = 1;

    while (1)
    {
        switch (g_net_state)
        {
            case NET_INIT:
                Net_ModuleInit();
                break;
            case NET_REG_CHECK:
                Net_CheckReg();
                break;
            case NET_ATTACH_CHECK:
                Net_CheckAttach();
                break;
            case NET_SET_APN:
                NET_Set_APN();
                break;
            case NET_CONNECT:
                Net_ConnectTCP();
                break;
            case NET_SET_KEEPALIVE:
                Net_SetKeepAlive();
                break;
            case NET_SET_HEART:
                Net_SetHeartBeat();
                break;
            case NET_READY:
                // 周期性发送数据（示例）
                Net_SendData("Hello Server!", 13);
                osDelay(5000);
                // 接收数据（处理URC上报）
                Net_RecvData();
                break;
            case NET_RECONNECT:
                Net_Reconnect();
                break;
            default:
                g_net_state = NET_INIT;
                break;
        }
        osDelay(100);
    }
}