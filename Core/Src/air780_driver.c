#include "main.h"
#include "cmsis_os.h"
#include "air780_driver.h"
#include "console.h"
#include <string.h>


// -------------------------- 配置参数（根据实际需求修改）--------------------------
#define SERVER_IP       "112.125.89.8"  // 服务器IP（或域名，需先解析）
#define SERVER_PORT     32147          // 服务器TCP端口
#define CID             1              // PDP上下文标识（固定1即可）
// TCP保活参数（参考手册11.30节）
#define KEEP_IDLE       300            // 无数据时300秒发送保活探针
#define KEEP_INTERVAL   60             // 探针重传间隔60秒
#define KEEP_COUNT      3              // 探针最大重传3次
// 心跳包参数（参考手册11.31节）
#define HEART_INTERVAL  120            // 心跳间隔120秒
#define HEART_CONTENT   "Air780_HeartBeat" // 心跳内容（可自定义）

// -------------------------- 状态定义 --------------------------


Net_State g_net_state = NET_INIT;
char UART2_RecvBuf[SIZE_OF_UART2_RX_BUF];
volatile uint32_t g_UART2_rx_len = 0;               //接收数据长度


// -------------------------- 工具函数 --------------------------
static void AT_AIR780_SendData(uint8_t* data,uint16_t length)
{
    if (osSemaphoreAcquire(UART2_tx_semaphoreHandle,100) == pdTRUE)
    {
        _KHAL_UART_DISABLE_RX(&huart2);  //防止在数据发送的过程中，发生空闲中断，会导致DMA禁能，死锁
        HAL_UART_Transmit_DMA(&huart2,data,length);
    }
}

//void UART_SendString(char *str)
//{ 
//    if (xSemaphoreTake(UART2_tx_semaphoreHandle,100) == pdTRUE)
//    {
//        _KHAL_UART_DISABLE_RX(&huart2);  //防止在数据发送的过程中，发生空闲中断，会导致DMA禁能，死锁
//        HAL_UART_Transmit_DMA(&huart2,(const uint8_t *)str,strlen(str));
//    }
//
//}
//// -------------------------- 工具函数 --------------------------
//// 串口发送AT指令（带回车换行）
//void AT_AIR780_SendCmd(char *cmd)
//{
//    char temp_str[30];
//    memset(temp_str,0,sizeof(temp_str));
//    strcpy(temp_str,cmd);
//    strcat(temp_str,"\r\n");
//    UART_SendString(temp_str);
//    osDelay(200); // 等待模块响应
//}

// 串口发送AT指令（带回车换行）
static void AT_AIR780_SendCmd(char *cmd)
{
    char temp_str[50];
    uint32_t t_len = strlen(cmd);
    if (t_len > 45) {t_len = 45;}
    memset(temp_str,0,sizeof(temp_str));
    strncpy(temp_str,cmd,t_len);
    strcat(temp_str,"\r\n");
    if (osSemaphoreAcquire(UART2_tx_semaphoreHandle,100) == pdTRUE)
    {
        _KHAL_UART_DISABLE_RX(&huart2);  //防止在数据发送的过程中，发生空闲中断，会导致DMA禁能，死锁
        HAL_UART_Transmit_DMA(&huart2,(const uint8_t *)temp_str,strlen(temp_str));
    }
    osDelay(50); // 等待模块响应
}

// 清空串口接收缓冲区,重新启动接收
static void AIR780_ClearBuf_RestartDMA()
{
    memset(UART2_RecvBuf, 0, sizeof(UART2_RecvBuf));
    HAL_UART_Receive_DMA(&huart2,(uint8_t*)UART2_RecvBuf,SIZE_OF_UART2_RX_BUF);
}

// 检查串口接收缓冲区是否包含目标字符串
char AT_AIR780_CheckResp_ClearBuf(char *target)
{
     if (osSemaphoreAcquire(UART2_rx_semaphoreHandle,300) == pdTRUE)  
     {
         if (strstr(UART2_RecvBuf, target))
         {
             AIR780_ClearBuf_RestartDMA();
             return 1;
         }
         AIR780_ClearBuf_RestartDMA();
     } 
    return  0;
}

// -------------------------- 核心流程函数 --------------------------
void Net_ModuleReset_overTime(uint32_t* counter,uint32_t Limit)
{
    (*counter)++;
    if (*counter > Limit)
    {
        *counter = 0;
        AT_AIR780_SendCmd("AT+RESET");
        g_net_state = NET_INIT;
        AIR_printf("Reset and init again\r\n");
    }
    AIR780_ClearBuf_RestartDMA();
}
// 1. 模块初始化（关闭回显、开启错误码显示）
void Net_ModuleInit()
{
    static uint32_t t_try_counter = 0;
    AT_AIR780_SendCmd("ATE0");        // 关闭命令回显（减少串口干扰）
    if (AT_AIR780_CheckResp_ClearBuf("OK"))
    {
        t_try_counter = 0;
        AT_AIR780_SendCmd("AT+CMEE=1"); // 开启数字型错误码显示
        if (AT_AIR780_CheckResp_ClearBuf("OK"))
        {
            g_net_state = NET_REG_CHECK;
            AIR_printf("Init OK\r\n");
        }
    }
    else
    {
        osDelay(1000);
        AIR_printf("Init again\r\n");
        Net_ModuleReset_overTime(&t_try_counter,10);
    }
    
}

// 2. 检查网络注册状态（AT+CREG?，参考手册5.5节）
void Net_CheckReg()
{
    static uint32_t t_try_counter = 0;
    
    AT_AIR780_SendCmd("AT+CFUN?");
    if (AT_AIR780_CheckResp_ClearBuf("+CFUN: 1"))
    {
        t_try_counter = 0;
        AIR_printf("CFUN Ready\r\n");
    }
    else
    {
        AIR_printf("CFUN Failed\r\n");
        Net_ModuleReset_overTime(&t_try_counter,10);
        return;
    }
    
    AT_AIR780_SendCmd("AT+CPIN?");
    if (AT_AIR780_CheckResp_ClearBuf("+CPIN: READY"))
    {
        AIR_printf("SIM Ready\r\n");
    }
    else
    {
        AIR_printf("SIM Failed\r\n");
        Net_ModuleReset_overTime(&t_try_counter,10);
        return;
    }
    
    AT_AIR780_SendCmd("AT+CSQ");
    if (AT_AIR780_CheckResp_ClearBuf("+CSQ"))
    {
        AIR_printf("CSQ check OK\r\n");
    }  
    else
    {
        AIR_printf("CSQ Failed\r\n");
        Net_ModuleReset_overTime(&t_try_counter,10);
        return;
    }
    
    AT_AIR780_SendCmd("AT+CREG?");
    if (osSemaphoreAcquire(UART2_rx_semaphoreHandle,300) == pdTRUE)  
    {
        if ((strstr(UART2_RecvBuf, "+CREG: 0,1")) || (strstr(UART2_RecvBuf, "+CREG: 0,5")))
        {
            g_net_state = NET_ATTACH_CHECK;
            AIR_printf("CREG OK\r\n");
        }
        else if ((strstr(UART2_RecvBuf, "+CREG: 0,2")) || (strstr(UART2_RecvBuf, "+CREG: 0,3")))
        {
            // 注册被拒绝，重启模块
            AT_AIR780_SendCmd("AT+RESET");
            osDelay(5000);
            g_net_state = NET_INIT;
            AIR_printf("Reset and init again\r\n");
        }
        AIR780_ClearBuf_RestartDMA();
    } 
    else
    {
        osDelay(1000); // 未注册，重试
        AIR_printf("CREG again\r\n");
        Net_ModuleReset_overTime(&t_try_counter,10);
    }
    
}

// 3. 检查网络附着状态（AT+CGATT?，参考手册8.2节）
void Net_CheckAttach()
{
    static uint32_t t_try_counter = 0;
    AT_AIR780_SendCmd("AT+CGACT?");
    if (AT_AIR780_CheckResp_ClearBuf("+CGACT: 1"))
    {
        t_try_counter = 0;
        // 已附着，进入激活流程
        g_net_state = NET_SET_APN;
        AIR_printf("NET Attach access!\r\n");
    }
    else
    {  
        // 未附着，尝试附着
        AT_AIR780_SendCmd("AT+CGATT=1");
        AIR780_ClearBuf_RestartDMA();
        AIR_printf("NET Attach failed,try again\r\n");
        osDelay(2000);
        
        Net_ModuleReset_overTime(&t_try_counter,5);
        return;
    }

}
// 4. Set_APN配置
void NET_Set_APN()
{
    static uint32_t t_try_counter = 0;
    
    // 配置单连接模式
    AT_AIR780_SendCmd("AT+CIPMUX=0");
    if (AT_AIR780_CheckResp_ClearBuf("OK"))
    {
        t_try_counter = 0;
        AIR_printf("CIPMUX OK\r\n");
    }    
    else
    {
        AIR_printf("APN failed,try again\r\n");
        osDelay(2000);
        
        Net_ModuleReset_overTime(&t_try_counter,5);
        return;
    }
    osDelay(500);
    
    
    // 配置快发模式（避免阻塞）
    AT_AIR780_SendCmd("AT+CIPQSEND=1");
    if (AT_AIR780_CheckResp_ClearBuf("OK"))
    {
        t_try_counter = 0;
        AIR_printf("CIPQSEND OK\r\n");
    }     
    else
    {
        AIR_printf("APN failed,try again\r\n");
        osDelay(2000);
        
        Net_ModuleReset_overTime(&t_try_counter,5);
        return;
    }
    osDelay(500);

    AT_AIR780_SendCmd("AT+CSTT");
    if (AT_AIR780_CheckResp_ClearBuf("OK"))
    {
        g_net_state = NET_CONNECT;
        AIR_printf("APN OK\r\n");
    }   
    else
    {
        AIR_printf("APN failed,try again\r\n");
        osDelay(2000);
        
        Net_ModuleReset_overTime(&t_try_counter,5);
        return;
    }
    
}


// 5. 建立TCP连接（参考手册11.9节）
void Net_ConnectTCP()
{
    char cmd[64];
    memset(cmd,0,sizeof(cmd));
    static uint32_t t_try_counter = 0;
    
    AT_AIR780_SendCmd("AT+CIICR");
    if (AT_AIR780_CheckResp_ClearBuf("OK"))
    {
        t_try_counter = 0;
        AIR_printf("CIICR OK\r\n");
        osDelay(2000);
    }   
    else
    {
        AIR_printf("CIICR failed,try again\r\n");
        osDelay(2000);
        
        Net_ModuleReset_overTime(&t_try_counter,5);
        return;
    }
    
    
//    AT_AIR780_SendCmd("AT+CIFSR");
//    if (AT_AIR780_CheckResp_ClearBuf("."))
//    {
//        AIR_printf("CIFSR OK\r\n");
//        
//    }  
//    else
//    {
//        AIR_printf("GO...CIPSTATUS CHECK IP\r\n");
//    }
    
    while(1)
    {
        AT_AIR780_SendCmd("AT+CIPSTATUS");
        if (AT_AIR780_CheckResp_ClearBuf("OK"))
        {
            AIR_printf("CIPSTATUS OK\r\n");
            break;
        }   
        else
        {
            AIR780_ClearBuf_RestartDMA();
            AIR_printf("CIPSTATUS failed,try again\r\n");
            osDelay(2000);
            t_try_counter++;
            if (t_try_counter > 10)
            {
                AT_AIR780_SendCmd("AT+RESET");
                g_net_state = NET_INIT;
                AIR_printf("Reset and init again\r\n");
                return;
            }
        }
    }
    
    
    // 建立TCP连接：AT+CIPSTART="TCP","IP",端口
    sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%d", SERVER_IP, SERVER_PORT);
    AT_AIR780_SendCmd(cmd);
    if (AT_AIR780_CheckResp_ClearBuf("OK"))
    {
        t_try_counter = 0;
        g_net_state = NET_SET_KEEPALIVE;
        AIR_printf("CIPSTART OK\r\n");
    }  
    else
    {
        AIR_printf("Connect error\r\n");
        Net_ModuleReset_overTime(&t_try_counter,10);
        return;
    }
    osDelay(500);
    
}

// 6. 配置TCP保活（参考手册11.30节）
void Net_SetKeepAlive()
{
    static uint32_t t_try_counter = 0;
    char cmd[64];
    memset(cmd,0,sizeof(cmd));
    sprintf(cmd, "AT+CIPTKA=1,%d,%d,%d", KEEP_IDLE, KEEP_INTERVAL, KEEP_COUNT);
    AT_AIR780_SendCmd(cmd);
    if (AT_AIR780_CheckResp_ClearBuf("OK"))
    {
        t_try_counter = 0;
        g_net_state = NET_SET_HEART;
        AIR_printf("TCP connect Keep Alive\r\n");
    }
    else
    {
        
        osDelay(1000);
        AIR_printf("TCP connect Keep Alive faild\r\n");
        Net_ModuleReset_overTime(&t_try_counter,5);
        
    }
    
}

// 7. 配置心跳包（参考手册11.31-11.32节）
void Net_SetHeartBeat()
{
    char cmd[64];
    memset(cmd,0,sizeof(cmd));
    // 步骤1：开启心跳功能（socket id=0，单连接固定0）
    sprintf(cmd, "AT^HEARTCONFIG=1,0,%d", HEART_INTERVAL);
    AT_AIR780_SendCmd(cmd);
    AIR780_ClearBuf_RestartDMA();
    osDelay(500);
    // 步骤2：设置心跳内容
    memset(cmd,0,sizeof(cmd));
    sprintf(cmd, "AT^HEARTBEAT=0,\"%s\"", HEART_CONTENT);
    AT_AIR780_SendCmd(cmd);
    if (AT_AIR780_CheckResp_ClearBuf("OK"))
    {
        g_net_state = NET_READY;
        Printf("Set HeartBeat OK！\r\n");
    }
    else
    {
        osDelay(1000);
        AIR_printf("Set HeartBeat failed\r\n");
    }
    
}

// 8. 发送数据（参考手册11.14节）
char Net_SendData(char *data, uint16_t len)
{
    char cmd[32];
    memset(cmd,0,sizeof(cmd));
    if (g_net_state != NET_READY)
    {
        return 0;
    }
    // 检查连接状态
    AIR780_ClearBuf_RestartDMA();
    AT_AIR780_SendCmd("AT+CIPSTATUS");
    if (AT_AIR780_CheckResp_ClearBuf("OK"))
    {
        // 发送定长数据：AT+CIPSEND=长度
        sprintf(cmd, "AT+CIPSEND=%d", len);
        AT_AIR780_SendCmd(cmd);
        if (AT_AIR780_CheckResp_ClearBuf(">"))
        { 
            // 等待模块提示符
            AIR780_ClearBuf_RestartDMA();
            AT_AIR780_SendData(data,len); // 发送数据内容
            osDelay(100);
            // 检查发送结果（快发模式返回DATA ACCEPT）
            if (AT_AIR780_CheckResp_ClearBuf("DATA ACCEPT"))
            {
                return 1;
            }
        }
    }
    // 连接断开，触发重连
    g_net_state = NET_RECONNECT;
    return 0;
}

// 9. 接收数据（处理URC上报，参考手册11.24节）
void Net_RecvData()
{
    if (UART2_RecvBuf[0] == 0)
    {
        return;
    }
    // 检测连接断开
    if (osSemaphoreAcquire(UART2_rx_semaphoreHandle,300) == pdTRUE)  
    {
        if ((strstr(UART2_RecvBuf, "CLOSED")) || (strstr(UART2_RecvBuf, "+PDP DEACT")))
        {
            Printf("Disconnect！\r\n");
            g_net_state = NET_RECONNECT;
        }
        else if ((strstr(UART2_RecvBuf, HEART_CONTENT)))
        {
            Printf("Receive heart beat\r\n");
        }
        else
        {
            
            Printf("Data received：%s\r\n", UART2_RecvBuf);
        }
        
    }
    
}

// 10. 重连流程（重启连接步骤）
void Net_Reconnect()
{
    AT_AIR780_SendCmd("AT+CIPCLOSE"); // 关闭旧连接
    AIR780_ClearBuf_RestartDMA();
    osDelay(1000);
    
    
    AT_AIR780_SendCmd("AT+CIPSHUT");  // 关闭移动场景
    AIR780_ClearBuf_RestartDMA();
    osDelay(2000);
    
    
    g_net_state = NET_REG_CHECK; // 回到网络检查步骤
    AIR780_ClearBuf_RestartDMA();
    Printf("re-connected...\r\n");
}