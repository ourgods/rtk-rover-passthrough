#ifndef _AIR780_DRIVER_H
#define _AIR780_DRIVER_H

#define SIZE_OF_UART2_TX_BUF 100
#define SIZE_OF_UART2_RX_BUF 255

typedef enum 
{
    NET_INIT,        // 初始化
    NET_REG_CHECK,   // 检查网络注册
    NET_ATTACH_CHECK,// 检查网络附着
    NET_SET_APN,      // APN设置
    NET_CONNECT,     // 建立TCP连接
    NET_SET_KEEPALIVE,// 配置TCP保活
    NET_SET_HEART,   // 配置心跳包
    NET_READY,       // 连接就绪（可收发数据）
    NET_RECONNECT    // 重连中
} Net_State;

extern char UART2_RecvBuf[];
extern volatile uint32_t g_UART2_rx_len;
extern Net_State g_net_state;

void Net_ModuleInit();
void Net_CheckReg();
void Net_CheckAttach();
void NET_Set_APN();
void Net_ConnectTCP();
void Net_SetKeepAlive();
void Net_SetHeartBeat();
char Net_SendData(char *data, uint16_t len);
void Net_RecvData();
void Net_Reconnect();
#endif
