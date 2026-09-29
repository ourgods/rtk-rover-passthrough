#ifndef __L33_LORA_H
#define __L33_LORA_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <string.h>


// L33配置参数
#define L33_UART_HANDLE    huart1          // 对应STM32的UART外设（需自行初始化）
#define L33_BAUDRATE      115200            // 串口波特率（与AT+UART一致）
#define L33_BASE_STATION_ADDR  0x0001          // 主机地址
//#define L33_MASTER_ADDR  0x0000          // 主机地址
//#define L33_SLAVE1_ADDR  0x0001          // 子节点1地址
//#define L33_SLAVE2_ADDR  0x0002          // 子节点2地址
#define L33_MAX_DATA_LEN 64             // 最大数据包长度（文档1-1.4.4节）
#define L33_CMD_TIMEOUT  500            // AT指令超时时间（ms）

#define LORA_MAX_PACKET_LEN 249
#define L33_RCV_BUF_SIZE  (LORA_MAX_PACKET_LEN+3)
#define L33_SEND_BUF_SIZE (LORA_MAX_PACKET_LEN+3)
// 主从模式枚举
typedef enum 
{
    L33_ROLE_MASTER = 0,  // 主机
    L33_ROLE_SLAVE       // 从机
} L33_RoleTypeDef;

// 数据接收结构体
typedef struct 
{
    uint8_t buf[L33_MAX_DATA_LEN];  // 接收数据缓冲区
    uint16_t len;                  // 数据长度
    uint16_t src_addr;              // 源地址（主机接收时有效）
    uint8_t is_ready;               // 数据接收完成标志
} L33_ReceiveTypeDef;

// 全局变量

extern uint8_t L33_uart_rx_buf[L33_RCV_BUF_SIZE];
extern uint8_t L33_uart_tx_buf[L33_SEND_BUF_SIZE];
extern uint16_t L33_uart_rx_len;
extern L33_ReceiveTypeDef L33_rx_data;

void L33_Init();

uint8_t LoRa_PassThrough_Parse(uint8_t *buf, uint8_t len,uint8_t *out_chan, uint8_t *out_data);
uint8_t LoRa_PassThrough_Send( uint8_t channel, uint8_t *data, uint8_t len);
void L33_ParseData(uint8_t *data, uint16_t len);
void restart_L33_DMA_RCV();

#endif
