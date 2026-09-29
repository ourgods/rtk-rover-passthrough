
#ifndef _IMU_H
#define _IMU_H

#define SIZE_OF_UART3_TX_BUF 100
#define SIZE_OF_UART3_RX_BUF 100


extern uint8_t g_UART3_rx_buffer[];
extern volatile uint32_t g_UART3_rx_len;
#endif