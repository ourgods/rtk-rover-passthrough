#ifndef _PASSTHROUGH_H
#define _PASSTHROUGH_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#define PASSTHROUGH_BUF_SIZE              512U

/* UART4 A/B双片缓冲：每片1024，总共2048字节 */
#define PASSTHROUGH_UART4_SLICE_SIZE     1024U
#define PASSTHROUGH_UART4_BUF_SIZE       \
        (2U * PASSTHROUGH_UART4_SLICE_SIZE)

#define PASSTHROUGH_UART5_BUF_SIZE        512U
#define PASSTHROUGH_UART1_BUF_SIZE        512U

/* 透传缓冲区 */
extern uint8_t uart4_rx_buf[PASSTHROUGH_UART4_BUF_SIZE];
extern uint8_t uart5_rx_buf[PASSTHROUGH_UART5_BUF_SIZE];

/* 透传接收长度*/
extern volatile uint16_t g_passthrough_uart4_rx_len;
extern volatile uint16_t g_passthrough_uart5_rx_len;
extern volatile uint16_t g_passthrough_uart1_rx_len;

/* UART4 A/B缓冲诊断变量 */

/* 单次接收达到或超过旧的512字节上限 */
extern volatile uint32_t dbg_uart4_rx_reached_512_count;

/* 单次接收真正填满当前1024字节分片 */
extern volatile uint32_t dbg_uart4_rx_full_1024_count;

/* 运行过程中出现过的最大单次接收长度 */
extern volatile uint16_t dbg_uart4_rx_max_len;

/* 下一片仍被发送占用，无法立即开始接收 */
extern volatile uint32_t dbg_uart4_pingpong_busy_count;

/* 启动下一片DMA接收失败 */
extern volatile uint32_t dbg_uart4_rx_restart_fail_count;

/* UART5发送失败 */
extern volatile uint32_t dbg_uart5_tx_error_count;

/* 由stm32f1xx_it.c中的HAL回调调用 */
void passthrough_uart4_rx_event_callback(uint16_t size);
void passthrough_uart4_error_callback(void);

/* UART5 (PC) <-> UART4 (UM980) 透传任务 */
void passthrough_task(void *argument);

#endif
