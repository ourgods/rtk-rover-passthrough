
#ifndef _CONSOLE_H
#define _CONSOLE_H

#include <stdint.h>

extern volatile uint8_t g_uart5_rx_len;
extern volatile uint32_t dbg_uart5_start_ret;
extern volatile uint32_t dbg_uart5_restart_ret;
extern volatile uint32_t dbg_uart5_idle_cnt;
extern volatile uint32_t dbg_uart5_wakeup_cnt;
extern volatile uint32_t dbg_uart5_txcplt_cnt;
extern volatile uint32_t dbg_console_loop_cnt;
extern volatile uint8_t dbg_uart5_last_len;
extern volatile uint8_t dbg_uart5_last_buf[8];

void Printf(uint8_t *from,...);
void UM980_Printf_DMA(uint8_t *str,uint32_t len);
void Debug_DumpHex(const char *tag, uint8_t *data, uint16_t len);

void AIR_printf(char *str);
void UM980_printf(const char *fmt, ...);
void L33_printf(const char *from,...);
#endif
