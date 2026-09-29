#ifndef __LORA_LORA_H
#define __LORA_LORA_H

#include <stdint.h>

extern volatile uint8_t dbg_lora_print_enable;
extern volatile uint32_t dbg_lora_init;
extern volatile uint32_t dbg_lora_rtcm_read_cnt;
extern volatile uint32_t dbg_lora_rtcm_read_bytes;
extern volatile uint32_t dbg_lora_pkt_sent_cnt;
extern volatile uint32_t dbg_lora_pkt_sent_bytes;
extern volatile uint32_t dbg_lora_uart1_rx_cnt;
extern volatile uint32_t dbg_lora_uart1_rx_bytes;
extern volatile uint32_t dbg_lora_parse_ready_cnt;

extern volatile uint32_t dbg_l33_data_tx_try;
extern volatile uint32_t dbg_l33_data_tx_ok;
extern volatile uint32_t dbg_l33_data_tx_fail;
extern volatile uint32_t dbg_l33_data_tx_timeout;

/* ========== 移动站链路诊断（IAR watch g_rover_diag 一键展开） ==========
 * 链路: L33 收(UART1) -> 转发 UM980(UART4)
 * 不打印, 全 volatile, IAR/JTAG 在线 watch 零侵入.
 *
 * 归因速查表:
 *   uart1_rx_cnt 不涨           -> 空口断 / L33 死
 *   uart1_rtcm_cnt 停 + nonrtcm 涨 -> AT/状态字符串污染 RTCM 口
 *   uart4_fwd_fail 涨           -> HAL_UART_Transmit_DMA 失败
 *   uart4_sem_fail 涨           -> UART4 TX 背压(UM980 收不过来)
 *   uart1_err 涨                -> L33 串口误码(波特率/线路)
 *   max_gap_ms 飙大             -> 间歇性空口丢包
 *   last_tick 距当前 tick 越来越大 -> 链路已死
 *   rtcm_partial_cnt 涨         -> 收到的段不带 D3 头(帧被拆/碎片)
 */
typedef struct {
    /* ---- L33 接收侧 (UART1) ---- */
    volatile uint32_t uart1_rx_cnt;        /* L33 收到数据段的总次数 */
    volatile uint32_t uart1_rx_bytes;      /* L33 收到的总字节数 */
    volatile uint32_t uart1_rtcm_cnt;      /* 首字节=0xD3 的段数 (RTCM 头) */
    volatile uint32_t uart1_nonrtcm_cnt;   /* 首字节≠0xD3 的段数 (AT/碎片) */
    volatile uint32_t uart1_partial_cnt;   /* 首字节=0xD3 但长度不足3的段 */
    volatile uint32_t uart1_rx_len_last;   /* 最近一次接收长度 */
    volatile uint32_t uart1_rx_len_min;    /* 观察到的最小接收长度 */
    volatile uint32_t uart1_rx_len_max;    /* 观察到的最大接收长度 */
    volatile uint32_t uart1_err;           /* UART1 错误中断次数 (PE/FE/NE/ORE) */

    /* ---- RTCM 帧解析 ---- */
    volatile uint32_t rtcm_frame_len_last; /* 最近一次解析出的完整帧长 */
    volatile uint32_t rtcm_frame_len_max;  /* 最大帧长 */

    /* ---- 转发 UM980 (UART4 TX) ---- */
    volatile uint32_t uart4_fwd_cnt;       /* 成功提交 UART4 DMA 发送的次数 */
    volatile uint32_t uart4_fwd_bytes;     /* 成功转发的字节数 */
    volatile uint32_t uart4_fwd_fail;      /* HAL_UART_Transmit_DMA != HAL_OK */
    volatile uint32_t uart4_sem_fail;      /* UART4_tx 信号量 50ms 超时次数 */

    /* ---- 时序 / 活性 ---- */
    volatile uint32_t last_tick;           /* 最后一次收到 L33 数据的 HAL_GetTick() */
    volatile uint32_t prev_tick;           /* 上一次收到时的 tick (算 gap 用) */
    volatile uint32_t max_gap_ms;          /* 两次接收之间的最大间隔 (ms) */
    volatile uint32_t idle_dead_ms;        /* 当前距离上次接收过去了多久 (任务循环刷新) */
    volatile uint32_t diag_loop_cnt;       /* LORA_task 循环计数 (任务活着) */
} RoverLinkDiag;

extern RoverLinkDiag g_rover_diag;

/* ========== 事件快照环形缓冲 (IAR watch g_rover_events 一键看故障时间线) ==========
 * 解决痛点: 掉 RTK 是瞬态, 等暂停调试时事件早过了, 累计值看不出"断的那一刻".
 * 机制: 检测到关键事件时, 自动把那一刻的状态快照存入环形缓冲, 带 tick 时间戳.
 * IAR watch g_rover_events 展开即可看最近 N 条事件的时间线 + 当时快照.
 *
 * 事件类型:
 *   1 = LINK_DOWN     链路断(超过 LINK_DOWN_THRESH_MS 没收到 L33 数据)
 *   2 = LINK_RECOVER  链路恢复(从 DOWN 重新收到数据, 同时记录断了多久)
 *   3 = AT_BURST      AT/碎片突发(1秒内非RTCM段数 >= AT_BURST_THRESH)
 *   4 = FWD_STALL     转发背压突发(1秒内 fwd_fail+sem_fail >= FWD_STALL_THRESH)
 *   5 = UART_ERR_BURST 串口误码突发(1秒内 uart1_err 增量 >= ERR_BURST_THRESH)
 *
 * 读法: 看 idx_newest 指向最新事件, 往前数 ROVER_EVT_BUF_SIZE 条即历史.
 */
#define ROVER_EVT_BUF_SIZE        8    /* 环形缓冲深度: 最近8条事件 */
#define ROVER_EVT_LINK_DOWN_MS    3000 /* 链路断判定阈值: 3秒无数据 */
#define ROVER_EVT_WINDOW_MS       1000 /* 突发事件检测窗口: 1秒 */
#define ROVER_EVT_AT_BURST        3    /* 1秒内非RTCM段>=3 判AT污染突发 */
#define ROVER_EVT_FWD_STALL       3    /* 1秒内转发失败>=3 判背压突发 */
#define ROVER_EVT_ERR_BURST       3    /* 1秒内串口错误>=3 判误码突发 */

typedef struct {
    volatile uint32_t tick;          /* 事件发生时刻 HAL_GetTick() */
    volatile uint32_t type;          /* 事件类型 1~5 见上 */
    volatile uint32_t aux;           /* 辅助值: LINK_RECOVER=断了多久ms, 其他=突发量 */
    /* ---- 当时的快照 (事后归因用) ---- */
    volatile uint32_t rx_cnt;        /* 累计接收次数 */
    volatile uint32_t rtcm_cnt;      /* 累计RTCM段数 */
    volatile uint32_t nonrtcm_cnt;   /* 累计非RTCM段数 */
    volatile uint32_t fwd_fail;      /* 累计转发失败 */
    volatile uint32_t sem_fail;      /* 累计信号量失败 */
    volatile uint32_t uart_err;      /* 累计串口错误 */
    volatile uint32_t idle_ms;       /* 事件发生时链路空闲时长 */
} RoverEvent;

typedef struct {
    volatile RoverEvent buf[ROVER_EVT_BUF_SIZE]; /* 环形缓冲 */
    volatile uint32_t idx_newest;    /* 最新事件索引 (0..SIZE-1) */
    volatile uint32_t total;         /* 累计事件总数 (含被覆盖的) */
    /* ---- 突发检测用的"上一次基线" (非突发时不写事件) ---- */
    volatile uint32_t prev_nonrtcm;  /* 上一次窗口起点的 nonrtcm 计数 */
    volatile uint32_t prev_fwd_fail; /* 上一次窗口起点的 fwd_fail */
    volatile uint32_t prev_sem_fail; /* 上一次窗口起点的 sem_fail */
    volatile uint32_t prev_uart_err; /* 上一次窗口起点的 uart_err */
    volatile uint32_t burst_chk_tick;/* 上一次突发检测的 tick */
    volatile uint8_t  link_down_flag;/* 当前是否处于链路断状态(防重复记LINK_DOWN) */
} RoverEventLog;

extern RoverEventLog g_rover_events;

/* 事件类型常量 (便于 watch 时一眼识别) */
#define ROVER_EVT_TYPE_LINK_DOWN      1
#define ROVER_EVT_TYPE_LINK_RECOVER   2
#define ROVER_EVT_TYPE_AT_BURST       3
#define ROVER_EVT_TYPE_FWD_STALL      4
#define ROVER_EVT_TYPE_UART_ERR_BURST 5

#endif
