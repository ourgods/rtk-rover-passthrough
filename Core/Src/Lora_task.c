#include "main.h"
#include "cmsis_os.h"
#include "L33_driver.h"
#include "lora_task.h"
#include "rtk_task.h"
#include "console.h"
#include <string.h>
#include <stdio.h>

extern volatile uint8_t g_uart1_err_flag;
extern osSemaphoreId_t UART4_tx_semaphoreHandle;

#define LORA_RTCM_CHUNK_MAX 50
#define LORA_RTCM_CACHE_MAX 1024

volatile uint8_t dbg_lora_print_enable = 0;  /* 关闭打印 */
volatile uint32_t dbg_lora_init = 0;
volatile uint32_t dbg_lora_rtcm_read_cnt = 0;
volatile uint32_t dbg_lora_rtcm_read_bytes = 0;
volatile uint32_t dbg_lora_pkt_sent_cnt = 0;
volatile uint32_t dbg_lora_pkt_sent_bytes = 0;
volatile uint32_t dbg_lora_uart1_rx_cnt = 0;
volatile uint32_t dbg_lora_uart1_rx_bytes = 0;
volatile uint32_t dbg_lora_parse_ready_cnt = 0;
volatile uint8_t h0 ;
volatile uint8_t h1;
volatile uint8_t h2;
volatile uint8_t h3;
volatile uint16_t rtcm_len = 0;
volatile uint16_t frame_len = 0;

/* ========== 移动站链路诊断实例（IAR watch g_rover_diag） ==========
 * 用运行时初始化函数, 避免 C89/C99 指定初始化器争议, 同时保证 min 字段为最大值.
 */
RoverLinkDiag g_rover_diag;

/* ========== 事件快照环形缓冲（IAR watch g_rover_events） ========== */
RoverEventLog g_rover_events;

/* 记录一条事件到环形缓冲 (供本文件内部调用, 无锁, 单任务上下文安全) */
static void rover_evt_record(uint32_t type, uint32_t aux, uint32_t idle_ms)
{
    uint32_t idx;
    idx = (g_rover_events.idx_newest + 1U) % ROVER_EVT_BUF_SIZE;
    g_rover_events.buf[idx].tick        = HAL_GetTick();
    g_rover_events.buf[idx].type        = type;
    g_rover_events.buf[idx].aux         = aux;
    g_rover_events.buf[idx].rx_cnt      = g_rover_diag.uart1_rx_cnt;
    g_rover_events.buf[idx].rtcm_cnt    = g_rover_diag.uart1_rtcm_cnt;
    g_rover_events.buf[idx].nonrtcm_cnt = g_rover_diag.uart1_nonrtcm_cnt;
    g_rover_events.buf[idx].fwd_fail    = g_rover_diag.uart4_fwd_fail;
    g_rover_events.buf[idx].sem_fail    = g_rover_diag.uart4_sem_fail;
    g_rover_events.buf[idx].uart_err    = g_rover_diag.uart1_err;
    g_rover_events.buf[idx].idle_ms     = idle_ms;
    g_rover_events.idx_newest = idx;
    g_rover_events.total++;
}

void LORA_task(void const * argument)
{
    static uint8_t rtcm_snapshot[LORA_RTCM_CACHE_MAX];
    uint32_t last_rtcm_seq = 0;
    static uint32_t last_diag_tick = 0;

    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    while (osSemaphoreAcquire(UART1_rx_semaphoreHandle, 0) == osOK) {}
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, L33_uart_rx_buf, sizeof(L33_uart_rx_buf));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    

    osDelay(3000);
    L33_Init();
    dbg_lora_init++;

    /* 诊断计数器初始化 (min 字段必须设最大值, 首次接收才能覆盖) */
    g_rover_diag.uart1_rx_len_min = 0xFFFFFFFFU;
    /* 事件检测基线初始化 */
    g_rover_events.burst_chk_tick = HAL_GetTick();
    g_rover_events.prev_nonrtcm  = 0;
    g_rover_events.prev_fwd_fail = 0;
    g_rover_events.prev_sem_fail = 0;
    g_rover_events.prev_uart_err = 0;
    g_rover_events.link_down_flag = 0;
    g_rover_events.idx_newest = ROVER_EVT_BUF_SIZE - 1U; /* 初始指向最后, 首次记录落 buf[0] */
    g_rover_events.total = 0;

    // L33_printf("*** LORA_task ready\r\n");  // 透传模式关闭所有打印
    //移动站接收lora信号，直接透传um980
    for (;;)
    {
        uint32_t now_tick_loop;

        /* ---- 任务活性 + 链路空闲计时 (每轮刷新, watch 即可看活/死) ---- */
        g_rover_diag.diag_loop_cnt++;
        now_tick_loop = HAL_GetTick();
        /* last_tick==0 表示从未收到过数据, 此时不算断(还在等L33初始化/首帧) */
        if (g_rover_diag.last_tick != 0)
        {
            g_rover_diag.idle_dead_ms = now_tick_loop - g_rover_diag.last_tick;
        }

        /* ===== 事件检测 1: LINK_DOWN / LINK_RECOVER ===== */
        if (g_rover_diag.last_tick != 0)
        {
            if (g_rover_diag.idle_dead_ms >= ROVER_EVT_LINK_DOWN_MS)
            {
                if (g_rover_events.link_down_flag == 0)
                {
                    g_rover_events.link_down_flag = 1;
                    rover_evt_record(ROVER_EVT_TYPE_LINK_DOWN, g_rover_diag.idle_dead_ms,
                                     g_rover_diag.idle_dead_ms);
                }
            }
        }

        /* ===== 事件检测 2: 突发类(AT/转发/误码) 窗口检测 ===== */
        if ((now_tick_loop - g_rover_events.burst_chk_tick) >= ROVER_EVT_WINDOW_MS)
        {
            uint32_t d_nonrtcm  = g_rover_diag.uart1_nonrtcm_cnt - g_rover_events.prev_nonrtcm;
            uint32_t d_fwd_fail = g_rover_diag.uart4_fwd_fail   - g_rover_events.prev_fwd_fail;
            uint32_t d_sem_fail = g_rover_diag.uart4_sem_fail   - g_rover_events.prev_sem_fail;
            uint32_t d_uart_err = g_rover_diag.uart1_err        - g_rover_events.prev_uart_err;

            /* 更新基线 */
            g_rover_events.prev_nonrtcm  = g_rover_diag.uart1_nonrtcm_cnt;
            g_rover_events.prev_fwd_fail = g_rover_diag.uart4_fwd_fail;
            g_rover_events.prev_sem_fail = g_rover_diag.uart4_sem_fail;
            g_rover_events.prev_uart_err = g_rover_diag.uart1_err;
            g_rover_events.burst_chk_tick = now_tick_loop;

            if (d_nonrtcm >= ROVER_EVT_AT_BURST)
            {
                rover_evt_record(ROVER_EVT_TYPE_AT_BURST, d_nonrtcm, g_rover_diag.idle_dead_ms);
            }
            if ((d_fwd_fail + d_sem_fail) >= ROVER_EVT_FWD_STALL)
            {
                rover_evt_record(ROVER_EVT_TYPE_FWD_STALL, d_fwd_fail + d_sem_fail,
                                 g_rover_diag.idle_dead_ms);
            }
            if (d_uart_err >= ROVER_EVT_ERR_BURST)
            {
                rover_evt_record(ROVER_EVT_TYPE_UART_ERR_BURST, d_uart_err,
                                 g_rover_diag.idle_dead_ms);
            }
        }

        if (osSemaphoreAcquire(UART1_rx_semaphoreHandle, 10) == osOK)
        {
            if (L33_uart_rx_len > 0)
            {
              uint32_t now_tick;
              uint32_t gap_ms;
              uint16_t rxlen = L33_uart_rx_len;

              /* ---- 时序统计: 两次接收间隔 ---- */
              now_tick = HAL_GetTick();
              if (g_rover_diag.prev_tick != 0)
              {
                  gap_ms = now_tick - g_rover_diag.prev_tick;
                  if (gap_ms > g_rover_diag.max_gap_ms) g_rover_diag.max_gap_ms = gap_ms;
              }
              g_rover_diag.prev_tick = now_tick;

              /* ===== 事件检测 1b: LINK_RECOVER (收到数据且之前是DOWN) =====
               * 注意: 必须在更新 last_tick 之前算 down_dur, 否则恒为0 */
              if (g_rover_events.link_down_flag != 0)
              {
                  uint32_t down_dur = now_tick - g_rover_diag.last_tick;
                  g_rover_events.link_down_flag = 0;
                  rover_evt_record(ROVER_EVT_TYPE_LINK_RECOVER, down_dur, 0);
              }

              g_rover_diag.last_tick = now_tick;

              /* ---- L33 接收侧统计 ---- */
              g_rover_diag.uart1_rx_cnt++;
              g_rover_diag.uart1_rx_bytes += rxlen;
              g_rover_diag.uart1_rx_len_last = rxlen;
              if (rxlen < g_rover_diag.uart1_rx_len_min) g_rover_diag.uart1_rx_len_min = rxlen;
              if (rxlen > g_rover_diag.uart1_rx_len_max) g_rover_diag.uart1_rx_len_max = rxlen;

              h0 = (rxlen > 0) ? L33_uart_rx_buf[0] : 0;
              h1 = (rxlen > 1) ? L33_uart_rx_buf[1] : 0;
              h2 = (rxlen > 2) ? L33_uart_rx_buf[2] : 0;
              h3 = (rxlen > 3) ? L33_uart_rx_buf[3] : 0;

              /* ---- 首字节分类: RTCM头 / 非RTCM(AT污染·碎片) ---- */
              if (h0 == 0xD3)
              {
                  if (rxlen >= 3)
                  {
                      uint16_t rtcm_payload;
                      uint16_t flen;
                      g_rover_diag.uart1_rtcm_cnt++;
                      rtcm_payload = (((uint16_t)L33_uart_rx_buf[1] & 0x03) << 8)
                                   | L33_uart_rx_buf[2];
                      flen = rtcm_payload + 6;   /* RTCM总长度 */
                      rtcm_len = rtcm_payload;
                      frame_len = flen;
                      g_rover_diag.rtcm_frame_len_last = flen;
                      if (flen > g_rover_diag.rtcm_frame_len_max) g_rover_diag.rtcm_frame_len_max = flen;
                  }
                  else
                  {
                      g_rover_diag.uart1_partial_cnt++;
                  }
              }
              else
              {
                  g_rover_diag.uart1_nonrtcm_cnt++;
              }
               
                // 转发给UM980 (UART4)
                if (L33_uart_rx_len > 0)
                {
                  HAL_StatusTypeDef ret;
                  /* DMA发送 */
                  static uint8_t lora_uart4_tx_buf[L33_RCV_BUF_SIZE];
                  if (osSemaphoreAcquire(UART4_tx_semaphoreHandle, 50) == osOK)
                  {
                      memcpy(lora_uart4_tx_buf, L33_uart_rx_buf, L33_uart_rx_len);

                      ret = HAL_UART_Transmit_DMA(&huart4, lora_uart4_tx_buf, L33_uart_rx_len);
                      if (ret != HAL_OK)
                      {
                          osSemaphoreRelease(UART4_tx_semaphoreHandle);
                          g_rover_diag.uart4_fwd_fail++;
                      }
                      else
                      {
                          g_rover_diag.uart4_fwd_cnt++;
                          g_rover_diag.uart4_fwd_bytes += L33_uart_rx_len;
                      }
                  }
                  else
                  {
                      /* 信号量 50ms 没拿到 = UART4 TX 背压 */
                      g_rover_diag.uart4_sem_fail++;
                  }
                }
            }
            L33_uart_rx_len = 0;
            memset(L33_uart_rx_buf, 0, sizeof(L33_uart_rx_buf));
            restart_L33_DMA_RCV();
        }

        if (g_uart1_err_flag)
        {
            g_uart1_err_flag = 0;
            // L33_printf("UART1 err recovered\r\n");  // 透传关闭
            restart_L33_DMA_RCV();
        }
    }
}
