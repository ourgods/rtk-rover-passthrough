#include "main.h"
#include "cmsis_os.h"
#include "um980_driver.h"
#include "rtk_task.h"
#include "lora_task.h"
#include "console.h"
#include <string.h>
#include <stdio.h>

/* ========== RTCM 共享缓冲（RTK_task写，LORA_task读） ========== */
uint8_t g_rtcm_buf[RTCM_CACHE_SIZE];
volatile uint16_t g_rtcm_len = 0;
volatile uint32_t g_rtcm_seq = 0;
osMutexId g_rtcm_mutex = NULL;

/* ========== dump once 控制（console写1，RTK_task消费后清0） ========== */
volatile uint8_t g_uart4_dump_once = 0;

/* ========== 基站运行状态（console可读） ========== */
volatile RoverStatusEnum g_rover_status = ROVER_STATUS_IDLE;

static char status_msg[128] = {0};

/* ========== RTK_task调试计数器 ========== */
volatile uint32_t dbg_rtk_loop_cnt = 0;
volatile uint32_t dbg_rtcm_write_cnt = 0;
volatile uint32_t dbg_rtcm_write_bytes = 0;
volatile uint32_t dbg_um980_rx_rtcm_cnt = 0;
volatile uint32_t dbg_um980_rx_ascii_cnt = 0;
volatile uint32_t dbg_um980_rx_other_cnt = 0;
volatile uint32_t dbg_base_wait_elapsed = 0;
volatile uint32_t dbg_rover_status_field = 0xFFFFFFFFU;  /* 最近一次轮询到的BASEINFO Status */
volatile uint32_t dbg_uart4_idle_cnt = 0;
volatile uint32_t dbg_uart4_rx_len_last = 0;
volatile uint8_t dbg_uart4_dump_enable = 0;
volatile uint8_t dbg_uart4_dump_once = 0;
volatile uint8_t dbg_pause_uart4_rx = 0;
volatile uint8_t dbg_status_report_enable = 0;
volatile uint8_t dbg_rtcm_sample_enable = 1;
volatile uint32_t dbg_rtcm_sample_cnt = 0;
volatile uint32_t dbg_rtcm_sample_bytes = 0;
volatile uint16_t dbg_rtcm_sample_rx_len = 0;
volatile uint16_t dbg_rtcm_sample_dump_len = 0;
volatile uint16_t dbg_rtcm_sample_frame_len = 0;
volatile uint8_t dbg_rtcm_sample_buf[RTK_RTCM_SAMPLE_MAX_BYTES] = {0};

/* ========== 函数声明 ========== */
static void rtk_process_uart4_rx(void);
static void rtk_capture_rtcm_sample(uint8_t *data, uint16_t len);
static HAL_StatusTypeDef rtcm_config_outputs(void);
static void base_station_error_handle(uint8_t err_code);

extern volatile uint32_t dbg_uart4_err_pe;
extern volatile uint32_t dbg_uart4_err_fe;
extern volatile uint32_t dbg_uart4_err_ne;
extern volatile uint32_t dbg_uart4_err_ore;

void RTK_task(void const * argument)
{
    static uint32_t Antana_check_counter = 0;
    static uint32_t last_uart4_idle_cnt = 0;
    static uint32_t last_rtcm_write_cnt = 0;
    static uint32_t last_rtcm_write_bytes = 0;
    static uint32_t last_lora_pkt_sent_cnt = 0;
    static uint32_t base_wait_start_tick = 0;
    static uint32_t last_base_wait_report = 0;
    static uint32_t last_base_poll = 0;

    osDelay(1000);
    __HAL_UART_CLEAR_IDLEFLAG(&huart4);
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    while (osSemaphoreAcquire(UART4_rx_semaphoreHandle, 0) == osOK) {}
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, um980_rx_buf, UM980_RX_BUF_LEN);
    __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);

    g_rover_status = ROVER_STATUS_IDLE;

    for (;;)
    {
        rtk_process_uart4_rx();

        switch (g_rover_status)
        {
            case ROVER_STATUS_IDLE:
            {
                g_rover_status = ROVER_STATUS_INIT;
                UM980_printf("***Rover System init start...\r\n");
                break;
            }
            case ROVER_STATUS_INIT:
            {
                UM980_printf("[RTK] RST low\r\n");
                UM980_RST = 0;
                osDelay(500);
                UM980_RST = 1;
                UM980_printf("[RTK] RST high, wait 3s\r\n");
                osDelay(3000);
                UM980_printf("[RTK] um980_init start\r\n");

                g_rover_status = ROVER_STATUS_ROVER_CONFIG;
                if (um980_init(UM980_MODE_BASE) != HAL_OK)
                {
                    base_station_error_handle(0x01);
                    UM980_printf("[RTK] init FAIL step=%d ret=%d\r\n",
                                 dbg_um980_init_step, dbg_um980_init_ret);
                    g_rover_status = ROVER_STATUS_ERROR;
                    break;
                }
                UM980_printf("[RTK] init OK step=%d ret=%d pvtalg=%d\r\n",
                             dbg_um980_init_step, dbg_um980_init_ret, dbg_um980_pvtalg_ret);
                break;
            }
            case ROVER_STATUS_ROVER_CONFIG:
            {
                UM980_printf("[RTK] rover auto config...\r\n");
                if (um980_set_base_auto(BASE_AUTO_TIME, BASE_DISTANCE) != HAL_OK)
                {
                    base_station_error_handle(0x02);
                    UM980_printf("[RTK] rover auto FAIL ret=%d save=%d\r\n",
                                 dbg_um980_base_cmd_ret, dbg_um980_base_save_ret);
                    g_rover_status = ROVER_STATUS_ERROR;
                }
                else
                {
                    base_wait_start_tick = HAL_GetTick();
                    dbg_base_wait_elapsed = 0;
                    last_base_wait_report = 0;
                    last_base_poll = 0;
                    g_rover_status = ROVER_STATUS_ROVER_WAIT;
                    UM980_printf("[RTK] rover wait start, poll BASEINFO until valid (max %ds) ret=%d save=%d\r\n",
                                 BASE_SURVEY_MAX_WAIT,
                                 dbg_um980_base_cmd_ret, dbg_um980_base_save_ret);
                }
                break;
            }
            case ROVER_STATUS_RUNNING:
            {
                dbg_rtk_loop_cnt++;

                osDelay(1);

                Antana_check_counter++;
                if (dbg_status_report_enable != 0 &&
                    Antana_check_counter >= STATUS_REPORT_INTERVAL)
                {
                    uint32_t idle_delta;
                    uint32_t wr_delta;
                    uint32_t byte_delta;
                    uint32_t lora_pkt_delta;
                    Antana_check_counter = 0;

                    idle_delta = dbg_uart4_idle_cnt - last_uart4_idle_cnt;
                    wr_delta = dbg_rtcm_write_cnt - last_rtcm_write_cnt;
                    byte_delta = dbg_rtcm_write_bytes - last_rtcm_write_bytes;
                    lora_pkt_delta = dbg_lora_pkt_sent_cnt - last_lora_pkt_sent_cnt;
                    last_uart4_idle_cnt = dbg_uart4_idle_cnt;
                    last_rtcm_write_cnt = dbg_rtcm_write_cnt;
                    last_rtcm_write_bytes = dbg_rtcm_write_bytes;
                    last_lora_pkt_sent_cnt = dbg_lora_pkt_sent_cnt;

                    UM980_printf("[DBG] idle=%d(+%d) len=%d rtcm=%d(+%d) bytes=%d(+%d) lora=%d(+%d) tx=%d/%d/%d rx=%d/%d/%d\r\n",
                                 dbg_uart4_idle_cnt, idle_delta, dbg_uart4_rx_len_last,
                                 dbg_rtcm_write_cnt, wr_delta, dbg_rtcm_write_bytes, byte_delta,
                                 dbg_lora_pkt_sent_cnt, lora_pkt_delta,
                                 dbg_um980_last_tx_ret, dbg_um980_tx_busy, dbg_um980_tx_take_timeout,
                                 dbg_um980_rx_rtcm_cnt, dbg_um980_rx_ascii_cnt, dbg_um980_rx_other_cnt);
                }
                break;
            }
            case ROVER_STATUS_ERROR:
            {
                osDelay(1000);
                break;
            }
            default:
                break;
        }
        osDelay(10);
    }
}

static void rtk_process_uart4_rx(void)
{
    uint16_t rx_len;
    uint16_t rtcm_offset;

    if (dbg_pause_uart4_rx != 0)
    {
        return;
    }

    if (osSemaphoreAcquire(UART4_rx_semaphoreHandle, 0) != osOK)
    {
        return;
    }

    rx_len = um980_rx_len;
    if (rx_len > 0 && rx_len <= UM980_RX_BUF_LEN)
    {
        if (dbg_uart4_dump_enable != 0 || dbg_uart4_dump_once != 0)
        {
            Debug_DumpHex("UART4 RX", um980_rx_buf, rx_len);
            dbg_uart4_dump_once = 0;
        }

        rtcm_offset = 0;

        while (rtcm_offset < rx_len && um980_rx_buf[rtcm_offset] != RTCM_SYNC_BYTE)
        {
            rtcm_offset++;
        }

        if (rtcm_offset < rx_len)
        {
            uint16_t rtcm_len = rx_len - rtcm_offset;
            if (rtcm_len > RTCM_CACHE_SIZE)
            {
                rtcm_len = RTCM_CACHE_SIZE;
            }

            dbg_um980_rx_rtcm_cnt++;
            rtk_capture_rtcm_sample(&um980_rx_buf[rtcm_offset], rtcm_len);
            if (g_rtcm_mutex != NULL)
            {
                osMutexWait(g_rtcm_mutex, osWaitForever);
                memcpy(g_rtcm_buf, &um980_rx_buf[rtcm_offset], rtcm_len);
                g_rtcm_len = rtcm_len;
                g_rtcm_seq++;
                osMutexRelease(g_rtcm_mutex);
            }
            dbg_rtcm_write_cnt++;
            dbg_rtcm_write_bytes += rtcm_len;
        }
        else if (um980_rx_buf[0] == '#' || um980_rx_buf[0] == '$' ||
                 (um980_rx_buf[0] >= 'A' && um980_rx_buf[0] <= 'Z') ||
                 (um980_rx_buf[0] >= 'a' && um980_rx_buf[0] <= 'z'))
        {
            dbg_um980_rx_ascii_cnt++;
        }
        else
        {
            dbg_um980_rx_other_cnt++;
        }
    }

    memset(um980_rx_buf, 0, UM980_RX_BUF_LEN);
    HAL_UART_AbortReceive(&huart4);
    __HAL_UART_CLEAR_IDLEFLAG(&huart4);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, um980_rx_buf, UM980_RX_BUF_LEN);
    __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
}

static void rtk_capture_rtcm_sample(uint8_t *data, uint16_t len)
{
    uint16_t payload_len;
    uint16_t copy_len;

    if (dbg_rtcm_sample_enable == 0 || data == NULL || len == 0)
    {
        return;
    }

    copy_len = (len > RTK_RTCM_SAMPLE_MAX_BYTES) ? RTK_RTCM_SAMPLE_MAX_BYTES : len;
    dbg_rtcm_sample_rx_len = len;
    dbg_rtcm_sample_dump_len = copy_len;
    memcpy((void *)dbg_rtcm_sample_buf, data, copy_len);

    if (len >= 3 && data[0] == RTCM_SYNC_BYTE)
    {
        payload_len = (((uint16_t)data[1] & 0x03U) << 8) | data[2];
        dbg_rtcm_sample_frame_len = payload_len + 6U;
    }
    else
    {
        dbg_rtcm_sample_frame_len = 0;
    }

    dbg_rtcm_sample_cnt++;
    dbg_rtcm_sample_bytes += len;
}

void RTK_ResetRtcmSample(void)
{
    dbg_rtcm_sample_cnt = 0;
    dbg_rtcm_sample_bytes = 0;
    dbg_rtcm_sample_rx_len = 0;
    dbg_rtcm_sample_dump_len = 0;
    dbg_rtcm_sample_frame_len = 0;
    memset((void *)dbg_rtcm_sample_buf, 0, RTK_RTCM_SAMPLE_MAX_BYTES);
}

void RTK_PrintDiag(void)
{
    Printf("\r\nRTK state=%d run=%d wait=%d baseStatus=%lu log=%d\r\n",
           g_rover_status, dbg_rtk_loop_cnt, dbg_base_wait_elapsed,
           (unsigned long)dbg_rover_status_field, dbg_status_report_enable);
    Printf("UART4 idle=%d len=%d err=%d/%d/%d/%d rx=%d/%d/%d\r\n",
           dbg_uart4_idle_cnt, dbg_uart4_rx_len_last,
           dbg_uart4_err_pe, dbg_uart4_err_fe, dbg_uart4_err_ne, dbg_uart4_err_ore,
           dbg_um980_rx_rtcm_cnt, dbg_um980_rx_ascii_cnt, dbg_um980_rx_other_cnt);
    Printf("UM980 init_step=%d init_ret=%d pvtalg=%d base=%d/%d tx=%d busy=%d timeout=%d ack=%d/%d/%d\r\n",
           dbg_um980_init_step, dbg_um980_init_ret, dbg_um980_pvtalg_ret,
           dbg_um980_base_cmd_ret, dbg_um980_base_save_ret,
           dbg_um980_last_tx_ret, dbg_um980_tx_busy, dbg_um980_tx_take_timeout,
           dbg_um980_ack_ok_cnt, dbg_um980_ack_err_cnt, dbg_um980_ack_timeout_cnt);
    Printf("RTCM write=%d bytes=%d sample=%d rx_len=%d frame_len=%d dump=%d\r\n",
           dbg_rtcm_write_cnt, dbg_rtcm_write_bytes,
           dbg_rtcm_sample_cnt, dbg_rtcm_sample_rx_len,
           dbg_rtcm_sample_frame_len, dbg_rtcm_sample_dump_len);
}

static HAL_StatusTypeDef rtcm_config_outputs(void)
{
    HAL_StatusTypeDef unlog_ret;
    HAL_StatusTypeDef rtcm1006_ret;
    HAL_StatusTypeDef rtcm1124_ret;
    HAL_StatusTypeDef rtcm1033_ret;
    HAL_StatusTypeDef rtcm_save_ret;


    unlog_ret = um980_send_cmd_trace((uint8_t*)"unlog com1", 1500);
    //um980_send_cmd_trace((uint8_t*)"UNMASK GPS", 1500);
    //um980_send_cmd_trace((uint8_t*)"UNMASK GLO", 1500);
    //um980_send_cmd_trace((uint8_t*)"UNMASK GAL", 1500);
    //um980_send_cmd_trace((uint8_t*)"UNMASK BDS", 1500);
    //um980_send_cmd_trace((uint8_t*)"UNMASK QZSS", 1500);
    rtcm1006_ret = um980_send_cmd_trace((uint8_t*)"rtcm1006 com1 10", 1500);
    rtcm1124_ret = um980_send_cmd_trace((uint8_t*)"rtcm1124 com1 1", 1500);
    rtcm1033_ret = um980_send_cmd_trace((uint8_t*)"rtcm1033 com1 10", 1500);
    rtcm_save_ret = um980_send_cmd_trace((uint8_t*)"saveconfig", 1500);

    UM980_printf("[DBG] RTCM cfg ret: save=%d ack=%d/%d/%d\r\n",
                 rtcm_save_ret, dbg_um980_ack_ok_cnt, dbg_um980_ack_err_cnt, dbg_um980_ack_timeout_cnt);

    if (unlog_ret != HAL_OK
        || rtcm1006_ret != HAL_OK || rtcm1124_ret != HAL_OK || rtcm1033_ret != HAL_OK || rtcm_save_ret != HAL_OK)
    {
        return HAL_ERROR;
    }
    else
    {
       UM980_printf("[DBG] RTCM cfg success");
    }

    return HAL_OK;
}

static void base_station_error_handle(uint8_t err_code)
{
    switch (err_code)
    {
        case 0x01:
            sprintf(status_msg, "ERROR 0x01: UM980 init failed\r\n");
            break;
        case 0x02:
            sprintf(status_msg, "ERROR 0x02: Base station config failed\r\n");
            break;
        case 0x03:
            sprintf(status_msg, "ERROR 0x03: Main antenna (ANT1) error\r\n");
            break;
        default:
            sprintf(status_msg, "ERROR 0xFF: Unknown error\r\n");
            break;
    }

    UM980_Printf_DMA((uint8_t*)status_msg, strlen(status_msg));

    RTK_LED = 0;
}
