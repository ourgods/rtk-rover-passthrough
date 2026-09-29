#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "console.h"
#include "rtk_task.h"
#include "um980_driver.h"
#include "lora_task.h"
#include "L33_driver.h"

#define DEVICE_ID_LENGTH 5

extern volatile uint32_t dbg_uart1_txcplt_cnt;

//#define UM980_DEBUG    // 透传模式关闭
//#define L33_DEBUG      // 透传模式关闭
#define RTCM_SAMPLE_PRINT_BYTES 16
#define UART5_PRINT_TIMEOUT_MS 20

static char* Hardware_Ver = "1.0";
static char* Software_Ver = "1.0";
static char* Srelease_time = "2024.10";
static char* Hrelease_time = "2024.10";

volatile uint8_t g_uart5_rx_len = 0;

uint8_t g_printf_buffer[500];
//gai
uint8_t g_scanf_buffer[200];

volatile uint32_t dbg_uart5_start_ret = 0;
volatile uint32_t dbg_uart5_restart_ret = 0;
volatile uint32_t dbg_uart5_idle_cnt = 0;
volatile uint32_t dbg_uart5_wakeup_cnt = 0;
volatile uint32_t dbg_uart5_txcplt_cnt = 0;
volatile uint32_t dbg_console_loop_cnt = 0;
volatile uint8_t dbg_uart5_last_len = 0;
volatile uint8_t dbg_uart5_last_buf[8] = {0};

static uint16_t printf_clamp_len(int len, uint16_t buf_size)
{
    if (len <= 0)
    {
        return 0;
    }
    if ((uint16_t)len >= buf_size)
    {
        return (uint16_t)(buf_size - 1U);
    }
    return (uint16_t)len;
}

static void uart5_print_bytes(uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
    {
        return;
    }

    if (osSemaphoreAcquire(UART5_tx_semaphoreHandle, 20) != osOK)
    {
        return;
    }

    _KHAL_UART_DISABLE_RX(&huart5);
    HAL_UART_Transmit(&huart5, data, len, UART5_PRINT_TIMEOUT_MS);
    _KHAL_UART_ENABLE_RX(&huart5);
    osSemaphoreRelease(UART5_tx_semaphoreHandle);
}

void UM980_printf(const char *fmt, ...)
{
    #ifdef UM980_DEBUG
    va_list arg_list;
    uint16_t len;

    va_start(arg_list, fmt);
    len = printf_clamp_len(vsnprintf((char *)g_printf_buffer, sizeof(g_printf_buffer), fmt, arg_list),
                           sizeof(g_printf_buffer));
    va_end(arg_list);
    if (len != 0)
    {
        uart5_print_bytes(g_printf_buffer, len);
    }
    #endif
}

void L33_printf(const char *from,...)
{
    #ifdef L33_DEBUG
    va_list arg_list;
    uint16_t len;

    va_start(arg_list, from);
    len = printf_clamp_len(vsnprintf((char *)g_printf_buffer, sizeof(g_printf_buffer), from, arg_list),
                           sizeof(g_printf_buffer));
    va_end(arg_list);
    if (len != 0)
    {
        uart5_print_bytes(g_printf_buffer, len);
    }
    #endif
}

void UM980_Printf_DMA(uint8_t *str,uint32_t len)
{
    #ifdef UM980_DEBUG
    if (len > 0xFFFFU)
    {
        len = 0xFFFFU;
    }
    uart5_print_bytes(str, (uint16_t)len);
    #endif
}

void Printf(uint8_t *from,...)
{
    uint16_t len;
    va_list arg_list;

    va_start(arg_list,from);
    len = printf_clamp_len(vsnprintf((char *)g_printf_buffer,sizeof(g_printf_buffer),(char const *)from,arg_list),
                           sizeof(g_printf_buffer));
    va_end(arg_list);
    if (len != 0)
    {
        uart5_print_bytes(g_printf_buffer, len);
    }
}

void Debug_DumpHex(const char *tag, uint8_t *data, uint16_t len)
{
    uint16_t i;
    char line[96];

    if (data == NULL)
    {
        return;
    }

    UM980_printf("[%s] len=%d\r\n", tag, len);
    for (i = 0; i < len; i += 16)
    {
        uint16_t j;
        uint16_t pos = (uint16_t)snprintf(line, sizeof(line), "%04X:", i);
        for (j = 0; j < 16 && (i + j) < len; j++)
        {
            pos += (uint16_t)snprintf(&line[pos], sizeof(line) - pos,
                                      " %02X", data[i + j]);
            if (pos >= sizeof(line) - 5)
            {
                break;
            }
        }
        snprintf(&line[pos], sizeof(line) - pos, "\r\n");
        UM980_printf("%s", line);
    }
}

void AIR_printf(char *str)
{
    Printf((uint8_t *)str);
}

static void reset_buffer_restart_DMA()
{
    memset(g_scanf_buffer, 0, sizeof(g_scanf_buffer));
    g_uart5_rx_len = 0;
    dbg_uart5_restart_ret = HAL_UARTEx_ReceiveToIdle_IT(&huart5, g_scanf_buffer, sizeof(g_scanf_buffer));
}

static void welcome()
{
    Printf("\r\nSYSTEM running......\r\n");
    Printf("\r\n%s(%s)", Hardware_Ver, Hrelease_time);
    Printf("\r\n%s(%s)", Software_Ver, Srelease_time);
    Printf("\r\n\"?\"or\"help\"");
    Printf("\r\n\r\n************************************************\r\n");
}

static void command_list()
{
    Printf("\r\n\r\n\r\n");
    Printf("  ver:version info\r\n");
    Printf("  log:show [DBG] status print state\r\n");
    Printf("  log on/off:enable/disable [DBG] status print\r\n");
    Printf("  lora:show [LORA] diag print state\r\n");
    Printf("  lora on/off:enable/disable [LORA] diag print\r\n");
    
    Printf("  rtk stat:print RTK state machine and UART4/UM980 diag counters\r\n");
    Printf("  dump once:print next UART4 raw input once\r\n");
    Printf("  dump on/off:continuous UART4 raw input print, use with caution\r\n");
    Printf("  rtcm stat:show RTCM sample counters and last sample length\r\n");
    Printf("  rtcm sample:print first 16 bytes of last RTCM sample\r\n");
    Printf("  rtcm cap on/off/clear:enable/disable/clear RTCM sample cache\r\n");
    Printf("  lora:show LoRa TX diag counters\r\n");
    
    Printf("  gga:show base station GGA position\r\n");
    Printf("  gsv:show visible satellites\r\n");
    Printf("  rtcm on/off:rtcm out or not\r\n");
    Printf("  bi:show baseinfoa\r\n");
    Printf("\r\n************************************************\r\n");
}

static void print_rtcm_sample(void)
{
    uint16_t j;
    uint16_t dump_len;
    static uint8_t sample_head[RTCM_SAMPLE_PRINT_BYTES];
    static char line[64];

    dump_len = dbg_rtcm_sample_dump_len;
    if (dump_len > RTCM_SAMPLE_PRINT_BYTES)
    {
        dump_len = RTCM_SAMPLE_PRINT_BYTES;
    }
    memcpy(sample_head, (void *)dbg_rtcm_sample_buf, dump_len);

    Printf("\r\nRTCM sample cap=%d cnt=%d bytes=%d rx_len=%d frame_len=%d dump=%d/%d\r\n",
           dbg_rtcm_sample_enable,
           dbg_rtcm_sample_cnt,
           dbg_rtcm_sample_bytes,
           dbg_rtcm_sample_rx_len,
           dbg_rtcm_sample_frame_len,
           dbg_rtcm_sample_dump_len,
           RTK_RTCM_SAMPLE_MAX_BYTES);

    if (dbg_rtcm_sample_dump_len > RTCM_SAMPLE_PRINT_BYTES)
    {
        Printf("Only first %d bytes printed; check if starts with D3\r\n",
               RTCM_SAMPLE_PRINT_BYTES);
    }

    if (dump_len == 0)
    {
        Printf("No RTCM sample; wait for running state, or use rtcm stat to check cnt\r\n");
        return;
    }

    uint16_t i = (uint16_t)snprintf(line, sizeof(line), "0000:");
    for (j = 0; j < dump_len && i < sizeof(line) - 5U; j++)
    {
        i += (uint16_t)snprintf(&line[i], sizeof(line) - i, " %02X", sample_head[j]);
    }
    snprintf(&line[i], sizeof(line) - i, "\r\n");
    Printf("%s", line);
}

static void ParseCommands()
{
    if (strstr((char const *)g_scanf_buffer,"?") || strstr((char const *)g_scanf_buffer,"help"))
    {
        command_list();
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"ver") || strstr((char const *)g_scanf_buffer,"VER"))
    {
        Printf("\r\nHW: %s(%s)\r\n", Hardware_Ver, Hrelease_time);
        Printf("SW: %s(%s)\r\n", Software_Ver, Srelease_time);
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"log on"))
    {
        dbg_status_report_enable = 1;
        Printf("\r\n[DBG] status print ON\r\n");
        return;
    }
    // rtcm off
    if (strstr((char const *)g_scanf_buffer,"rtcm off"))
    {
        if (um980_rtcmlog_disable() == HAL_OK)
            Printf("\r\nRTCM log disabled\r\n");
        else
            Printf("\r\nRTCM disable failed\r\n");
        return;
    }
    // rtcm on
    if (strstr((char const *)g_scanf_buffer,"rtcm on"))
    {
        if (um980_rtcmlog_enable() == HAL_OK)
            Printf("\r\nRTCM log enabled\r\n");
        else
            Printf("\r\nRTCM enable failed\r\n");
        return;
    }
        if (strstr((char const *)g_scanf_buffer,"log off"))
    {
        dbg_status_report_enable = 0;
        Printf("\r\n[DBG] status print OFF\r\n");
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"log") &&
        !strstr((char const *)g_scanf_buffer,"log on") &&
        !strstr((char const *)g_scanf_buffer,"log off"))
    {
        Printf("\r\n[DBG] status print: %s\r\n", dbg_status_report_enable ? "ON" : "OFF");
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"lora on"))
    {
        dbg_lora_print_enable = 1;
        Printf("\r\n[LORA] diag print ON\r\n");
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"lora off"))
    {
        dbg_lora_print_enable = 0;
        Printf("\r\n[LORA] diag print OFF\r\n");
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"lora") &&
        !strstr((char const *)g_scanf_buffer,"lora on") &&
        !strstr((char const *)g_scanf_buffer,"lora off"))
    {
        Printf("\r\n[LORA] diag print: %s\r\n", dbg_lora_print_enable ? "ON" : "OFF");
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"rtk stat"))
    {
        RTK_PrintDiag();
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"dump once"))
    {
        dbg_uart4_dump_once = 1;
        Printf("\r\nNext UART4 raw input will be printed once\r\n");
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"bi"))
    {
        if (um980_query_bi() != HAL_OK)
        {
            Printf("BASEINFOA query timeout\r\n");
        }
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"dump on"))
    {
        dbg_uart4_dump_enable = 1;
        Printf("\r\nUART4 raw input continuous print enabled; data量大, use dump off to stop\r\n");
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"dump off"))
    {
        dbg_uart4_dump_enable = 0;
        dbg_uart4_dump_once = 0;
        Printf("\r\nUART4 raw input print disabled; use log off to disable [DBG] too\r\n");
        return;
    }
    /*if (strstr((char const *)g_scanf_buffer,"ant") || strstr((char const *)g_scanf_buffer,"ANT"))
    {
        UM980_AntennaStatusTypeDef ant;
        if (um980_query_antenna_status(&ant) == HAL_OK)
        {
            Printf("ANT1=%s ANT2=%s\r\n", ant.ant1, ant.ant2);
        }
        else
        {
            Printf("Antenna query timeout\r\n");
        }
        return;
    }*/
    if (strstr((char const *)g_scanf_buffer,"gga"))
    {
        UM980_GGA_DataTypeDef gga;
        if (um980_query_gga(&gga) == HAL_OK)
        {
            Printf("fix=%d sat=%d hdop=%.1f lat=%.8f%c lon=%.8f%c alt=%.1f undulation=%.1f\r\n",
                   gga.fix_type, gga.sat_num, gga.hdop,
                   gga.latitude, gga.lat_dir,
                   gga.longitude, gga.lon_dir,
                   gga.altitude, gga.undulation);
        }
        else
        {
            Printf("GGA query timeout\r\n");
        }
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"gsv"))
    {
        if (um980_query_gsv() != HAL_OK)
        {
            Printf("GSV query timeout\r\n");
        }
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"lora"))
    {
        Printf("\r\nLORA read=%d/%d pkt=%d/%d init=%d",
               dbg_lora_rtcm_read_cnt, dbg_lora_rtcm_read_bytes,
               dbg_lora_pkt_sent_cnt, dbg_lora_pkt_sent_bytes,
               dbg_lora_init);
        Printf("\r\nLORA rx=%d/%d parsed=%d",
               dbg_lora_uart1_rx_cnt, dbg_lora_uart1_rx_bytes,
               dbg_lora_parse_ready_cnt);
        Printf("\r\nL33 txcplt=%d\r\n",
               dbg_uart1_txcplt_cnt);
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"rtcm stat"))
    {
        Printf("\r\nRTCM cap=%d cnt=%d bytes=%d rx_len=%d frame_len=%d dump=%d/%d\r\n",
               dbg_rtcm_sample_enable,
               dbg_rtcm_sample_cnt,
               dbg_rtcm_sample_bytes,
               dbg_rtcm_sample_rx_len,
               dbg_rtcm_sample_frame_len,
               dbg_rtcm_sample_dump_len,
               RTK_RTCM_SAMPLE_MAX_BYTES);
        Printf("cap=sample enable cnt=sample count rx_len=last RTCM length frame_len=first frame estimated length\r\n");
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"rtcm sample"))
    {
        print_rtcm_sample();
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"rtcm cap on"))
    {
        dbg_rtcm_sample_enable = 1;
        Printf("\r\nRTCM capture enabled\r\n");
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"rtcm cap off"))
    {
        dbg_rtcm_sample_enable = 0;
        Printf("\r\nRTCM capture disabled\r\n");
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"rtcm cap clear"))
    {
        uint8_t cap_enable = dbg_rtcm_sample_enable;
        dbg_rtcm_sample_enable = 0;
        osDelay(2);
        RTK_ResetRtcmSample();
        dbg_rtcm_sample_enable = cap_enable;
        Printf("\r\nRTCM capture cleared\r\n");
        return;
    }
    if (strstr((char const *)g_scanf_buffer,"reset") || strstr((char const *)g_scanf_buffer,"RESET"))
    {
        HAL_NVIC_SystemReset();
        return;
    }

    return;
}

void console_task(void const * argument)
{
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(&huart5);
    while (osSemaphoreAcquire(UART5_rx_semaphoreHandle, 0) == osOK) {}
    memset(g_scanf_buffer, 0, sizeof(g_scanf_buffer));
    g_uart5_rx_len = 0;
    dbg_uart5_start_ret = HAL_UARTEx_ReceiveToIdle_IT(&huart5, g_scanf_buffer, sizeof(g_scanf_buffer));

    welcome();

    for (;;)
    {
        dbg_console_loop_cnt++;
        if (osSemaphoreAcquire(UART5_rx_semaphoreHandle, 100) == osOK)
        {
            dbg_uart5_wakeup_cnt++;
            dbg_uart5_last_len = g_uart5_rx_len;
            memcpy((void *)dbg_uart5_last_buf, g_scanf_buffer, sizeof(dbg_uart5_last_buf));
            if (g_uart5_rx_len < sizeof(g_scanf_buffer))
            {
                g_scanf_buffer[g_uart5_rx_len] = '\0';
            }
            else
            {
                g_scanf_buffer[sizeof(g_scanf_buffer) - 1U] = '\0';
            }
            ParseCommands();
            reset_buffer_restart_DMA();
        }

        osDelay(1);
    }
}
