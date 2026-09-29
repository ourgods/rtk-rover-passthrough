#include "main.h"
#include "cmsis_os.h"
#include "um980_driver.h"
#include "rtk_task.h"
#include "console.h"
#include <string.h>
#include <stdio.h>

UM980_HandleTypeDef hum980;
uint8_t um980_rx_buf[UM980_RX_BUF_LEN];
uint8_t um980_unicore_buf[UM980_UNICORE_BUF_LEN];
volatile uint32_t um980_unicore_len = 0;
volatile uint32_t um980_rx_len = 0;

volatile uint32_t dbg_um980_last_tx_ret = 0;
volatile uint32_t dbg_um980_init_step = 0;
volatile uint32_t dbg_um980_init_ret = 0;
volatile uint32_t dbg_um980_sbas_ret = 0;
volatile uint32_t dbg_um980_sbas_save_ret = 0;
volatile uint32_t dbg_um980_base_cmd_ret = 0;
volatile uint32_t dbg_um980_base_save_ret = 0;
volatile uint32_t dbg_um980_tx_take_timeout = 0;
volatile uint32_t dbg_um980_tx_busy = 0;
volatile uint32_t dbg_um980_pvtalg_ret = 0;
volatile uint32_t dbg_um980_ack_ok_cnt = 0;
volatile uint32_t dbg_um980_ack_err_cnt = 0;
volatile uint32_t dbg_um980_ack_timeout_cnt = 0;
volatile uint32_t dbg_um980_ppp_ret = 0;

volatile uint32_t dbg_uart4_err_pe = 0;
volatile uint32_t dbg_uart4_err_fe = 0;
volatile uint32_t dbg_uart4_err_ne = 0;
volatile uint32_t dbg_uart4_err_ore = 0;

static void um980_restart_rx_dma(void);
static uint8_t um980_print_response(const char *cmd, uint16_t len);
static char *um980_find_gngga(uint8_t *buf, uint16_t len);

HAL_StatusTypeDef um980_send_cmd(uint8_t *cmd, uint8_t use_crc)
{
    static uint8_t send_buf[UM980_CMD_BUF_LEN];
    uint16_t cmd_len = strlen((char*)cmd);
    memset(send_buf, 0, sizeof(send_buf));
    if (use_crc != NO_CRC_MODE)
    {
        dbg_um980_last_tx_ret = HAL_ERROR;
        return HAL_ERROR;
    }
    if (cmd_len >= UM980_CMD_BUF_LEN - 2)
    {
        dbg_um980_last_tx_ret = HAL_ERROR;
        return HAL_ERROR;
    }

    memcpy(send_buf, cmd, cmd_len);
    send_buf[cmd_len++] = '\r';
    send_buf[cmd_len++] = '\n';
    
    if (osSemaphoreAcquire(UART4_tx_semaphoreHandle, 500) != osOK)
    {
        dbg_um980_tx_take_timeout++;
        dbg_um980_last_tx_ret = HAL_TIMEOUT;
        return HAL_TIMEOUT;
    }

    dbg_um980_last_tx_ret = HAL_UART_Transmit_DMA(&huart4, send_buf, cmd_len);
    if (dbg_um980_last_tx_ret != HAL_OK)
    {
        if (dbg_um980_last_tx_ret == HAL_BUSY)
        {
            dbg_um980_tx_busy++;
        }
        osSemaphoreRelease(UART4_tx_semaphoreHandle);
        return (HAL_StatusTypeDef)dbg_um980_last_tx_ret;
    }
    osSemaphoreRelease(UART4_tx_semaphoreHandle);
    return (HAL_StatusTypeDef)dbg_um980_last_tx_ret;
}

HAL_StatusTypeDef um980_send_cmd_trace(uint8_t *cmd, uint16_t wait_ms)
{
    HAL_StatusTypeDef tx_ret;
    uint8_t retry;
    uint16_t len;
    uint8_t matched;

    while (osSemaphoreAcquire(UART4_rx_semaphoreHandle, 0) == osOK) {}
    memset(um980_rx_buf, 0, UM980_RX_BUF_LEN);
    um980_restart_rx_dma();

    UM980_printf("[UM980 CMD] %s\r\n", cmd);
    tx_ret = um980_send_cmd(cmd, NO_CRC_MODE);
    if (tx_ret != HAL_OK)
    {
        UM980_printf("[UM980 TX] ret=%d\r\n", tx_ret);
        return tx_ret;
    }

    matched = 0;
    for (retry = 0; retry < 5; retry++)
    {
        if (osSemaphoreAcquire(UART4_rx_semaphoreHandle, wait_ms) != osOK)
        {
            break;
        }

        len = (um980_rx_len <= UM980_RX_BUF_LEN) ? um980_rx_len : UM980_RX_BUF_LEN;
        matched = um980_print_response((const char *)cmd, len);

        memset(um980_rx_buf, 0, UM980_RX_BUF_LEN);
        um980_restart_rx_dma();

        if (matched != 0)
        {
            break;
        }

        UM980_printf("[UM980 RSP] %s => non-cmd data, retry\r\n", cmd);
    }
    
    if (matched == 0 && retry >= 5)
    {
        dbg_um980_ack_timeout_cnt++;
        UM980_printf("[UM980 RSP] %s => NO MATCH after %d retries\r\n", cmd, retry);
    }

    return tx_ret;
}

static void um980_restart_rx_dma(void)
{
    HAL_UART_AbortReceive(&huart4);
    __HAL_UART_CLEAR_IDLEFLAG(&huart4);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, um980_rx_buf, UM980_RX_BUF_LEN);
    __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
}

static uint8_t um980_print_response(const char *cmd, uint16_t len)
{
    char line[80];
    char expect[96];
    uint16_t i;
    uint16_t pos = 0;
    uint8_t matched = 0;

    if (len == 0)
    {
        UM980_printf("[UM980 RSP] %s => EMPTY\r\n", cmd);
        return 0;
    }

    if (len < UM980_RX_BUF_LEN)
    {
        um980_rx_buf[len] = '\0';
    }
    else
    {
        um980_rx_buf[UM980_RX_BUF_LEN - 1] = '\0';
    }

    snprintf(expect, sizeof(expect), "$command,%s,response:", cmd);
    matched = (strstr((char *)um980_rx_buf, expect) != NULL) ? 1 : 0;

    if (matched != 0 && strstr((char *)um980_rx_buf, "OK") != NULL)
    {
        dbg_um980_ack_ok_cnt++;
    }
    if (matched != 0 &&
        (strstr((char *)um980_rx_buf, "ERROR") != NULL || strstr((char *)um980_rx_buf, "FAIL") != NULL))
    {
        dbg_um980_ack_err_cnt++;
    }

    UM980_printf("[UM980 RSP] %s len=%d match=%d\r\n", cmd, len, matched);

    for (i = 0; i < len; i++)
    {
        uint8_t ch = um980_rx_buf[i];
        line[pos++] = (ch >= 32 && ch <= 126) ? (char)ch : '.';
        if (pos >= (sizeof(line) - 1))
        {
            line[pos] = '\0';
            UM980_printf("%s\r\n", line);
            pos = 0;
        }
    }
    if (pos > 0)
    {
        line[pos] = '\0';
        UM980_printf("%s\r\n", line);
    }

    return matched;
}

HAL_StatusTypeDef um980_init(UM980_WorkModeTypeDef UM980_MODE)
{
    hum980.work_mode = UM980_MODE;
    hum980.sbas_mode = UM980_SBAS_WAAS;
    hum980.ppp_mode = UM980_PPP_DISABLE;

    dbg_um980_init_step = 1;
    dbg_um980_init_ret = HAL_OK;
    HAL_Delay(500);

    dbg_um980_init_step = 2;
    dbg_um980_pvtalg_ret = um980_send_cmd_trace((uint8_t*)"config pvtalg multi", 500);
    dbg_um980_init_ret = dbg_um980_pvtalg_ret;
    if (dbg_um980_pvtalg_ret != HAL_OK) return HAL_ERROR;
    HAL_Delay(200);

    dbg_um980_init_step = 3;
    dbg_um980_ppp_ret = um980_send_cmd_trace((uint8_t*)"config ppp disable", 500);
    if (dbg_um980_ppp_ret != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef um980_set_rover_auto(uint16_t time, uint8_t distance)
{
    uint8_t cmd[64];
    memset(cmd, 0, sizeof(cmd));
    
    sprintf((char*)cmd, "mode rover survey");
    dbg_um980_base_cmd_ret = um980_send_cmd_trace(cmd, 500);
    if (dbg_um980_base_cmd_ret != HAL_OK) return HAL_ERROR;
    HAL_Delay(500);
    dbg_um980_base_save_ret = um980_send_cmd_trace((uint8_t*)"saveconfig", 800);
    return (HAL_StatusTypeDef)dbg_um980_base_save_ret;
}

static char *um980_find_gngga(uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (buf == NULL || len < 6U)
    {
        return NULL;
    }

    for (i = 0; i <= (uint16_t)(len - 6U); i++)
    {
        if (buf[i] == '$' && buf[i+2] == 'G' && buf[i+3] == 'G' && buf[i+4] == 'A' && buf[i+5] == ',')
        {
            return (char *)&buf[i];
        }
    }

    return NULL;
}

HAL_StatusTypeDef um980_parse_gga_v411(uint8_t *gga_buf, UM980_GGA_DataTypeDef *gga_data)
{
    char *p = strstr((char*)gga_buf, "$GNGGA");
    if (p == NULL) p = strstr((char*)gga_buf, "$GPGGA");
    if (p == NULL) p = strstr((char*)gga_buf, "$GBGGA");
    if (p == NULL) return HAL_ERROR;

    memset(gga_data, 0, sizeof(UM980_GGA_DataTypeDef));

    int fix = 0, sat = 0;
    double lat_raw = 0, lon_raw = 0, alt = 0, undulation = 0;
    float hdop = 0;
    char lat_dir = 'N', lon_dir = 'E';

    int parsed = sscanf(p, "$G%*cGGA,%*f,%lf,%c,%lf,%c,%d,%d,%f,%lf,%*c,%lf",
                        &lat_raw, &lat_dir, &lon_raw, &lon_dir, &fix, &sat, &hdop, &alt, &undulation);

    if (parsed < 5) return HAL_ERROR;

    gga_data->lat_raw = lat_raw;
    gga_data->lon_raw = lon_raw;
    gga_data->lat_dir = lat_dir;
    gga_data->lon_dir = lon_dir;
    gga_data->fix_type = (uint8_t)fix;
    gga_data->sat_num = (uint8_t)sat;
    gga_data->hdop = hdop;
    gga_data->altitude = alt;
    gga_data->undulation = undulation;

    int lat_deg = (int)(lat_raw / 100);
    gga_data->latitude = lat_deg + (lat_raw - lat_deg * 100) / 60.0;
    int lon_deg = (int)(lon_raw / 100);
    gga_data->longitude = lon_deg + (lon_raw - lon_deg * 100) / 60.0;

    return HAL_OK;
}
