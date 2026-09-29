#ifndef _UM980_DRIVER_H
#define _UM980_DRIVER_H

#include "stm32f1xx_hal.h"

#define UM980_RX_BUF_LEN 1024
#define UM980_UNICORE_BUF_LEN 512
#define UM980_CMD_BUF_LEN 128
#define UM980_CMD_TIMEOUT 1000

#define NO_CRC_MODE 0
#define CRC_MODE 1

typedef enum
{
    UM980_MODE_ROVER = 0,
    UM980_MODE_BASE,
    UM980_MODE_HEADING2_FIX,
    UM980_MODE_HEADING2_VAR,
    UM980_MODE_HEADING2_LOW
} UM980_WorkModeTypeDef;

typedef enum
{
    UM980_SBAS_WAAS = 0,
    UM980_SBAS_EGNOS,
    UM980_SBAS_MSAS,
    UM980_SBAS_GAGAN,
    UM980_SBAS_SDCM,
    UM980_SBAS_BDS,
    UM980_SBAS_MAX
} UM980_SBAS_TypeDef;

typedef enum {
    UM980_PPP_DISABLE = 0,
    UM980_PPP_B2B,
    UM980_PPP_QX,
    UM980_PPP_SSR
} UM980_PPP_TypeDef;

typedef struct
{
    double lat_raw;
    double lon_raw;
    double latitude;
    double longitude;
    double altitude;
    double undulation;
    char lat_dir;
    char lon_dir;
    uint8_t fix_type;
    uint8_t sat_num;
    float hdop;
} UM980_GGA_DataTypeDef;

typedef struct
{
    uint16_t week;
    float tow;
    double latitude;
    double longitude;
    double altitude;
    float horiz_speed;
    float vert_speed;
    uint8_t fix_type;
} UM980_BestnavTypeDef;

/*typedef struct
{
    char ant1[10];
    char ant2[10];
} UM980_AntennaStatusTypeDef;
*/
typedef struct
{
    uint8_t  valid;    /* 1 = Status字段为0且ECEF非零，基准点已建立 */
    uint32_t status;   /* BASEINFO Status字段: 0=valid, 1=invalid */
    double   x;        /* ECEF X */
    double   y;        /* ECEF Y */
    double   z;        /* ECEF Z */
} UM980_BaseInfoTypeDef;

typedef struct
{
    UM980_WorkModeTypeDef work_mode;
    UM980_SBAS_TypeDef sbas_mode;
    UM980_PPP_TypeDef ppp_mode;
} UM980_HandleTypeDef;

extern UM980_HandleTypeDef hum980;
extern uint8_t um980_rx_buf[];
extern uint8_t um980_unicore_buf[];
volatile extern uint32_t um980_unicore_len;
volatile extern uint32_t um980_rx_len;

extern volatile uint32_t dbg_um980_last_tx_ret;
extern volatile uint32_t dbg_um980_init_step;
extern volatile uint32_t dbg_um980_init_ret;
extern volatile uint32_t dbg_um980_sbas_ret;
extern volatile uint32_t dbg_um980_sbas_save_ret;
extern volatile uint32_t dbg_um980_base_cmd_ret;
extern volatile uint32_t dbg_um980_base_save_ret;
extern volatile uint32_t dbg_um980_tx_take_timeout;
extern volatile uint32_t dbg_um980_tx_busy;
extern volatile uint32_t dbg_um980_pvtalg_ret;
extern volatile uint32_t dbg_um980_ack_ok_cnt;
extern volatile uint32_t dbg_um980_ack_err_cnt;
extern volatile uint32_t dbg_um980_ack_timeout_cnt;
extern volatile uint32_t dbg_um980_ppp_ret;
extern volatile uint32_t dbg_uart4_err_pe;
extern volatile uint32_t dbg_uart4_err_fe;
extern volatile uint32_t dbg_uart4_err_ne;
extern volatile uint32_t dbg_uart4_err_ore;

HAL_StatusTypeDef um980_init(UM980_WorkModeTypeDef UM980_MODE);
HAL_StatusTypeDef um980_send_cmd(uint8_t *cmd, uint8_t use_crc);
HAL_StatusTypeDef um980_send_cmd_trace(uint8_t *cmd, uint16_t wait_ms);
HAL_StatusTypeDef um980_set_rover_auto(uint16_t time, uint8_t distance);

HAL_StatusTypeDef um980_parse_gga_v411(uint8_t *gga_buf, UM980_GGA_DataTypeDef *gga_data);
//HAL_StatusTypeDef um980_parse_bestnav(uint8_t *data, uint16_t len, UM980_BestnavTypeDef *bestnav);
HAL_StatusTypeDef um980_query_gga(UM980_GGA_DataTypeDef *gga_data);
HAL_StatusTypeDef um980_query_bi(void);
HAL_StatusTypeDef um980_query_baseinfo(UM980_BaseInfoTypeDef *info);
HAL_StatusTypeDef um980_query_gsv(void);
HAL_StatusTypeDef um980_query_status(void);
void um980_data_parse(uint8_t *data, uint16_t len);
HAL_StatusTypeDef um980_rtcmlog_enable(void);
HAL_StatusTypeDef um980_rtcmlog_disable(void);
uint32_t um980_crc32_calc(uint8_t *data, uint16_t len);

#endif
