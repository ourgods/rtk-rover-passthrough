#ifndef _RTK_H
#define _RTK_H

#include "um980_driver.h"
#include "cmsis_os.h"

/* ========== 基站配置参数 ========== */
#define BASE_AUTO_TIME     200    /* 基准站自主优化时长（秒，下发给mode base time的T） */
#define BASE_DISTANCE      0     /* 断电复用距离（米） */
#define SBAS_TYPE          UM980_SBAS_BDS  /* 国内场景优先BDS SBAS */

/* ========== survey-in 收敛轮询（轮询BASEINFO的Status，有效才进RUNNING） ========== */
#define BASE_SURVEY_POLL_START     20   /* 秒，最早开始轮询BASEINFO的时间 */
#define BASE_SURVEY_POLL_INTERVAL  10   /* 秒，轮询间隔 */
#define BASE_SURVEY_MAX_WAIT       60  /* 秒，最长等待，超时则告警并降级继续 */

/* ========== 周期状态打印间隔 ========== */
#define STATUS_REPORT_INTERVAL  100

/* ========== RTCM 相关常量 ========== */
#define RTCM_SYNC_BYTE     0xD3  /* RTCM3帧同步字节 */
#define RTCM_CACHE_SIZE    1024  /* RTCM共享缓冲区最大长度（字节） */

/* LoRa单包最大载荷（L33定点模式，去掉3字节帧头后的有效数据） */
#define LORA_RTCM_CHUNK    50    /* RTCM分片发送时每片最大长度 */

/* ========== RTCM样本调试缓存 ========== */
#define RTK_RTCM_SAMPLE_MAX_BYTES 1024

/* ========== RTCM 共享变量（RTK_task写，LORA_task读） ========== */
extern uint8_t g_rtcm_buf[RTCM_CACHE_SIZE];
extern volatile uint16_t g_rtcm_len;
extern volatile uint32_t g_rtcm_seq;
extern osMutexId g_rtcm_mutex;

/* ========== dump once 控制（console写，RTK_task读） ========== */
extern volatile uint8_t g_uart4_dump_once;

/* ========== 基站运行状态查询 ========== */
typedef enum {
    ROVER_STATUS_IDLE = 0,
    ROVER_STATUS_INIT,
    ROVER_STATUS_ROVER_CONFIG,
    ROVER_STATUS_ROVER_WAIT,
    ROVER_STATUS_RTCM_CONFIG,
    ROVER_STATUS_RUNNING,
    ROVER_STATUS_ERROR
} RoverStatusEnum;

extern volatile RoverStatusEnum g_rover_status;

/* ========== RTK调试计数器 ========== */
extern volatile uint32_t dbg_rtk_loop_cnt;
extern volatile uint32_t dbg_rtcm_write_cnt;
extern volatile uint32_t dbg_rtcm_write_bytes;
extern volatile uint32_t dbg_um980_rx_rtcm_cnt;
extern volatile uint32_t dbg_um980_rx_ascii_cnt;
extern volatile uint32_t dbg_um980_rx_other_cnt;
extern volatile uint32_t dbg_base_wait_elapsed;
extern volatile uint32_t dbg_base_status_field;
extern volatile uint32_t dbg_uart4_idle_cnt;
extern volatile uint32_t dbg_uart4_rx_len_last;
extern volatile uint8_t dbg_uart4_dump_enable;
extern volatile uint8_t dbg_uart4_dump_once;
extern volatile uint8_t dbg_pause_uart4_rx;
extern volatile uint8_t dbg_status_report_enable;
extern volatile uint8_t dbg_rtcm_sample_enable;
extern volatile uint32_t dbg_rtcm_sample_cnt;
extern volatile uint32_t dbg_rtcm_sample_bytes;
extern volatile uint16_t dbg_rtcm_sample_rx_len;
extern volatile uint16_t dbg_rtcm_sample_dump_len;
extern volatile uint16_t dbg_rtcm_sample_frame_len;
extern volatile uint8_t dbg_rtcm_sample_buf[];

void RTK_ResetRtcmSample(void);
void RTK_PrintDiag(void);

#endif
