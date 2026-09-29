/**
 * @file passthrough.c
 * @brief UART5(PC) <-> UART4(UM980) 透传 + RTCM捕获
 *
 * 核心原理：
 * - UART4使用Normal DMA和A/B双缓冲接收
 * - A、B两片各1024字节，总缓冲区2048字节
 * - 当前片接收完成后，DMA立即切换到另一片
 * - 完成的分片由任务通过UART5发送
 * - 同一片不会同时被UART4写入和UART5读取
 */

#include "main.h"
#include "cmsis_os.h"
#include "passthrough.h"
#include "rtk_task.h"
#include <string.h>

/* RTCM同步字节 */
#define RTCM_SYNC_BYTE 0xD3

/* 透传接收长度 - 全局变量 */
volatile uint16_t g_passthrough_uart4_rx_len = 0;
volatile uint16_t g_passthrough_uart5_rx_len = 0;
volatile uint16_t g_passthrough_uart1_rx_len = 0;

/* UART4 (UM980) 接收缓冲区 - 全局 */
uint8_t uart4_rx_buf[PASSTHROUGH_UART4_BUF_SIZE];
/* UART5 (PC) 接收缓冲区 - 全局 */
uint8_t uart5_rx_buf[PASSTHROUGH_UART5_BUF_SIZE];
/* UART1 (Lora) 接收缓冲区 - 全局 */
uint8_t uart1_rx_buf[PASSTHROUGH_UART1_BUF_SIZE];

/* 外部声明 */
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

/* 信号量声明 - 来自main.c */
extern osSemaphoreId_t UART4_tx_semaphoreHandle;
extern osSemaphoreId_t UART4_rx_semaphoreHandle;
extern osSemaphoreId_t UART5_tx_semaphoreHandle;
extern osSemaphoreId_t UART5_rx_semaphoreHandle;

/* UART4 A/B分片状态 */
typedef enum
{
    UART4_SLICE_FREE = 0,
    UART4_SLICE_RECEIVING,
    UART4_SLICE_READY,
    UART4_SLICE_SENDING
} uart4_slice_state_t;

/* A片为0，B片为1 */
static volatile uart4_slice_state_t uart4_slice_state[2] =
{
    UART4_SLICE_FREE,
    UART4_SLICE_FREE
};

static volatile uint16_t uart4_slice_len[2] = {0U, 0U};

/* 当前DMA正在接收哪一片 */
static volatile uint8_t uart4_rx_index = 0U;

/* 下一片应该发送哪一片 */
static volatile uint8_t uart4_tx_index = 0U;

/* 两片都没有空闲空间，接收暂时停止 */
static volatile uint8_t uart4_rx_stalled = 0U;

/* UART4接收错误，等待任务恢复 */
static volatile uint8_t uart4_rx_error_pending = 0U;

/* 诊断变量：只记录，不参与A/B切换 */
volatile uint32_t dbg_uart4_rx_reached_512_count = 0U;
volatile uint32_t dbg_uart4_rx_full_1024_count = 0U;
volatile uint16_t dbg_uart4_rx_max_len = 0U;
volatile uint32_t dbg_uart4_pingpong_busy_count = 0U;
volatile uint32_t dbg_uart4_rx_restart_fail_count = 0U;
volatile uint32_t dbg_uart5_tx_error_count = 0U;


/* 获取A片或B片的起始地址 */
static uint8_t *uart4_get_slice(uint8_t index)
{
    return &uart4_rx_buf[
        (uint32_t)index * PASSTHROUGH_UART4_SLICE_SIZE
    ];
}

/* 启动指定分片接收 */
static HAL_StatusTypeDef uart4_start_rx_slice(uint8_t index)
{
    HAL_StatusTypeDef ret;

    if (index > 1U)
    {
        return HAL_ERROR;
    }

    /* 正在发送或等待发送的片绝不能拿来接收 */
    if (uart4_slice_state[index] != UART4_SLICE_FREE)
    {
        return HAL_BUSY;
    }

    uart4_rx_index = index;
    uart4_slice_len[index] = 0U;
    uart4_slice_state[index] = UART4_SLICE_RECEIVING;

    __HAL_UART_CLEAR_IDLEFLAG(&huart4);

    ret = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart4,
        uart4_get_slice(index),
        PASSTHROUGH_UART4_SLICE_SIZE
    );

    if (ret != HAL_OK)
    {
        uart4_slice_state[index] = UART4_SLICE_FREE;
        dbg_uart4_rx_restart_fail_count++;
        return ret;
    }

    /*
     * 每片为1024字节。
     * 不需要在512处触发HT回调。
     */
    __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);

    return HAL_OK;
}


/* UART4完成一次接收：立即切换到另一片 */
void passthrough_uart4_rx_event_callback(uint16_t size)
{
    uint8_t completed_index;
    uint8_t next_index;

    completed_index = uart4_rx_index;
    next_index = completed_index ^ 1U;

    if (uart4_slice_state[completed_index] !=
        UART4_SLICE_RECEIVING)
    {
        return;
    }

    if ((size == 0U) ||
        (size > PASSTHROUGH_UART4_SLICE_SIZE))
    {
        uart4_slice_len[completed_index] = 0U;
        uart4_slice_state[completed_index] =
            UART4_SLICE_FREE;

        if (uart4_start_rx_slice(completed_index) != HAL_OK)
        {
            uart4_rx_stalled = 1U;
        }

        return;
    }

    uart4_slice_len[completed_index] = size;
    g_passthrough_uart4_rx_len = size;

    /* 判断是否碰到过旧的512字节上限 */
    if (size >= 512U)
    {
        dbg_uart4_rx_reached_512_count++;
    }

    /* 判断新的1024字节分片是否也被填满 */
    if (size == PASSTHROUGH_UART4_SLICE_SIZE)
    {
        dbg_uart4_rx_full_1024_count++;
    }

    if (size > dbg_uart4_rx_max_len)
    {
        dbg_uart4_rx_max_len = size;
    }

    /* 当前片以后只能发送，DMA不能再写 */
    uart4_slice_state[completed_index] =
        UART4_SLICE_READY;

    /* 立即让DMA切换到另一片 */
    if (uart4_slice_state[next_index] ==
        UART4_SLICE_FREE)
    {
        if (uart4_start_rx_slice(next_index) == HAL_OK)
        {
            uart4_rx_stalled = 0U;
        }
        else
        {
            uart4_rx_stalled = 1U;
        }
    }
    else
    {
        /*
         * 另一片仍在发送或等待发送，不能覆盖，只暂停接收并记录。
         */
        dbg_uart4_pingpong_busy_count++;
        uart4_rx_stalled = 1U;
    }

    /* 通知任务发送刚完成的分片 */
    (void)osSemaphoreRelease(UART4_rx_semaphoreHandle);
}


/* UART4接收错误：不直接碰整个缓冲区 */
void passthrough_uart4_error_callback(void)
{
    uart4_rx_error_pending = 1U;
    (void)osSemaphoreRelease(UART4_rx_semaphoreHandle);
}

/**
 * @brief 启动UART4 DMA接收 (UM980 -> STM32)
 */
static void passthrough_uart4_start(void)
{
    while (osSemaphoreAcquire(
               UART4_rx_semaphoreHandle, 0) == osOK)
    {
    }

    uart4_slice_state[0] = UART4_SLICE_FREE;
    uart4_slice_state[1] = UART4_SLICE_FREE;

    uart4_slice_len[0] = 0U;
    uart4_slice_len[1] = 0U;

    uart4_rx_index = 0U;
    uart4_tx_index = 0U;
    uart4_rx_stalled = 0U;
    uart4_rx_error_pending = 0U;

    /* 开机先使用A片接收 */
    if (uart4_start_rx_slice(0U) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 启动UART5 DMA接收 (PC -> STM32)
 *
 * 注意：UART5在CubeMX里没有配置DMA，这里先用IT模式
 * TODO: 需要在CubeMX里添加UART5的DMA配置
 */
static void passthrough_uart5_start(void)
{
    /* 清除IDLE标志 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart5);
    /* 使能IDLE中断 */
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    /* 清除之前的信号量 */
    while (osSemaphoreAcquire(UART5_rx_semaphoreHandle, 0) == osOK) {}
    /* 启动IT中断接收（暂时用IT，等CubeMX配好DMA后改回DMA） */
    HAL_UARTEx_ReceiveToIdle_IT(&huart5, uart5_rx_buf, PASSTHROUGH_UART5_BUF_SIZE);
}

/**
 * @brief 通过UART1 dma接收数据 (Lora -> stm32)
 */
static void passthrough_uart1_rx(uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef ret;
    
    if (data == NULL || len == 0) return;
    
    /* 等待信号量 */
    if (osSemaphoreAcquire(UART1_tx_semaphoreHandle, 50) != osOK) return;
    
    /* DMA接收 */
    ret = HAL_UART_Receive_DMA(&huart1, data, len);
    
    if (ret != HAL_OK)
    {
        osSemaphoreRelease(UART1_rx_semaphoreHandle);
        return;
    }

    if (osSemaphoreAcquire(UART1_rx_semaphoreHandle, osWaitForever) == osOK)
    {
        osSemaphoreRelease(UART1_rx_semaphoreHandle);
    }
}

/**
 * @brief 通过UART4发送数据 (STM32 -> UM980)
 */
static void passthrough_uart4_tx(uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef ret;

    if (data == NULL || len == 0) return;

    /* 等待信号量 */
    if (osSemaphoreAcquire(UART4_tx_semaphoreHandle, 50) != osOK) return;

    /* DMA发送 */
    ret = HAL_UART_Transmit_DMA(&huart4, data, len);
    if (ret != HAL_OK)
    {
        osSemaphoreRelease(UART4_tx_semaphoreHandle);
        return;
    }

    if (osSemaphoreAcquire(UART4_tx_semaphoreHandle, osWaitForever) == osOK)
    {
        osSemaphoreRelease(UART4_tx_semaphoreHandle);
    }
}

/**
 * @brief 通过UART5发送数据 (STM32 -> PC)
 *
 */
static HAL_StatusTypeDef passthrough_uart5_tx(
    uint8_t *data,
    uint16_t len)
{
    HAL_StatusTypeDef ret;

    if ((data == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    /*
     * 1024字节在115200下约需89ms，
     * 超时设置为200ms。
     */
    ret = HAL_UART_Transmit(
        &huart5,
        data,
        len,
        200U
    );

    if (ret != HAL_OK)
    {
        dbg_uart5_tx_error_count++;
    }

    return ret;
}

/**
 * @brief 透传任务主体
 *
 * 数据流：
 * - Lora -> UART1 -> 转发 -> UART4 -> UM980
 * - PC -> UART5 -> 转发 -> UART4 -> UM980
 * -UM980 -> UART4 ->  转发 -> UART5 ->PC
 */
void passthrough_task(void *argument)
{
    /* 启动DMA接收 */
    passthrough_uart4_start();
    passthrough_uart5_start();

    for (;;)
    {
        /* ===== 处理UM980 -> PC 方向 ===== */
        if (osSemaphoreAcquire(
                UART4_rx_semaphoreHandle, 10) == osOK)
        {
            /* 先处理UART4接收错误 */
            if (uart4_rx_error_pending != 0U)
            {
                uint8_t index = uart4_rx_index;

                uart4_rx_error_pending = 0U;

                /*
                 * 出错的是当前正在接收的片。
                 * 丢弃其中不完整的数据。
                 */
                if (uart4_slice_state[index] ==
                    UART4_SLICE_RECEIVING)
                {
                    uart4_slice_len[index] = 0U;
                    uart4_slice_state[index] =
                        UART4_SLICE_FREE;
                }

                if (uart4_slice_state[index] ==
                    UART4_SLICE_FREE)
                {
                    if (uart4_start_rx_slice(index) != HAL_OK)
                    {
                        uart4_rx_stalled = 1U;
                    }
                }
            }

            /* 按A、B顺序发送READY分片 */
            for (;;)
            {
                uint8_t index = uart4_tx_index;
                uint16_t len;

                if (uart4_slice_state[index] !=
                    UART4_SLICE_READY)
                {
                    break;
                }

                /*
                 * 设置成SENDING以后，
                 * DMA绝不能使用这一片。
                 */
                uart4_slice_state[index] =
                    UART4_SLICE_SENDING;

                len = uart4_slice_len[index];

                (void)passthrough_uart5_tx(
                    uart4_get_slice(index),
                    len
                );

                uart4_slice_len[index] = 0U;
                uart4_slice_state[index] =
                    UART4_SLICE_FREE;

                g_passthrough_uart4_rx_len = 0U;

                /* 下次轮到另一片发送 */
                uart4_tx_index ^= 1U;

                /*
                 * 如果之前两片均被占用而暂停接收，
                 * 现在刚发送完的片已经FREE，可以接收。
                 */
                if (uart4_rx_stalled != 0U)
                {
                    uart4_rx_stalled = 0U;

                    if (uart4_start_rx_slice(index) != HAL_OK)
                    {
                        uart4_rx_stalled = 1U;
                    }
                }
            }
        }

        /* ===== 处理PC -> UM980 方向 ===== */
        if (osSemaphoreAcquire(UART5_rx_semaphoreHandle, 10) == osOK)
        {
            /* 从全局变量获取接收长度 */
            extern volatile uint16_t g_passthrough_uart5_rx_len;
            uint16_t len = g_passthrough_uart5_rx_len;

            if (len > 0 && len <= PASSTHROUGH_UART5_BUF_SIZE)
            {
                /* 转发到UM980 */
                passthrough_uart4_tx(uart5_rx_buf, len);
            }

            /* 清除标志并重新启动IT接收 */
            g_passthrough_uart5_rx_len = 0;
            memset(uart5_rx_buf, 0, PASSTHROUGH_UART5_BUF_SIZE);
            HAL_UARTEx_ReceiveToIdle_IT(&huart5, uart5_rx_buf, PASSTHROUGH_UART5_BUF_SIZE);
        }
        
    }
}
