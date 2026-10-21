/**
 * @file    bsp_sim800.c
 * @brief   SIM800L GSM 模块驱动实现
 * @note    关键实现:
 *          1. DMA 循环接收 + UART Idle 中断
 *          2. 超时机制防止死等
 *          3. 字符串匹配检测 AT 响应
 *          4. 短信发送状态机
 */

#include "bsp_sim800.h"
#include "usart.h"
#include "dma.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS 支持（如果需要延时） */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "health_watch.h"

/* ========================================================================= */
/* 外部变量声明                                                              */
/* ========================================================================= */

/* USART2 DMA 接收句柄（在 usart.c 中定义） */
extern DMA_HandleTypeDef hdma_usart2_rx;

/* ========================================================================= */
/* 私有变量                                                                  */
/* ========================================================================= */

/* DMA 循环接收缓冲区 */
static uint8_t gsm_dma_buffer[GSM_RX_BUFFER_SIZE];
static char gsm_rx_buffer[GSM_RX_BUFFER_SIZE]; /* 任务拥有的响应快照，DMA不修改 */
static volatile uint16_t gsm_dma_position;
static uint16_t gsm_consumed;
static uint16_t gsm_text_length;
static volatile uint32_t gsm_dma_wraps;
static uint32_t gsm_read_total;
static volatile uint8_t gsm_rx_error;
void gsm_receive_error(void) { gsm_rx_error = 1U; }

/* 接收完成标志（由 UART Idle 中断设置） */
static volatile uint8_t gsm_rx_complete = 0;

/* 最后接收到的数据长度 */
static volatile uint16_t gsm_rx_length = 0;

/* 模块是否已初始化 */
static uint8_t gsm_initialized = 0;

/* ========================================================================= */
/* 私有函数声明                                                              */
/* ========================================================================= */

static GSM_Status gsm_wait_response(const char *expected,
                                    uint32_t timeout_ms);
static uint8_t gsm_contains(const char *haystack, const char *needle);
static void gsm_delay_ms(uint32_t ms);

/* ========================================================================= */
/* UART Idle 中断回调                                                        */
/* ========================================================================= */

/**
 * @brief  UART 接收事件回调（Idle 中断触发）
 * @param  huart UART 句柄
 * @param  Size  本次接收的字节数
 * @note   这个函数由 HAL 库在检测到 Idle 事件时调用
 *         必须在 stm32f4xx_it.c 中启用 UART Idle 中断
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
        if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_TC) gsm_dma_wraps++;
        /* 记录接收长度 */
        gsm_rx_length = Size;
        gsm_dma_position = Size % GSM_RX_BUFFER_SIZE;

        /* 设置接收完成标志 */
        gsm_rx_complete = 1;

        /* 不在DMA正在写的缓冲区插入NUL，避免覆盖字节或读取碎片响应。 */
    }
}

/* ========================================================================= */
/* 公共函数实现                                                              */
/* ========================================================================= */

/**
 * @brief  启动 DMA 接收
 */
void gsm_start_receive(void)
{
    /* 清空缓冲区 */
    memset(gsm_rx_buffer, 0, GSM_RX_BUFFER_SIZE);
    gsm_rx_complete = 0;
    gsm_rx_length = 0;
    gsm_text_length = gsm_dma_position = gsm_consumed = 0U;
    gsm_read_total = gsm_dma_wraps = 0U;
    gsm_rx_error = 0U;

    /* 启动 UART DMA 接收，使用 Idle 中断模式 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, gsm_dma_buffer, GSM_RX_BUFFER_SIZE) != HAL_OK)
        gsm_rx_error = 1U;

    /* 禁用 DMA 半传输中断（只需要完整传输和 Idle） */
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

/**
 * @brief  清空接收缓冲区
 */
void gsm_clear_rx_buffer(void)
{
    /* 停止当前 DMA 接收 */
    HAL_UART_DMAStop(&huart2);

    /* 清空缓冲区和标志 */
    memset(gsm_rx_buffer, 0, GSM_RX_BUFFER_SIZE);
    gsm_rx_complete = 0;
    gsm_rx_length = 0;

    /* 重新启动接收 */
    gsm_start_receive();
}

/**
 * @brief  获取最后一次响应内容
 */
const char* gsm_get_last_response(void)
{
    return (const char*)gsm_rx_buffer;
}

/**
 * @brief  初始化 SIM800L 模块
 */
GSM_Status gsm_init(void)
{
    GSM_Status status;
    uint8_t rssi;
    gsm_initialized = 0U;
    (void)HAL_UART_DMAStop(&huart2);

    printf("[GSM] Initializing SIM800L...\r\n");

    /* 启动 DMA 接收 */
    gsm_start_receive();

    /* 延迟 2 秒等待模块上电稳定 */
    gsm_delay_ms(2000);

    /* 1. 测试 AT 连接 */
    printf("[GSM] Testing AT...\r\n");
    status = gsm_send_command("AT", "OK", 2000);
    if (status != GSM_OK)
    {
        printf("[GSM] AT test failed: %d\r\n", status);
        return GSM_NO_RESPONSE;
    }

    /* 2. 关闭回显 */
    printf("[GSM] Disabling echo...\r\n");
    status = gsm_send_command("ATE0", "OK", 2000);
    if (status != GSM_OK)
    {
        printf("[GSM] ATE0 failed: %d\r\n", status);
        return GSM_ERROR;
    }

    /* 3. 检查 SIM 卡 */
    printf("[GSM] Checking SIM card...\r\n");
    status = gsm_check_sim();
    if (status != GSM_OK)
    {
        printf("[GSM] SIM card error: %d\r\n", status);
        return GSM_SIM_ERROR;
    }

    /* 4. 检查信号质量 */
    printf("[GSM] Checking signal...\r\n");
    status = gsm_check_signal(&rssi);
    if (status != GSM_OK)
    {
        printf("[GSM] Signal check failed: %d\r\n", status);
        return GSM_ERROR;
    }

    printf("[GSM] Signal strength: %d\r\n", rssi);

    if (rssi < GSM_MIN_SIGNAL_QUALITY && rssi != 99)
    {
        printf("[GSM] Warning: weak signal\r\n");
        /* 不返回错误，继续初始化 */
    }

    /* 5. 设置短信文本模式 */
    printf("[GSM] Setting SMS text mode...\r\n");
    status = gsm_send_command("AT+CMGF=1", "OK", 2000);
    if (status != GSM_OK)
    {
        printf("[GSM] CMGF failed: %d\r\n", status);
        return GSM_ERROR;
    }

    /* 6. 查询网络注册状态 */
    printf("[GSM] Checking network registration...\r\n");
    status = gsm_send_command("AT+CREG?", "OK", 2000);
    {
        unsigned mode, registration;
        const char *registration_line = strstr(gsm_rx_buffer, "+CREG:");
        if (status != GSM_OK || !registration_line ||
            sscanf(registration_line, "+CREG: %u,%u", &mode, &registration) != 2 ||
            (registration != 1U && registration != 5U)) return GSM_NO_NETWORK;
    }

    gsm_initialized = 1;
    printf("[GSM] Initialization complete\r\n");

    return GSM_OK;
}

/**
 * @brief  发送 AT 命令并等待响应
 */
GSM_Status gsm_send_command(const char *command,
                            const char *expected_response,
                            uint32_t timeout_ms)
{
    char cmd_buffer[128];
    HAL_StatusTypeDef hal_status;
    GSM_Status status;

    if (command == NULL || strlen(command) + 3U > sizeof(cmd_buffer))
    {
        return GSM_ERROR;
    }

    /* 清空接收缓冲区 */
    gsm_clear_rx_buffer();

    /* 构造完整的 AT 命令（添加 \r\n） */
    snprintf(cmd_buffer, sizeof(cmd_buffer), "%s\r\n", command);

    /* 发送命令 */
    hal_status = HAL_UART_Transmit(&huart2,
                                   (uint8_t*)cmd_buffer,
                                   strlen(cmd_buffer),
                                   1000);

    if (hal_status != HAL_OK)
    {
        printf("[GSM] TX failed: %d\r\n", hal_status);
        return GSM_ERROR;
    }

    /* 等待响应 */
    if (expected_response != NULL)
    {
        status = gsm_wait_response(expected_response, timeout_ms);
    }
    else
    {
        status = gsm_wait_response("OK", timeout_ms);
    }

    return status;
}

/**
 * @brief  检查 SIM 卡状态
 */
GSM_Status gsm_check_sim(void)
{
    GSM_Status status;

    status = gsm_send_command("AT+CPIN?", "READY", 3000);

    if (status != GSM_OK)
    {
        return GSM_SIM_ERROR;
    }

    return GSM_OK;
}

/**
 * @brief  检查信号质量
 */
GSM_Status gsm_check_signal(uint8_t *rssi)
{
    GSM_Status status;
    const char *response;

    if (rssi == NULL)
    {
        return GSM_ERROR;
    }

    /* 发送 AT+CSQ */
    status = gsm_send_command("AT+CSQ", "+CSQ:", 3000);

    if (status != GSM_OK)
    {
        return GSM_ERROR;
    }

    /* 解析响应：+CSQ: <rssi>,<ber> */
    response = gsm_get_last_response();

    if (strstr(response, "+CSQ:") != NULL)
    {
        /* 简单解析：跳过 "+CSQ: " */
        const char *p = strstr(response, "+CSQ:");
        if (p != NULL)
        {
            p += 6;  /* 跳过 "+CSQ: " */
            *rssi = (uint8_t)atoi(p);
            return GSM_OK;
        }
    }

    *rssi = 99;  /* 未知 */
    return GSM_ERROR;
}

/**
 * @brief  软复位 GSM 模块
 */
GSM_Status gsm_reset(void)
{
    printf("[GSM] Resetting module...\r\n");

    /* 发送软复位命令 */
    GSM_Status status = gsm_send_command("AT+CFUN=1,1", "OK", 5000);

    if (status == GSM_OK)
    {
        /* 等待重启完成 */
        gsm_delay_ms(5000);
        gsm_initialized = 0;
    }

    return status;
}

/**
 * @brief  发送短信
 */
GSM_Status gsm_send_sms(const char *phone, const char *text)
{
    char cmd_buffer[32];
    HAL_StatusTypeDef hal_status;
    GSM_Status status;

    if (phone == NULL || text == NULL || strlen(phone) > 18U || strlen(text) > 160U)
    {
        return GSM_ERROR;
    }
    {
        const char *p = phone;
        if (*p == '+') p++;
        if (!*p) return GSM_ERROR;
        while (*p) { if (*p < '0' || *p > '9') return GSM_ERROR; p++; }
        p = text;
        while (*p) { if ((unsigned char)*p < 32U || (unsigned char)*p > 126U) return GSM_ERROR; p++; }
    }

    if (!gsm_initialized)
    {
        printf("[GSM] Not initialized\r\n");
        return GSM_ERROR;
    }

    printf("[GSM] Sending SMS to %s...\r\n", phone);

    /* ========== 阶段1: 发送 AT+CMGS 命令 ========== */

    gsm_clear_rx_buffer();

    /* 构造命令：AT+CMGS="13800138000" */
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+CMGS=\"%s\"\r\n", phone);

    hal_status = HAL_UART_Transmit(&huart2,
                                   (uint8_t*)cmd_buffer,
                                   strlen(cmd_buffer),
                                   1000);

    if (hal_status != HAL_OK)
    {
        printf("[GSM] CMGS TX failed\r\n");
        return GSM_ERROR;
    }

    /* ========== 阶段2: 等待 '>' 提示符 ========== */

    printf("[GSM] Waiting for '>'\r\n");

    if (gsm_wait_response(">", 10000U) != GSM_OK)
    {
        uint8_t escape = 27U;
        (void)HAL_UART_Transmit(&huart2, &escape, 1U, 100U);
        printf("[GSM] Timeout waiting for '>'\r\n");
        return GSM_TIMEOUT;
    }

    /* ========== 阶段3: 发送短信内容 ========== */

    gsm_clear_rx_buffer();

    hal_status = HAL_UART_Transmit(&huart2,
                                   (uint8_t*)text,
                                   strlen(text),
                                   1000);

    if (hal_status != HAL_OK)
    {
        uint8_t escape = 27U;
        (void)HAL_UART_Transmit(&huart2, &escape, 1U, 100U);
        printf("[GSM] Text TX failed\r\n");
        return GSM_ERROR;
    }

    /* ========== 阶段4: 发送 Ctrl+Z (0x1A) 结束 ========== */

    uint8_t ctrl_z = 0x1A;

    hal_status = HAL_UART_Transmit(&huart2, &ctrl_z, 1, 1000);

    if (hal_status != HAL_OK)
    {
        uint8_t escape = 27U;
        (void)HAL_UART_Transmit(&huart2, &escape, 1U, 100U);
        printf("[GSM] Ctrl+Z TX failed\r\n");
        return GSM_ERROR;
    }

    printf("[GSM] Text sent, waiting for confirmation...\r\n");

    /* ========== 阶段5: 等待 +CMGS: 或 OK ========== */

    status = gsm_wait_response("+CMGS:", GSM_SMS_TIMEOUT_MS);

    if (status == GSM_OK)
    {
        printf("[GSM] SMS sent successfully\r\n");
        printf("[GSM] Response: %s\r\n", gsm_get_last_response());
        return GSM_OK;
    }
    else if (status == GSM_TIMEOUT)
    {
        printf("[GSM] SMS timeout\r\n");
        return GSM_TIMEOUT;
    }
    else
    {
        printf("[GSM] SMS failed: %d\r\n", status);
        printf("[GSM] Response: %s\r\n", gsm_get_last_response());
        return GSM_SMS_FAILED;
    }
}

/* ========================================================================= */
/* 私有辅助函数                                                              */
/* ========================================================================= */

/**
 * @brief  等待期望的响应字符串
 * @param  expected  期望的字符串（例如 "OK", "+CMGS:"）
 * @param  timeout_ms超时时间
 * @return GSM_Status
 */
static GSM_Status gsm_wait_response(const char *expected,
                                    uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        /* 无论Idle是否到达都消费DMA新字节，支持响应跨Idle/环形边界。 */
        uint32_t wraps, total;
        uint16_t position;
        if (gsm_rx_error) return GSM_ERROR;
        do {
            wraps = gsm_dma_wraps;
            position = (uint16_t)(GSM_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx));
        } while (wraps != gsm_dma_wraps);
        /* NDTR=0且TC回调尚未执行时，当前位置本身表示一整圈。 */
        total = wraps * GSM_RX_BUFFER_SIZE + position;
        if ((uint32_t)(total - gsm_read_total) >= GSM_RX_BUFFER_SIZE) return GSM_ERROR;
        while (gsm_read_total != total)
        {
            char ch = (char)gsm_dma_buffer[gsm_consumed];
            gsm_consumed = (gsm_consumed + 1U) % GSM_RX_BUFFER_SIZE;
            gsm_read_total++;
            if (gsm_text_length >= GSM_RX_BUFFER_SIZE - 1U) return GSM_ERROR;
            gsm_rx_buffer[gsm_text_length++] = ch;
            gsm_rx_buffer[gsm_text_length] = '\0';
        }
        {
            /* 检查是否包含期望的字符串 */
            if (gsm_contains((const char*)gsm_rx_buffer, expected) &&
                (!strcmp(expected, ">") || gsm_contains(gsm_rx_buffer, "\r\nOK\r\n")))
            {
                return GSM_OK;
            }

            /* 检查是否收到 ERROR */
            if (gsm_contains((const char*)gsm_rx_buffer, "ERROR"))
            {
                return GSM_ERROR;
            }

            /* 重置标志，继续等待 */
            gsm_rx_complete = 0;
        }

        /* 短暂延时，避免占用 CPU */
        gsm_delay_ms(10);
    }

    return GSM_TIMEOUT;
}

/**
 * @brief  检查字符串是否包含子串（不区分大小写）
 * @param  haystack 被搜索的字符串
 * @param  needle   要查找的子串
 * @return 1=包含，0=不包含
 */
static uint8_t gsm_contains(const char *haystack, const char *needle)
{
    if (haystack == NULL || needle == NULL)
    {
        return 0;
    }

    return (strstr(haystack, needle) != NULL) ? 1 : 0;
}

/**
 * @brief  延时函数（FreeRTOS 环境使用 osDelay）
 */
static void gsm_delay_ms(uint32_t ms)
{
    /* 如果在 FreeRTOS 任务中，使用 osDelay */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        while (ms) {
            uint32_t slice = ms > 100U ? 100U : ms;
            health_beat(HEALTH_GSM, HAL_GetTick());
            osDelay(slice); ms -= slice;
        }
    }
    else
    {
        /* 裸机环境使用 HAL_Delay */
        HAL_Delay(ms);
    }
}
