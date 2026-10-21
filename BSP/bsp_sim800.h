/**
 * @file    bsp_sim800.h
 * @brief   SIM800L GSM 模块驱动头文件
 * @note    硬件要求:
 *          - 独立 4V/2A 电源供电（峰值电流 2A）
 *          - 电源近端加 1000uF 电解电容
 *          - GND 必须与 STM32 共地
 *          - USART2: PA2(TX), PA3(RX)
 *          - 波特率: 115200
 *
 *          通信架构:
 *          1. 使用 DMA 循环模式接收 AT 响应
 *          2. 使用 UART Idle 中断检测接收完成
 *          3. 每条 AT 命令都有超时保护
 *          4. 支持命令重试机制
 *
 *          初始化流程:
 *          AT       -> 检查模块响应
 *          ATE0     -> 关闭回显
 *          AT+CPIN? -> 检查 SIM 卡
 *          AT+CSQ   -> 检查信号强度
 *          AT+CMGF=1-> 文本模式
 */

#ifndef BSP_SIM800_H
#define BSP_SIM800_H

#include <stdint.h>

/* ========================================================================= */
/* 状态码定义                                                                */
/* ========================================================================= */

/**
 * @brief GSM 模块状态码
 */
typedef enum
{
    GSM_OK = 0,              /* 操作成功 */
    GSM_ERROR,               /* 通用错误 */
    GSM_TIMEOUT,             /* 命令超时 */
    GSM_NO_RESPONSE,         /* 无响应 */
    GSM_SIM_ERROR,           /* SIM 卡错误 */
    GSM_NO_NETWORK,          /* 未注册网络 */
    GSM_WEAK_SIGNAL,         /* 信号弱 */
    GSM_SMS_FAILED,          /* 短信发送失败 */
    GSM_BUSY                 /* 模块忙 */
} GSM_Status;

/* ========================================================================= */
/* 配置参数                                                                  */
/* ========================================================================= */

#define GSM_RX_BUFFER_SIZE      512U    /* DMA 接收缓冲区大小 */
#define GSM_CMD_TIMEOUT_MS      5000U   /* 普通 AT 命令超时 */
#define GSM_SMS_TIMEOUT_MS      30000U  /* 短信发送超时 */
#define GSM_INIT_TIMEOUT_MS     10000U  /* 初始化超时 */

#define GSM_MIN_SIGNAL_QUALITY  10U     /* 最小可用信号强度 */

/* ========================================================================= */
/* 公共函数接口                                                              */
/* ========================================================================= */

/**
 * @brief  初始化 SIM800L 模块
 * @return GSM_Status 状态码
 * @note   执行流程:
 *         1. 启动 DMA 接收
 *         2. 发送 AT 测试连接
 *         3. 关闭回显
 *         4. 检查 SIM 卡
 *         5. 检查信号质量
 *         6. 设置短信文本模式
 *         7. 查询网络注册状态
 *
 *         超时时间约 10 秒，如果失败返回对应错误码
 */
GSM_Status gsm_init(void);

/**
 * @brief  发送短信
 * @param  phone 电话号码字符串，例如 "13800138000"
 * @param  text  短信内容，最长 160 个 ASCII 字符
 * @return GSM_Status 状态码
 * @note   发送流程:
 *         1. 发送 AT+CMGS="号码"
 *         2. 等待 '>' 提示符
 *         3. 发送短信内容
 *         4. 发送 Ctrl+Z (0x1A) 结束
 *         5. 等待 +CMGS: xxx 响应
 *
 *         整个流程耗时 5-30 秒，调用者必须在独立任务中调用
 */
GSM_Status gsm_send_sms(const char *phone, const char *text);

/**
 * @brief  发送 AT 命令并等待期望的响应
 * @param  command          AT 命令字符串（自动添加 \r\n）
 * @param  expected_response期望的响应字符串（NULL 表示只等待 "OK"）
 * @param  timeout_ms       超时时间（毫秒）
 * @return GSM_Status 状态码
 * @note   这是底层接口，上层可以用它发送自定义 AT 命令
 *         例如: gsm_send_command("AT+COPS?", "+COPS:", 5000)
 */
GSM_Status gsm_send_command(const char *command,
                            const char *expected_response,
                            uint32_t timeout_ms);

/**
 * @brief  检查信号质量
 * @param  rssi 信号强度指针（0-31，99=未知）
 * @return GSM_Status 状态码
 * @note   发送 AT+CSQ 并解析响应
 *         rssi < 10: 信号弱，不建议发送短信
 *         rssi >= 10: 信号可用
 */
GSM_Status gsm_check_signal(uint8_t *rssi);

/**
 * @brief  软复位 GSM 模块
 * @return GSM_Status 状态码
 * @note   发送 AT+CFUN=1,1 重启模块
 */
GSM_Status gsm_reset(void);

/**
 * @brief  检查 SIM 卡状态
 * @return GSM_Status 状态码
 * @note   发送 AT+CPIN? 检查 SIM 卡是否就绪
 *         响应 READY 表示正常
 */
GSM_Status gsm_check_sim(void);

/**
 * @brief  启动 DMA 接收（初始化时自动调用）
 * @note   启动 USART2 的 DMA 循环接收
 *         使用 Idle 中断检测接收完成
 */
void gsm_start_receive(void);

/**
 * @brief  获取最后一次接收到的响应内容（用于调试）
 * @return 接收缓冲区指针（只读）
 */
const char* gsm_get_last_response(void);

/**
 * @brief  清空接收缓冲区
 * @note   发送新命令前调用，避免旧数据干扰
 */
void gsm_clear_rx_buffer(void);
/* UART错误回调仅标记；任务检测错误后在下次命令前重启DMA。 */
void gsm_receive_error(void);

#endif /* BSP_SIM800_H */
