/**
 * @file    gsm_test.h
 * @brief   SIM800L GSM 模块测试接口
 */

#ifndef GSM_TEST_H
#define GSM_TEST_H

#include <stdint.h>

/**
 * @brief  基础 AT 命令测试
 * @return 0=成功，非0=失败
 * @note   测试 AT、ATE0、CPIN、CSQ 命令
 */
uint8_t gsm_test_basic_at(void);

/**
 * @brief  完整初始化测试
 * @return 0=成功，非0=失败
 * @note   调用 gsm_init() 并验证结果
 */
uint8_t gsm_test_init(void);

/**
 * @brief  短信发送测试
 * @return 0=成功，非0=失败
 * @note   发送测试短信到预设号码
 */
uint8_t gsm_test_send_sms(void);

/**
 * @brief  网络注册状态测试
 * @return 0=成功，非0=失败
 * @note   查询网络注册和运营商信息
 */
uint8_t gsm_test_network_status(void);

/**
 * @brief  完整的集成测试流程
 * @return 0=全部通过，非0=失败
 * @note   依次执行所有测试项
 */
uint8_t gsm_test_full(void);

/**
 * @brief  循环发送测试短信（压力测试）
 * @param  count 发送次数
 * @param  interval_ms 发送间隔（毫秒）
 * @return 成功发送的次数
 */
uint8_t gsm_test_send_multiple(uint8_t count, uint32_t interval_ms);

#endif /* GSM_TEST_H */
