/**
 * @file    gsm_test.c
 * @brief   SIM800L GSM 模块独立测试
 * @note    这个文件提供独立的测试函数，用于验证 SIM800L 硬件连接和驱动
 *          可以在 main.c 或独立任务中调用
 */

#include "gsm_test.h"
#include "bsp_sim800.h"
#include <stdio.h>
#include <string.h>

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"

/* ========================================================================= */
/* 测试配置                                                                  */
/* ========================================================================= */

/* 测试短信接收号码（修改为你的手机号） */
#define TEST_PHONE_NUMBER   "13335515640"

/* ========================================================================= */
/* 测试函数实现                                                              */
/* ========================================================================= */

/**
 * @brief  基础 AT 命令测试
 * @return 0=成功，非0=失败
 * @note   测试项：
 *         1. AT 响应
 *         2. ATE0 关闭回显
 *         3. AT+CPIN? SIM 卡检测
 *         4. AT+CSQ 信号强度
 */
uint8_t gsm_test_basic_at(void)
{
    GSM_Status status;
    uint8_t rssi;

    printf("\r\n");
    printf("========================================\r\n");
    printf("  GSM Basic AT Test\r\n");
    printf("========================================\r\n");

    /* 延迟等待模块上电 */
    printf("[TEST] Waiting for module power-up...\r\n");
    vTaskDelay(3000);

    /* 测试 1: AT */
    printf("\r\n[TEST 1/4] Testing AT command...\r\n");
    status = gsm_send_command("AT", "OK", 2000);

    if (status == GSM_OK)
    {
        printf("[PASS] AT responded OK\r\n");
    }
    else
    {
        printf("[FAIL] AT failed: %d\r\n", status);
        printf("       Check: power supply, RX/TX wiring, baud rate\r\n");
        return 1;
    }

    /* 测试 2: ATE0 */
    printf("\r\n[TEST 2/4] Testing ATE0 (disable echo)...\r\n");
    status = gsm_send_command("ATE0", "OK", 2000);

    if (status == GSM_OK)
    {
        printf("[PASS] Echo disabled\r\n");
    }
    else
    {
        printf("[FAIL] ATE0 failed: %d\r\n", status);
        return 2;
    }

    /* 测试 3: AT+CPIN? */
    printf("\r\n[TEST 3/4] Checking SIM card...\r\n");
    status = gsm_check_sim();

    if (status == GSM_OK)
    {
        printf("[PASS] SIM card ready\r\n");
    }
    else
    {
        printf("[FAIL] SIM card error: %d\r\n", status);
        printf("       Check: SIM card inserted correctly\r\n");
        return 3;
    }

    /* 测试 4: AT+CSQ */
    printf("\r\n[TEST 4/4] Checking signal strength...\r\n");
    status = gsm_check_signal(&rssi);

    if (status == GSM_OK)
    {
        printf("[PASS] Signal RSSI: %d\r\n", rssi);

        if (rssi == 99)
        {
            printf("[WARN] Signal unknown (99)\r\n");
        }
        else if (rssi < 10)
        {
            printf("[WARN] Weak signal (< 10)\r\n");
        }
        else
        {
            printf("[INFO] Good signal (>= 10)\r\n");
        }
    }
    else
    {
        printf("[FAIL] Signal check failed: %d\r\n", status);
        return 4;
    }

    printf("\r\n========================================\r\n");
    printf("  Basic AT Test: PASSED\r\n");
    printf("========================================\r\n\r\n");

    return 0;
}

/**
 * @brief  完整初始化测试
 * @return 0=成功，非0=失败
 */
uint8_t gsm_test_init(void)
{


    
    GSM_Status status;

    printf("\r\n");
    printf("========================================\r\n");
    printf("  GSM Initialization Test\r\n");
    printf("========================================\r\n");

    status = gsm_init();

    if (status == GSM_OK)
    {
        printf("\r\n[PASS] GSM initialization successful\r\n");

        /* 等待网络注册（增加等待时间） */
        printf("[GSM] Waiting for network registration...\r\n");
        printf("[INFO] This may take 10-60 seconds\r\n\r\n");

        uint8_t registered = 0;
        for (int i = 0; i < 60; i++)  /* 最多等待 60 秒 */
        {
            vTaskDelay(1000);

            status = gsm_send_command("AT+CREG?", "+CREG:", 2000);
            const char *resp = gsm_get_last_response();

            /* 检查是否注册：+CREG: 0,1 (本地) 或 +CREG: 0,5 (漫游) */
            if (strstr(resp, "+CREG: 0,1") != NULL ||
                strstr(resp, "+CREG: 0,5") != NULL)
            {
                printf("[PASS] Network registered! (after %d seconds)\r\n", i + 1);
                registered = 1;
                break;
            }

            /* 每 5 秒输出一次进度 */
            if ((i + 1) % 5 == 0)
            {
                printf("[INFO] Still waiting for network... (%d/60)\r\n", i + 1);
            }
        }

        if (!registered)
        {
            printf("\r\n[WARN] Network registration timeout\r\n");
            printf("      SMS sending may fail\r\n");
            printf("      Check: antenna, SIM card, signal coverage\r\n");
        }

        printf("========================================\r\n\r\n");
        return 0;
    }
    else
    {
        printf("\r\n[FAIL] GSM initialization failed: %d\r\n", status);

        switch (status)
        {
            case GSM_NO_RESPONSE:
                printf("       Possible causes:\r\n");
                printf("       - Power supply insufficient (need 4V/2A)\r\n");
                printf("       - RX/TX wiring reversed\r\n");
                printf("       - Baud rate mismatch\r\n");
                break;

            case GSM_SIM_ERROR:
                printf("       Possible causes:\r\n");
                printf("       - SIM card not inserted\r\n");
                printf("       - SIM card locked (PIN required)\r\n");
                printf("       - SIM card damaged\r\n");
                break;

            case GSM_WEAK_SIGNAL:
                printf("       Possible causes:\r\n");
                printf("       - Antenna not connected\r\n");
                printf("       - Poor signal coverage\r\n");
                break;

            default:
                printf("       Unknown error\r\n");
                break;
        }

        printf("========================================\r\n\r\n");
        return (uint8_t)status;
    }
}




/**
 * @brief  短信发送测试
 * @return 0=成功，非0=失败
 * @note   需要先调用 gsm_init() 成功后才能测试
 */
uint8_t gsm_test_send_sms(void)
{
    GSM_Status status;

    printf("\r\n");
    printf("========================================\r\n");
    printf("  GSM SMS Sending Test\r\n");
    printf("========================================\r\n");

    printf("[TEST] Sending test SMS to %s...\r\n", TEST_PHONE_NUMBER);
    printf("[INFO] This may take 10-30 seconds\r\n\r\n");

    status = gsm_send_sms(TEST_PHONE_NUMBER, "Test from STM32F4 Smart Parking");

    if (status == GSM_OK)
    {
        printf("\r\n[PASS] SMS sent successfully\r\n");
        printf("       Check your phone for the message\r\n");
        printf("========================================\r\n\r\n");
        return 0;
    }
    else
    {
        printf("\r\n[FAIL] SMS sending failed: %d\r\n", status);

        switch (status)
        {
            case GSM_TIMEOUT:
                printf("       Possible causes:\r\n");
                printf("       - Network not registered\r\n");
                printf("       - Poor signal coverage\r\n");
                printf("       - SIM card has no credit\r\n");
                break;

            case GSM_SMS_FAILED:
                printf("       Possible causes:\r\n");
                printf("       - Invalid phone number format\r\n");
                printf("       - SMS service not enabled on SIM\r\n");
                break;

            default:
                printf("       Unknown error\r\n");
                break;
        }

        printf("========================================\r\n\r\n");
        return (uint8_t)status;
    }
}

/**
 * @brief  网络注册状态测试
 * @return 0=成功，非0=失败
 */
uint8_t gsm_test_network_status(void)
{
    GSM_Status status;

    printf("\r\n");
    printf("========================================\r\n");
    printf("  GSM Network Registration Test\r\n");
    printf("========================================\r\n");

    /* 查询网络注册状态 */
    printf("[TEST] Querying network registration (AT+CREG?)...\r\n");
    status = gsm_send_command("AT+CREG?", "+CREG:", 3000);

    if (status == GSM_OK)
    {
        printf("[PASS] Network query successful\r\n");
        printf("Response: %s\r\n", gsm_get_last_response());
    }
    else
    {
        printf("[FAIL] Network query failed: %d\r\n", status);
    }

    /* 查询运营商 */
    printf("\r\n[TEST] Querying operator (AT+COPS?)...\r\n");
    status = gsm_send_command("AT+COPS?", "+COPS:", 3000);

    if (status == GSM_OK)
    {
        printf("[PASS] Operator query successful\r\n");
        printf("Response: %s\r\n", gsm_get_last_response());
    }
    else
    {
        printf("[FAIL] Operator query failed: %d\r\n", status);
    }

    printf("========================================\r\n\r\n");

    return 0;
}

/**
 * @brief  完整的集成测试流程
 * @return 0=全部通过，非0=失败
 * @note   依次执行所有测试项
 */
uint8_t gsm_test_full(void)
{
    uint8_t result;

    printf("\r\n\r\n");
    printf("****************************************\r\n");
    printf("*  SIM800L Full Integration Test      *\r\n");
    printf("****************************************\r\n");

    /* 阶段 1: 基础 AT 测试 */
    result = gsm_test_basic_at();
    if (result != 0)
    {
        printf("\r\n[ABORT] Basic AT test failed, stopping tests\r\n");
        return result;
    }

    vTaskDelay(1000);

    /* 阶段 2: 完整初始化 */
    result = gsm_test_init();
    if (result != 0)
    {
        printf("\r\n[ABORT] Initialization failed, stopping tests\r\n");
        return result;
    }

    vTaskDelay(1000);

    /* 阶段 3: 网络状态 */
    result = gsm_test_network_status();

    vTaskDelay(1000);

    /* 阶段 4: 短信发送 */
    printf("[INFO] SMS test requires manual verification\r\n");
    printf("[INFO] Press any key to send test SMS, or skip...\r\n");

    /* 可选：等待用户确认 */
    vTaskDelay(2000);

    result = gsm_test_send_sms();

    printf("\r\n****************************************\r\n");
    if (result == 0)
    {
        printf("*  Full Test: ALL PASSED               *\r\n");
    }
    else
    {
        printf("*  Full Test: FAILED (code %d)         *\r\n", result);
    }
    printf("****************************************\r\n\r\n");

    return result;
}

/**
 * @brief  循环发送测试短信（压力测试）
 * @param  count 发送次数
 * @param  interval_ms 发送间隔（毫秒）
 * @return 成功发送的次数
 */
uint8_t gsm_test_send_multiple(uint8_t count, uint32_t interval_ms)
{
    uint8_t success_count = 0;
    uint8_t i;
    char text[100];
    GSM_Status status;

    printf("\r\n");
    printf("========================================\r\n");
    printf("  GSM Multiple SMS Test\r\n");
    printf("  Count: %d, Interval: %lu ms\r\n", count, interval_ms);
    printf("========================================\r\n");

    for (i = 0; i < count; i++)
    {
        snprintf(text, sizeof(text), "Test SMS %d/%d from STM32", i + 1, count);

        printf("\r\n[TEST %d/%d] Sending...\r\n", i + 1, count);

        status = gsm_send_sms(TEST_PHONE_NUMBER, text);

        if (status == GSM_OK)
        {
            printf("[PASS] SMS %d sent\r\n", i + 1);
            success_count++;
        }
        else
        {
            printf("[FAIL] SMS %d failed: %d\r\n", i + 1, status);
        }

        /* 最后一次不延时 */
        if (i < count - 1)
        {
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
        }
    }

    printf("\r\n========================================\r\n");
    printf("  Result: %d/%d sent successfully\r\n", success_count, count);
    printf("========================================\r\n\r\n");

    return success_count;
}
