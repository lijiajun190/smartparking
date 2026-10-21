#ifndef BSP_OLED_H
#define BSP_OLED_H

/**
 * @file    bsp_oled.h
 * @brief   STM32F411 OLED SSD1306 驱动接口 (I2C3)
 * @note    硬件连接：PA8(I2C3_SCL), PC9(I2C3_SDA)
 *          OLED模块：VCC接3.3V, GND接GND
 */

#include "main.h"

/* OLED 的 7 位地址为 0x3C；HAL 接口需要传入左移一位后的地址。 */
#define OLED_I2C_ADDRESS       (0x3CU << 1U)

/**
 * SSD1306 的列地址通常从 0 开始。
 * 如果你的模块实际是 SH1106，且画面整体向左偏移，可改为 2。
 */
#ifndef OLED_COLUMN_OFFSET
#define OLED_COLUMN_OFFSET     0U
#endif

/* 模块的可视分辨率为 128x64，即 8 个页。 */
#define OLED_WIDTH             128U
#define OLED_PAGE_COUNT        8U

/**
 * @brief 初始化 SSD1306 OLED
 * @retval HAL_OK: 初始化成功
 * @retval HAL_ERROR: 初始化失败
 */
HAL_StatusTypeDef OLED_Init(void);

/**
 * @brief 清屏（全黑）
 * @retval HAL_OK: 操作成功
 */
HAL_StatusTypeDef OLED_Clear(void);

/**
 * @brief 全屏填充
 * @param fill_data: 填充数据，0x00 为全黑，0xFF 为全亮
 * @retval HAL_OK: 操作成功
 */
HAL_StatusTypeDef OLED_Fill(uint8_t fill_data);

/**
 * @brief 开启 OLED 显示
 * @retval HAL_OK: 操作成功
 */
HAL_StatusTypeDef OLED_ON(void);

/**
 * @brief 关闭 OLED 显示
 * @retval HAL_OK: 操作成功
 */
HAL_StatusTypeDef OLED_OFF(void);

/**
 * @brief 显示一个 8x16 ASCII 字符
 * @param line: 行号，范围 1~4
 * @param column: 列号，范围 1~16
 * @param character: 要显示的字符
 * @retval HAL_OK: 操作成功
 * @retval HAL_ERROR: 参数错误
 */
HAL_StatusTypeDef OLED_ShowChar(uint8_t line, uint8_t column, char character);

/**
 * @brief 显示一个 8x16 ASCII 字符串
 * @param line: 行号，范围 1~4
 * @param column: 起始列号，范围 1~16
 * @param text: 要显示的字符串
 * @retval HAL_OK: 操作成功
 * @retval HAL_ERROR: 参数错误
 */
HAL_StatusTypeDef OLED_ShowString(uint8_t line, uint8_t column, const char *text);

/**
 * @brief 显示无符号十进制数
 * @param line: 行号，范围 1~4
 * @param column: 起始列号，范围 1~16
 * @param number: 要显示的数字
 * @param length: 显示位数
 * @retval HAL_OK: 操作成功
 */
HAL_StatusTypeDef OLED_ShowNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);

/**
 * @brief 显示有符号十进制数
 * @param line: 行号，范围 1~4
 * @param column: 起始列号，范围 1~16
 * @param number: 要显示的数字
 * @param length: 显示位数（不包括符号位）
 * @retval HAL_OK: 操作成功
 */
HAL_StatusTypeDef OLED_ShowSignedNum(uint8_t line, uint8_t column, int32_t number, uint8_t length);

/**
 * @brief 显示十六进制数
 * @param line: 行号，范围 1~4
 * @param column: 起始列号，范围 1~16
 * @param number: 要显示的数字
 * @param length: 显示位数
 * @retval HAL_OK: 操作成功
 */
HAL_StatusTypeDef OLED_ShowHexNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);

/**
 * @brief 显示二进制数
 * @param line: 行号，范围 1~4
 * @param column: 起始列号，范围 1~16
 * @param number: 要显示的数字
 * @param length: 显示位数
 * @retval HAL_OK: 操作成功
 */
HAL_StatusTypeDef OLED_ShowBinNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);

/**
 * @brief 兼容旧版坐标式字符串显示接口
 * @param x: X坐标（必须是8的倍数）
 * @param page: 页地址（必须是2的倍数）
 * @param text: 要显示的字符串
 * @param text_size: 字体大小，当前只支持 2 (8x16)
 * @retval HAL_OK: 操作成功
 * @retval HAL_ERROR: 参数错误
 */
HAL_StatusTypeDef OLED_ShowStr(uint8_t x, uint8_t page, const char *text, uint8_t text_size);

#endif /* BSP_OLED_H */
