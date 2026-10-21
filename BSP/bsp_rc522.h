/*
 *RC522RFID读卡器驱动头文件
 *软件SPI：PA5-SCK   PA6-MISO    PA7-MOSI   PA4-CS   PC4-RST复位
 *时序：SCK空闲高电平，软件产生时钟
 */

#ifndef BSP_RC522_H
#define BSP_RC522_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>

/* 调试宏 - 开启后会输出详细调试信息 */
#define RC522_DEBUG

/* ========================================================================= */
/* RC522 寄存器地址定义 (参考 MFRC522 数据手册)                              */
/* ========================================================================= */

/* 命令和状态寄存器 */
#define RC522_REG_COMMAND       0x01    /* 启动和停止命令的执行 */
#define RC522_REG_COMIEN        0x02    /* 中断请求使能控制位 */
#define RC522_REG_DIVIEN        0x03    /* 中断请求使能控制位 */
#define RC522_REG_COMIRQ        0x04    /* 中断请求标志位 */
#define RC522_REG_DIVIRQ        0x05    /* 中断请求标志位 */
#define RC522_REG_ERROR         0x06    /* 错误标志位 */
#define RC522_REG_STATUS1       0x07    /* 通信状态位 */
#define RC522_REG_STATUS2       0x08    /* 接收器和发送器状态位 */
#define RC522_REG_FIFODATA      0x09    /* FIFO 数据输入输出 */
#define RC522_REG_FIFOLEVEL     0x0A    /* FIFO 中存储的字节数 */
#define RC522_REG_CONTROL       0x0C    /* 各种控制寄存器 */
#define RC522_REG_BITFRAMING    0x0D    /* 位帧调整 */

/* 命令寄存器 */
#define RC522_REG_MODE          0x11    /* 定义发送和接收的通用模式 */
#define RC522_REG_TXCONTROL     0x14    /* 控制天线驱动引脚TX1和TX2的逻辑行为 */
#define RC522_REG_TXAUTO        0x15    /* 控制天线驱动器设置 */

#define RC522_REG_RXSEL        0x17
#define RC522_REG_RFCFG        0x26

/* 配置寄存器 */
#define RC522_REG_CRCRESULT_H   0x21    /* CRC 计算结果高字节 */
#define RC522_REG_CRCRESULT_L   0x22    /* CRC 计算结果低字节 */
#define RC522_REG_TMODE         0x2A    /* 定时器模式设置 */
#define RC522_REG_TPRESCALER    0x2B    /* 定时器预分频 */
#define RC522_REG_TRELOAD_H     0x2C    /* 定时器重载值高字节 */
#define RC522_REG_TRELOAD_L     0x2D    /* 定时器重载值低字节 */
#define RC522_REG_VERSION       0x37    /* 显示版本号 */

/* ========================================================================= */
/* RC522 命令字 (写入 CommandReg 寄存器)                                    */
/* ========================================================================= */
#define RC522_CMD_IDLE          0x00    /* 无动作，取消当前命令 */
#define RC522_CMD_CALCCRC       0x03    /* 激活CRC协处理器 */
#define RC522_CMD_TRANSMIT      0x04    /* 发送FIFO中的数据 */
#define RC522_CMD_RECEIVE       0x08    /* 激活接收电路 */
#define RC522_CMD_TRANSCEIVE    0x0C    /* 发送FIFO数据并自动激活接收 */
#define RC522_CMD_AUTHENT       0x0E    /* 执行Mifare标准认证 */
#define RC522_CMD_SOFTRESET     0x0F    /* 复位RC522 */

/* ========================================================================= */
/* Mifare 卡片命令 (ISO 14443A 协议)                                        */
/* ========================================================================= */
#define PICC_REQIDL             0x26    /* 寻天线区内未进入休眠状态的卡 */
#define PICC_REQALL             0x52    /* 寻天线区内全部卡 */
#define PICC_ANTICOLL           0x93    /* 防冲撞，读取卡片序列号 */
#define PICC_SELECTTAG          0x93    /* 选卡 */
#define PICC_AUTHENT1A          0x60    /* 验证A密钥 */
#define PICC_AUTHENT1B          0x61    /* 验证B密钥 */
#define PICC_READ               0x30    /* 读块 */
#define PICC_WRITE              0xA0    /* 写块 */
#define PICC_HALT               0x50    /* 休眠 */

/* ========================================================================= */
/* 返回状态码                                                                */
/* ========================================================================= */
typedef enum
{
    RC522_OK          = 0,      /* 成功 */
    RC522_ERR         = 1,      /* 错误 */
    RC522_NOTAG       = 2,      /* 无卡 */
    RC522_COLLISION   = 3,      /* 冲突 */
    RC522_TIMEOUT     = 4       /* 超时 */
} RC522_Status;

/* ========================================================================= */
/* 驱动接口函数                                                              */
/* ========================================================================= */

/**
 * @brief  RC522 模块初始化
 * @note   复位芯片、配置寄存器、打开天线、读取版本号验证通信
 * @retval HAL_OK = 初始化成功; HAL_ERROR = 失败（SPI通信异常或版本号错误）
 */
HAL_StatusTypeDef rc522_init(void);

/**
 * @brief  寻卡（检测卡片是否存在）
 * @param  req_mode  寻卡模式
 *                   - PICC_REQIDL (0x26): 只寻未进入休眠的卡
 *                   - PICC_REQALL (0x52): 寻所有卡
 * @param  card_type 返回卡片类型（2字节）
 *                   常见值: 0x0004 = Mifare S50, 0x0002 = Mifare S70
 * @retval RC522_OK     = 有卡应答
 *         RC522_NOTAG  = 无卡
 *         RC522_ERR    = 通信错误
 */
RC522_Status rc522_request(uint8_t req_mode, uint8_t *card_type);

/**
 * @brief  防冲撞，读取卡片唯一序列号（UID）
 * @param  uid  输出缓冲区，至少4字节
 * @retval RC522_OK        = 成功读取UID
 *         RC522_COLLISION = 检测到多张卡冲突
 *         RC522_ERR       = 通信错误
 * @note   UID 是卡片的唯一身份标识，用于识别不同卡片
 */
RC522_Status rc522_anticoll(uint8_t *uid);

/**
 * @brief  带去抖的读卡接口（推荐使用）
 * @param  uid        输出UID缓冲区（4字节）
 * @param  debounce_ms 去抖时间（毫秒），同一张卡在此时间内只上报一次
 *                     建议值: 500ms
 * @retval RC522_OK     = 成功读取到新卡
 *         RC522_NOTAG  = 无卡或卡片未变化（在去抖时间内）
 *         RC522_ERR    = 错误
 * @note   内部维护上次卡片的UID和时间戳，避免重复读取同一张卡
 */
RC522_Status rc522_read_with_debounce(uint8_t *uid, uint32_t debounce_ms);

/**
 * @brief  读取RC522版本寄存器（用于自检）
 * @retval 版本号，常见值: 0x91 或 0x92
 *         如果返回 0x00 或 0xFF，通常表示SPI通信异常
 */
uint8_t rc522_read_version(void);

/**
 * @brief  打开天线
 * @note   停止读卡一段时间后可以关天线省电，需要时再打开
 */
void rc522_antenna_on(void);

/**
 * @brief  关闭天线
 */
void rc522_antenna_off(void);

#endif /* BSP_RC522_H */
