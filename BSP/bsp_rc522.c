/**
 * @file    bsp_rc522.c
 * @brief   MFRC522 RFID 读卡器驱动实现
 * @note    驱动架构说明:
 *          1. 底层: SPI读写寄存器（rc522_read_reg / rc522_write_reg）
 *          2. 中层: 命令执行与数据收发（rc522_command_transceive）
 *          3. 上层: ISO14443A 寻卡/防冲撞协议（rc522_request / rc522_anticoll）
 *          4. 应用层: 去抖封装（rc522_read_with_debounce）
 * 
 *          关键时序:
 *          - 复位后需要延时等待芯片就绪
 *          - 每次命令执行都有超时保护（避免死等）
 *          - 天线需要手动打开才能检测到卡片
 */

#include "bsp_rc522.h"
#include "main.h"
#include <string.h>

/* ========================================================================= */
/* 引脚控制宏                                                                */
/* ========================================================================= */
#define RC522_CS_LOW()      HAL_GPIO_WritePin(RC522_CS_GPIO_Port, RC522_CS_Pin, GPIO_PIN_RESET)
#define RC522_CS_HIGH()     HAL_GPIO_WritePin(RC522_CS_GPIO_Port, RC522_CS_Pin, GPIO_PIN_SET)
#define RC522_RST_LOW()     HAL_GPIO_WritePin(RC522_RST_GPIO_Port, RC522_RST_Pin, GPIO_PIN_RESET)
#define RC522_RST_HIGH()    HAL_GPIO_WritePin(RC522_RST_GPIO_Port, RC522_RST_Pin, GPIO_PIN_SET)

/* Software SPI follows the known-good legacy driver: SCK idles high. */
#define RC522_SCK_LOW()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
#define RC522_SCK_HIGH()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define RC522_MOSI_LOW()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)
#define RC522_MOSI_HIGH()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)
#define RC522_MISO_READ()   HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)

/* About 5 us per half-cycle at the project clock; adjust only if needed. */
static void rc522_soft_spi_delay(void)
{
    for (volatile uint32_t i = 0; i < 180U; i++)
    {
        __NOP();
    }
}

static void rc522_soft_spi_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin = RC522_CS_Pin | GPIO_PIN_5 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin = RC522_RST_Pin;
    HAL_GPIO_Init(RC522_RST_GPIO_Port, &gpio);

    RC522_CS_HIGH();
    RC522_SCK_LOW();   // 修正：SPI Mode 0, SCK空闲低电平
    RC522_MOSI_LOW();
}

/* ========================================================================= */
/* 超时配置                                                                  */
/* ========================================================================= */
#define RC522_SPI_TIMEOUT_MS    100U    /* SPI 单次传输超时 */
#define RC522_CMD_TIMEOUT_MS    50U     /* 命令执行超时 */

/* ========================================================================= */
/* 去抖状态（静态变量，保存上次读卡结果）                                    */
/* ========================================================================= */
static uint8_t  s_last_uid[4] = {0};        /* 上次读到的UID */
static uint32_t s_last_read_tick = 0;       /* 上次读卡时刻 */
static uint8_t  s_has_last_uid = 0;         /* 是否有有效的上次UID */

/* ========================================================================= */
/* 底层 SPI 寄存器读写                                                       */
/* ========================================================================= */

/**
 * @brief  读取 RC522 寄存器
 * @param  addr  寄存器地址（0x00 - 0x3F）
 * @retval 寄存器值
 * @note   SPI 读取格式: 地址字节高位为1 表示读，((addr<<1)&0x7E)|0x80
 */
static uint8_t rc522_read_reg(uint8_t addr)
{
    uint8_t address;
    uint8_t value = 0U;
    uint8_t bit;

    /* SPI 读取: 地址最高位置1，bit0固定为0 */
    address = ((addr << 1) & 0x7E) | 0x80;

    RC522_CS_LOW();
    for (bit = 0U; bit < 8U; bit++)
    {
        if ((address & 0x80U) != 0U)
            RC522_MOSI_HIGH();
        else
            RC522_MOSI_LOW();

        rc522_soft_spi_delay();
        RC522_SCK_LOW();
        rc522_soft_spi_delay();
        RC522_SCK_HIGH();
        rc522_soft_spi_delay();
        address <<= 1;
    }

    for (bit = 0U; bit < 8U; bit++)
    {
        value <<= 1;
        RC522_SCK_LOW();
        rc522_soft_spi_delay();
        RC522_SCK_HIGH();        // 先产生上升沿
        rc522_soft_spi_delay();
        if (RC522_MISO_READ() == GPIO_PIN_SET)  // 在上升沿后采样 (SPI Mode 0)
            value |= 0x01U;
        rc522_soft_spi_delay();
    }
    RC522_SCK_LOW();  // 恢复空闲状态

    RC522_CS_HIGH();
    return value;
}

/**
 * @brief  写入 RC522 寄存器
 * @param  addr  寄存器地址
 * @param  value 要写入的值
 * @note   SPI 写入格式: 地址字节高位为0 表示写，((addr<<1)&0x7E)
 */
static void rc522_write_reg(uint8_t addr, uint8_t value)
{
    uint8_t address;
    uint8_t bit;

    /* 地址字节: 最高位为0表示写 */
    address = (addr << 1) & 0x7EU;

    RC522_CS_LOW();
    for (bit = 0U; bit < 8U; bit++)
    {
        if ((address & 0x80U) != 0U)
            RC522_MOSI_HIGH();
        else
            RC522_MOSI_LOW();

        rc522_soft_spi_delay();
        RC522_SCK_LOW();
        rc522_soft_spi_delay();
        RC522_SCK_HIGH();
        rc522_soft_spi_delay();
        address <<= 1;
    }

    for (bit = 0U; bit < 8U; bit++)
    {
        if ((value & 0x80U) != 0U)
            RC522_MOSI_HIGH();
        else
            RC522_MOSI_LOW();

        rc522_soft_spi_delay();
        RC522_SCK_LOW();
        rc522_soft_spi_delay();
        RC522_SCK_HIGH();
        rc522_soft_spi_delay();
        value <<= 1;
    }

    RC522_CS_HIGH();
}

/**
 * @brief  对寄存器某些位置1（OR 操作）
 * @param  addr  寄存器地址
 * @param  mask  要置1的位掩码
 */
static void rc522_set_bit_mask(uint8_t addr, uint8_t mask)
{
    uint8_t temp;
    temp = rc522_read_reg(addr);
    rc522_write_reg(addr, temp | mask);
}

/**
 * @brief  对寄存器某些位清0（AND NOT 操作）
 * @param  addr  寄存器地址
 * @param  mask  要清0的位掩码
 */
static void rc522_clear_bit_mask(uint8_t addr, uint8_t mask)
{
    uint8_t temp;
    temp = rc522_read_reg(addr);
    rc522_write_reg(addr, temp & (~mask));
}

/* ========================================================================= */
/* 中层: 命令执行与数据收发                                                  */
/* ========================================================================= */

/**
 * @brief  执行 RC522 命令并收发数据
 * @param  command    命令字（参考 RC522_CMD_xxx）
 * @param  send_data  要发送的数据
 * @param  send_len   发送数据长度
 * @param  recv_data  接收数据缓冲区
 * @param  recv_len   返回实际接收到的字节数
 * @retval RC522_OK      = 成功
 *         RC522_TIMEOUT = 超时
 *         RC522_ERR     = 错误
 * @note   这是驱动的核心函数，封装了"发送数据 -> 执行命令 -> 等待完成 -> 读取结果"的完整流程
 */
static RC522_Status rc522_command_transceive(uint8_t command,
                                               uint8_t *send_data,
                                               uint8_t send_len,
                                               uint8_t *recv_data,
                                               uint8_t recv_capacity,
                                               uint8_t *recv_len)
{
    uint8_t  n;
    uint8_t  irq_en   = 0x00;
    uint8_t  wait_irq = 0x00;
    uint8_t  last_bits;
    uint8_t  fifo_level;
    uint32_t start_tick;
    uint8_t  irq_reg;
    uint8_t  error_reg;

    /* 根据命令设置对应的中断使能和等待标志 */
    switch (command)
    {
        case RC522_CMD_AUTHENT:     /* 认证命令 */
            irq_en   = 0x12;        /* 使能 IdleIRq 和 ErrIRq */
            wait_irq = 0x10;        /* 等待 IdleIRq */
            break;

        case RC522_CMD_TRANSCEIVE:  /* 收发命令（寻卡/防冲撞） */
            irq_en   = 0x77;        /* 使能所有中断 */
            wait_irq = 0x30;        /* 等待 RxIRq 或 IdleIRq */
            break;

        default:
            break;
    }

    /* 配置命令对应的中断源 */
    rc522_write_reg(RC522_REG_COMIEN, irq_en | 0x80U);
    rc522_write_reg(RC522_REG_DIVIEN, 0x00U);

    /* Clear the Set1 control bit as in the known-good driver. */
    rc522_clear_bit_mask(RC522_REG_COMIRQ, 0x80);

    /* 清空 FIFO */
    rc522_set_bit_mask(RC522_REG_FIFOLEVEL, 0x80);

    /* 进入 IDLE 状态，停止当前命令 */
    rc522_write_reg(RC522_REG_COMMAND, RC522_CMD_IDLE);

    /* 将要发送的数据写入 FIFO */
    for (n = 0; n < send_len; n++)
    {
        rc522_write_reg(RC522_REG_FIFODATA, send_data[n]);
    }

    /* 启动命令执行 */
    rc522_write_reg(RC522_REG_COMMAND, command);

    /* 如果是收发命令，需要启动发送 */
    if (command == RC522_CMD_TRANSCEIVE)
    {
        rc522_set_bit_mask(RC522_REG_BITFRAMING, 0x80);  /* StartSend=1 */
    }

    /* 等待命令完成（超时保护） */
    start_tick = HAL_GetTick();
    while (1)
    {
        irq_reg = rc522_read_reg(RC522_REG_COMIRQ);

        /* 检查超时 */
        if ((HAL_GetTick() - start_tick) > RC522_CMD_TIMEOUT_MS)
        {
            rc522_clear_bit_mask(RC522_REG_BITFRAMING, 0x80);
            rc522_write_reg(RC522_REG_COMMAND, RC522_CMD_IDLE);
            return RC522_TIMEOUT;
        }

        /* 检查目标中断是否触发 */
        if (irq_reg & wait_irq)
        {
            break;  /* 命令完成 */
        }

        /* 检查是否发生错误 */
        if (irq_reg & 0x01)  /* TimerIRq 超时 */
        {
            rc522_clear_bit_mask(RC522_REG_BITFRAMING, 0x80);
            rc522_write_reg(RC522_REG_COMMAND, RC522_CMD_IDLE);
            return RC522_TIMEOUT;
        }
    }

    rc522_clear_bit_mask(RC522_REG_BITFRAMING, 0x80);

    /* 读取错误寄存器，检查是否有通信错误 */
    error_reg = rc522_read_reg(RC522_REG_ERROR);

    #ifdef RC522_DEBUG
    printf("[RC522] error_reg=0x%02X, irq_reg=0x%02X\r\n", error_reg, irq_reg);
    #endif

    /* 协议错误、奇偶校验错误、CRC错误都视为通信失败 */
    if (error_reg & 0x1BU)
    {
        #ifdef RC522_DEBUG
        if (error_reg & 0x01) printf("[RC522] ProtocolErr\r\n");
        if (error_reg & 0x02) printf("[RC522] ParityErr\r\n");
        if (error_reg & 0x08) printf("[RC522] CollErr\r\n");
        if (error_reg & 0x10) printf("[RC522] CRCErr\r\n");
        #endif
        rc522_write_reg(RC522_REG_COMMAND, RC522_CMD_IDLE);
        return (error_reg & 0x08U) ? RC522_COLLISION : RC522_ERR;
    }

    /* 读取接收到的数据 */
    if (recv_data != NULL && recv_len != NULL)
    {
        fifo_level = rc522_read_reg(RC522_REG_FIFOLEVEL);
        if (fifo_level > recv_capacity) {
            rc522_write_reg(RC522_REG_COMMAND, RC522_CMD_IDLE);
            return RC522_ERR;
        }
        *recv_len  = fifo_level;

        /* 最后一个字节可能不满8位，记录有效位数 */
        last_bits = rc522_read_reg(RC522_REG_CONTROL) & 0x07;

        if (last_bits != 0)
        {
            /* 最后一字节有效位数 < 8，这里简化处理，按完整字节读取 */
        }

        /* 限制接收长度，避免缓冲区溢出（假设缓冲区足够大） */
        if (fifo_level > 0)
        {
            for (n = 0; n < fifo_level; n++)
            {
                recv_data[n] = rc522_read_reg(RC522_REG_FIFODATA);
            }
        }
    }

    rc522_write_reg(RC522_REG_COMMAND, RC522_CMD_IDLE);
    return RC522_OK;
}

/* ========================================================================= */
/* 上层: ISO14443A 协议实现                                                  */
/* ========================================================================= */

/**
 * @brief  寻卡（Request）
 */
RC522_Status rc522_request(uint8_t req_mode, uint8_t *card_type)
{
    RC522_Status status;
    uint8_t recv_buf[2];
    uint8_t recv_len = 0;

    /* 配置为短帧格式（7位） */
    rc522_write_reg(RC522_REG_BITFRAMING, 0x07);

    /* 发送请求命令 */
    recv_buf[0] = req_mode;
    status = rc522_command_transceive(RC522_CMD_TRANSCEIVE,
                                        recv_buf, 1,
                                        recv_buf, sizeof(recv_buf), &recv_len);

    #ifdef RC522_DEBUG
    printf("[RC522] Request: status=%d, recv_len=%d\r\n", status, recv_len);
    #endif

    /* 正常应答应该返回2字节的卡片类型（ATQA） */
    if (status == RC522_TIMEOUT) return RC522_NOTAG;
    if (status != RC522_OK) return status;
    if (recv_len != 2U) return RC522_ERR;

    /* 返回卡片类型 */
    if (card_type != NULL)
    {
        card_type[0] = recv_buf[0];
        card_type[1] = recv_buf[1];
    }

    return RC522_OK;
}

/**
 * @brief  防冲撞，读取UID
 */
RC522_Status rc522_anticoll(uint8_t *uid)
{
    RC522_Status status;
    uint8_t send_buf[2];
    uint8_t recv_buf[5];  /* 4字节UID + 1字节BCC校验 */
    uint8_t recv_len = 0;
    uint8_t i;
    uint8_t bcc_check = 0;

    /* 取消短帧模式，恢复正常字节传输 */
    rc522_write_reg(RC522_REG_BITFRAMING, 0x00);

    /* 防冲撞命令: 0x93 0x20 */
    send_buf[0] = PICC_ANTICOLL;
    send_buf[1] = 0x20;

    status = rc522_command_transceive(RC522_CMD_TRANSCEIVE,
                                        send_buf, 2,
                                        recv_buf, sizeof(recv_buf), &recv_len);

    /* 正常应该返回5字节: 4字节UID + 1字节BCC */
    if ((status != RC522_OK) || (recv_len != 5))
    {
        return RC522_ERR;
    }

    /* BCC 校验: UID[0] ^ UID[1] ^ UID[2] ^ UID[3] 应该等于 BCC */
    bcc_check = 0;
    for (i = 0; i < 4; i++)
    {
        bcc_check ^= recv_buf[i];
    }

    if (bcc_check != recv_buf[4])
    {
        return RC522_COLLISION;  /* 校验失败，可能有多卡冲突 */
    }

    /* 复制 UID 到输出缓冲区 */
    if (uid != NULL)
    {
        for (i = 0; i < 4; i++)
        {
            uid[i] = recv_buf[i];
        }
    }

    return RC522_OK;
}

/* ========================================================================= */
/* 天线控制                                                                  */
/* ========================================================================= */

/**
 * @brief  打开天线
 * @note   TX1 和 TX2 是天线驱动引脚，必须都打开才能正常工作
 */
void rc522_antenna_on(void)
{
    uint8_t temp;

    temp = rc522_read_reg(RC522_REG_TXCONTROL);

    if ((temp & 0x03) != 0x03)  /* 检查 Tx1RFEn 和 Tx2RFEn 是否已开启 */
    {
        rc522_set_bit_mask(RC522_REG_TXCONTROL, 0x03);
    }
}

/**
 * @brief  关闭天线
 */
void rc522_antenna_off(void)
{
    rc522_clear_bit_mask(RC522_REG_TXCONTROL, 0x03);
}

/* ========================================================================= */
/* 初始化与自检                                                              */
/* ========================================================================= */

/**
 * @brief  读取版本寄存器（用于验证 SPI 通信）
 * @retval 版本号，常见值: 0x91 或 0x92
 */
uint8_t rc522_read_version(void)
{
    return rc522_read_reg(RC522_REG_VERSION);
}

/**
 * @brief  RC522 初始化
 */
HAL_StatusTypeDef rc522_init(void)
{
    uint8_t version;

    rc522_soft_spi_gpio_init();

    /* 硬件复位 */
    RC522_RST_LOW();
    HAL_Delay(10);   /* 保持复位至少10ms */
    RC522_RST_HIGH();
    HAL_Delay(50);   /* 等待芯片启动 */

    /* 软件复位 */
    rc522_write_reg(RC522_REG_COMMAND, RC522_CMD_SOFTRESET);
    HAL_Delay(50);   /* 复位后需要延时等待内部状态稳定 */

    /* 读取版本寄存器验证通信 */
    version = rc522_read_version();
    if ((version != 0x91U) && (version != 0x92U) && (version != 0x88U) && (version != 0x82U))
    {
        /* 版本号不正确，SPI 通信可能有问题 */
        return HAL_ERROR;
    }

    /* 配置定时器（用于超时控制） */
    rc522_write_reg(RC522_REG_TMODE, 0x8D);        /* TAuto=1, 自动启动定时器 */
    rc522_write_reg(RC522_REG_TPRESCALER, 0x3E);   /* 与已验证的软件 SPI 例程一致 */
    rc522_write_reg(RC522_REG_TRELOAD_H, 0x00);    /* 重载值高字节 */
    rc522_write_reg(RC522_REG_TRELOAD_L, 30);      /* 重载值低字节 */

    /* 配置发射功率和通信模式 */
    rc522_write_reg(RC522_REG_TXAUTO, 0x40);       /* 强制100% ASK调制 */
    rc522_write_reg(RC522_REG_MODE, 0x3D);         /* CRC初始值0x6363 */

    /* ISO14443A 模式参数，来自已验证的标准库驱动。 */
    rc522_clear_bit_mask(RC522_REG_STATUS2, 0x08);
    rc522_write_reg(RC522_REG_RXSEL, 0x86);
    rc522_write_reg(RC522_REG_RFCFG, 0x7F);

    
    /* 打开天线 */
    rc522_antenna_on();

    return HAL_OK;
}

/* ========================================================================= */
/* 应用层: 去抖封装                                                          */
/* ========================================================================= */

/**
 * @brief  带去抖的读卡接口
 * @note   去抖逻辑:
 *         1. 如果没有卡，返回 RC522_NOTAG
 *         2. 如果读到的卡与上次相同，且时间间隔 < debounce_ms，返回 RC522_NOTAG
 *         3. 否则，更新去抖状态并返回 RC522_OK
 */
RC522_Status rc522_read_with_debounce(uint8_t *uid, uint32_t debounce_ms)
{
    RC522_Status status;
    uint8_t card_type[2];
    uint8_t temp_uid[4];
    uint32_t current_tick;

    /* 第一步: 寻卡 */
    status = rc522_request(PICC_REQALL, card_type);
    if (status != RC522_OK)
    {
        /* 无卡，清除去抖状态 */
        if (status == RC522_NOTAG &&
            (uint32_t)(HAL_GetTick() - s_last_read_tick) >= debounce_ms) s_has_last_uid = 0;
        return status;
    }

    /* 第二步: 读取 UID */
    status = rc522_anticoll(temp_uid);
    if (status != RC522_OK)
    {
        return status;  /* 冲突或错误 */
    }

    /* 第三步: 去抖判断 */
    current_tick = HAL_GetTick();

    if (s_has_last_uid != 0)
    {
        /* 与上次卡片比较 */
        if (memcmp(temp_uid, s_last_uid, 4) == 0)
        {
            /* 同一张卡，检查时间间隔 */
            {
                s_last_read_tick = current_tick; /* 持卡不重复触发，移开稳定后才重新受理 */
                /* 在去抖时间窗口内，不重复上报 */
                return RC522_NOTAG;
            }
        }
    }

    /* 新卡或超过去抖时间，更新状态 */
    memcpy(s_last_uid, temp_uid, 4);
    s_last_read_tick = current_tick;
    s_has_last_uid = 1;

    /* 复制 UID 到输出 */
    if (uid != NULL)
    {
        memcpy(uid, temp_uid, 4);
    }

    return RC522_OK;
}
