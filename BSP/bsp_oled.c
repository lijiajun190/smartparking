/**
 * @file    bsp_oled.c
 * @brief   OLED SSD1306 驱动 - 完全按照标准库移植到HAL库
 * @note    引脚: PA8(SCL), PC9(SDA), 软件I2C
 */

#include "bsp_oled.h"
#include "oled_font.h"
#include "main.h"

/* 引脚宏定义 - 对应标准库 */
#define OLED_W_SCL(x)   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define OLED_W_SDA(x)   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)

/* 引脚初始化 */
static void OLED_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    OLED_W_SCL(1);
    OLED_W_SDA(1);
}

/* I2C开始 */
static void OLED_I2C_Start(void)
{
    OLED_W_SDA(1);
    OLED_W_SCL(1);
    OLED_W_SDA(0);
    OLED_W_SCL(0);
}

/* I2C停止 */
static void OLED_I2C_Stop(void)
{
    OLED_W_SDA(0);
    OLED_W_SCL(1);
    OLED_W_SDA(1);
}

/* I2C发送一个字节 */
static void OLED_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        OLED_W_SDA(!!(Byte & (0x80 >> i)));
        OLED_W_SCL(1);
        OLED_W_SCL(0);
    }
    OLED_W_SCL(1);
    OLED_W_SCL(0);
}

/* OLED写命令 */
static void OLED_WriteCommand(uint8_t Command)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);
    OLED_I2C_SendByte(0x00);
    OLED_I2C_SendByte(Command);
    OLED_I2C_Stop();
}

/* OLED写数据 */
static void OLED_WriteData(uint8_t Data)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);
    OLED_I2C_SendByte(0x40);
    OLED_I2C_SendByte(Data);
    OLED_I2C_Stop();
}

/* OLED设置光标位置 */
static void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));
    OLED_WriteCommand(0x00 | (X & 0x0F));
}

/* OLED清屏 */
HAL_StatusTypeDef OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        for(i = 0; i < 128; i++)
        {
            OLED_WriteData(0x00);
        }
    }
    return HAL_OK;
}

/* OLED显示一个字符 */
HAL_StatusTypeDef OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i]);
    }
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
    }
    return HAL_OK;
}

/* OLED显示字符串 */
HAL_StatusTypeDef OLED_ShowString(uint8_t Line, uint8_t Column, const char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
    return HAL_OK;
}

/* 次方函数 */
static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

/* OLED显示数字 */
HAL_StatusTypeDef OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
    return HAL_OK;
}

/* OLED显示有符号数 */
HAL_StatusTypeDef OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t Number1;
    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        Number1 = Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        Number1 = -Number;
    }
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
    return HAL_OK;
}

/* OLED显示十六进制 */
HAL_StatusTypeDef OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++)
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10)
        {
            OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        }
        else
        {
            OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
    return HAL_OK;
}

/* OLED显示二进制 */
HAL_StatusTypeDef OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
    return HAL_OK;
}

/* OLED初始化 */
HAL_StatusTypeDef OLED_Init(void)
{
    uint32_t i, j;
    
    /* 上电延时 */
    for (i = 0; i < 1000; i++)
    {
        for (j = 0; j < 1000; j++);
    }
    
    OLED_I2C_Init();
    
    OLED_WriteCommand(0xAE);
    OLED_WriteCommand(0xD5);
    OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8);
    OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0xA1);
    OLED_WriteCommand(0xC8);
    OLED_WriteCommand(0xDA);
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81);
    OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9);
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB);
    OLED_WriteCommand(0x30);
    OLED_WriteCommand(0xA4);
    OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(0xAF);
    
    OLED_Clear();
    
    return HAL_OK;
}

/* 其他函数 */
HAL_StatusTypeDef OLED_Fill(uint8_t fill_data)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        for(i = 0; i < 128; i++)
        {
            OLED_WriteData(fill_data);
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef OLED_ON(void)
{
    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(0xAF);
    return HAL_OK;
}

HAL_StatusTypeDef OLED_OFF(void)
{
    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x10);
    OLED_WriteCommand(0xAE);
    return HAL_OK;
}

HAL_StatusTypeDef OLED_ShowStr(uint8_t x, uint8_t page, const char *text, uint8_t text_size)
{
    if ((text_size != 2) || ((x % 8) != 0) || ((page % 2) != 0))
    {
        return HAL_ERROR;
    }
    return OLED_ShowString(page / 2 + 1, x / 8 + 1, text);
}
