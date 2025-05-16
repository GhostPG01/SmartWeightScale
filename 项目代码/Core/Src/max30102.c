#include "max30102.h"
#include "stm32f1xx_hal.h"

HAL_StatusTypeDef MAX30102_Init(I2C_HandleTypeDef* hi2c)
{
    HAL_StatusTypeDef ret;
    uint8_t buf;

    // 复位传感器：写入复位命令到 MODE_CONFIG 寄存器
    buf = 0x40;
    ret = HAL_I2C_Mem_Write(hi2c, MAX30102_ADDR, MAX30102_REG_MODE_CONFIG,
                              I2C_MEMADD_SIZE_8BIT, &buf, 1, HAL_MAX_DELAY);
    if(ret != HAL_OK)
        return ret;

    HAL_Delay(100);

    // 清零 FIFO 写指针和读指针
    buf = 0x00;
    ret = HAL_I2C_Mem_Write(hi2c, MAX30102_ADDR, MAX30102_REG_FIFO_WR_PTR,
                              I2C_MEMADD_SIZE_8BIT, &buf, 1, HAL_MAX_DELAY);
    if(ret != HAL_OK)
        return ret;
    ret = HAL_I2C_Mem_Write(hi2c, MAX30102_ADDR, MAX30102_REG_FIFO_RD_PTR,
                              I2C_MEMADD_SIZE_8BIT, &buf, 1, HAL_MAX_DELAY);
    if(ret != HAL_OK)
        return ret;

    // 设置模式为 SpO2 模式（0x03 表示 SpO2 模式，详见数据手册）
    buf = 0x03;
    ret = HAL_I2C_Mem_Write(hi2c, MAX30102_ADDR, MAX30102_REG_MODE_CONFIG,
                              I2C_MEMADD_SIZE_8BIT, &buf, 1, HAL_MAX_DELAY);
    if(ret != HAL_OK)
        return ret;

    // 设置 LED 脉冲幅度（示例值，根据实际需要调整）
    buf = 0x24;
    ret = HAL_I2C_Mem_Write(hi2c, MAX30102_ADDR, MAX30102_REG_LED1_PA,
                              I2C_MEMADD_SIZE_8BIT, &buf, 1, HAL_MAX_DELAY);
    if(ret != HAL_OK)
        return ret;
    ret = HAL_I2C_Mem_Write(hi2c, MAX30102_ADDR, MAX30102_REG_LED2_PA,
                              I2C_MEMADD_SIZE_8BIT, &buf, 1, HAL_MAX_DELAY);
    if(ret != HAL_OK)
        return ret;

    // 配置 SpO2 相关参数（采样率、脉冲宽度等，示例值）
    buf = 0x27;
    ret = HAL_I2C_Mem_Write(hi2c, MAX30102_ADDR, MAX30102_REG_SPO2_CONFIG,
                              I2C_MEMADD_SIZE_8BIT, &buf, 1, HAL_MAX_DELAY);

    return ret;
}

HAL_StatusTypeDef MAX30102_ReadFIFO(I2C_HandleTypeDef* hi2c, uint8_t *buffer, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MAX30102_ADDR, MAX30102_REG_FIFO_DATA,
                              I2C_MEMADD_SIZE_8BIT, buffer, len, HAL_MAX_DELAY);
}
