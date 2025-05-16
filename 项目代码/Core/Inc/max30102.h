#ifndef __MAX30102_H
#define __MAX30102_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "stdint.h"

// MAX30102 I2C 地址（7位地址左移1位，适用于 HAL 库）
#define MAX30102_ADDR            (0x57 << 1)

// 寄存器地址定义
#define MAX30102_REG_INTR_STATUS_1  0x00
#define MAX30102_REG_INTR_STATUS_2  0x01
#define MAX30102_REG_INTR_ENABLE_1  0x02
#define MAX30102_REG_INTR_ENABLE_2  0x03
#define MAX30102_REG_FIFO_WR_PTR    0x04
#define MAX30102_REG_OVF_COUNTER    0x05
#define MAX30102_REG_FIFO_RD_PTR    0x06
#define MAX30102_REG_FIFO_DATA      0x07
#define MAX30102_REG_MODE_CONFIG    0x09
#define MAX30102_REG_SPO2_CONFIG    0x0A
#define MAX30102_REG_LED1_PA        0x0C  // IR LED 脉冲安培值
#define MAX30102_REG_LED2_PA        0x0D  // 红光 LED 脉冲安培值

/**
  * @brief  初始化 MAX30102 传感器
  * @param  hi2c: 指向 I2C 句柄的指针
  * @retval HAL 状态
  */
HAL_StatusTypeDef MAX30102_Init(I2C_HandleTypeDef* hi2c);

/**
  * @brief  从 MAX30102 FIFO 读取数据
  * @param  hi2c: 指向 I2C 句柄的指针
  * @param  buffer: 存储 FIFO 数据的缓冲区指针
  * @param  len: 读取的字节数
  * @retval HAL 状态
  */
HAL_StatusTypeDef MAX30102_ReadFIFO(I2C_HandleTypeDef* hi2c, uint8_t *buffer, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __MAX30102_H */
