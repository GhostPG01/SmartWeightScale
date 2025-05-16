#ifndef __SPI_H
#define __SPI_H

#include "stm32f1xx_hal.h"

/* W25Q64 指令定义 */
#define W25Q64_CMD_READ   0x03    // 读取数据
#define W25Q64_CMD_WRITE  0x02    // 写入数据
#define W25Q64_CMD_WREN   0x06    // 写使能
#define W25Q64_CMD_RDSR   0x05    // 读取状态寄存器
#define W25Q64_CMD_SE     0x20    // 擦除扇区（4KB）

/* SPI 片选控制，请确保 SPI_CS_GPIO_Port 与 SPI_CS_Pin 在 CubeMX 中已配置 */
#define W25Q64_CS_LOW()   HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET)
#define W25Q64_CS_HIGH()  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET)

/* 函数声明 */
void MX_SPI1_Init(void);
void W25Q64_WriteEnable(void);
uint8_t W25Q64_IsBusy(void);
void W25Q64_SectorErase(uint32_t address);
void W25Q64_WriteData(uint32_t address, uint8_t *data, uint16_t length);
void W25Q64_ReadData(uint32_t address, uint8_t *data, uint16_t length);

#endif // __SPI_H
