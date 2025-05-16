#include "spi.h"
#include "main.h"

/* 定义 SPI1 全局句柄，如果 CubeMX 已生成请确保重复定义问题 */
SPI_HandleTypeDef hspi1;

/**
  * @brief  初始化 SPI1 外设
  * 请根据实际情况调整各参数，如波特率分频、数据位、极性、相位等。
  */
void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // 可根据需要调整
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* 内部函数：通过 SPI 发送数据 */
static void W25Q64_SPI_Transmit(uint8_t *data, uint16_t length)
{
    HAL_SPI_Transmit(&hspi1, data, length, HAL_MAX_DELAY);
}

/* 内部函数：通过 SPI 接收数据 */
static void W25Q64_SPI_Receive(uint8_t *data, uint16_t length)
{
    HAL_SPI_Receive(&hspi1, data, length, HAL_MAX_DELAY);
}

void W25Q64_WriteEnable(void)
{
    uint8_t cmd = W25Q64_CMD_WREN;
    W25Q64_CS_LOW();
    W25Q64_SPI_Transmit(&cmd, 1);
    W25Q64_CS_HIGH();
}

uint8_t W25Q64_IsBusy(void)
{
    uint8_t cmd = W25Q64_CMD_RDSR;
    uint8_t status = 0;
    W25Q64_CS_LOW();
    W25Q64_SPI_Transmit(&cmd, 1);
    W25Q64_SPI_Receive(&status, 1);
    W25Q64_CS_HIGH();
    return (status & 0x01);
}

void W25Q64_SectorErase(uint32_t address)
{
    uint8_t cmd[4];
    cmd[0] = W25Q64_CMD_SE;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;
    W25Q64_WriteEnable();
    W25Q64_CS_LOW();
    W25Q64_SPI_Transmit(cmd, 4);
    W25Q64_CS_HIGH();
    while(W25Q64_IsBusy());
}

void W25Q64_WriteData(uint32_t address, uint8_t *data, uint16_t length)
{
    uint8_t cmd[4];
    cmd[0] = W25Q64_CMD_WRITE;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;
    W25Q64_WriteEnable();
    W25Q64_CS_LOW();
    W25Q64_SPI_Transmit(cmd, 4);
    W25Q64_SPI_Transmit(data, length);
    W25Q64_CS_HIGH();
    while(W25Q64_IsBusy());
}

void W25Q64_ReadData(uint32_t address, uint8_t *data, uint16_t length)
{
    uint8_t cmd[4];
    cmd[0] = W25Q64_CMD_READ;
    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;
    W25Q64_CS_LOW();
    W25Q64_SPI_Transmit(cmd, 4);
    W25Q64_SPI_Receive(data, length);
    W25Q64_CS_HIGH();
}
