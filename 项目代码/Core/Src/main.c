/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序
  * 功能：
  *  - 采集体重（通过 WeightReal/ReadCount）与心率（MAX30102）数据，在 OLED 上显示并通过 UART 输出
  *  - 当按下 KEY2 时，将当前体重与心率保存到记录数组（最多 3 组）
  *  - 通过 SPI 将数组中的记录保存到 W25Q64 Flash（按下 KEY3 触发存储）
  *  - 通过蓝牙 DMA 空闲接收（UART DMA）指令“b1”、“b2”、“b3”读取对应记录，或“load”指令加载 Flash 中数据
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"
#include "spi.h"
#include "oled.h"
#include "max30102.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct {
  float weight;      // 单位：克
  uint32_t bpm;      // 心率值
} Record_t;

/* Private define ------------------------------------------------------------*/
#define NO_SIGNAL_THRESHOLD 30000    // 红外数据阈值（无手指时低于此值）
#define BEAT_THRESHOLD      50000    // 心搏检测阈值
#define MIN_BEAT_INTERVAL   400      // 最小心搏间隔（ms）
#define MAX_BEAT_INTERVAL   2000     // 最大心搏间隔（ms）
#define MOVING_AVG_SIZE     5        // 移动平均滤波窗口

#define CMD_LENGTH 2                 // DMA接收蓝牙命令的最小长度

#define W25Q64_BASE_ADDR 0x000000    // Flash 存储起始地址

/* Private variables ---------------------------------------------------------*/
// ——体重相关变量（由外部文件或用户提供）——
extern uint8_t ReceiveBuff[BUFSIZ];
extern uint8_t recv_end_flag, Rx_len;
extern uint32_t weight_first;
extern uint32_t weight_real;
uint32_t weight_print = 0;  // 实际体重（单位：克）


// ——蓝牙/UART相关变量（全部采用DMA收发）——
uint8_t receiveData[50];    // DMA 接收缓冲区
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

// ——心率计算变量——
uint32_t lastBeatTime = 0;
uint32_t beatInterval = 0;
uint32_t bpm = 0;
uint32_t prevIR = 0;
int32_t  prevDiff = 0;
uint32_t beatIntervals[MOVING_AVG_SIZE] = {0};
uint8_t  beatCount = 0;
uint8_t  beatIndex = 0;
uint32_t lastOutputTime = 0;

// ——记录数据变量——（最多3组）
Record_t records[3] = {0};
uint8_t recordCount = 0;

// ——单位切换变量——
// unit_mode = 0：公斤；unit_mode = 1：磅
volatile uint8_t unit_mode = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void update_display_and_transmit(float f_w, uint32_t current_bpm);
static void send_record(uint8_t index);
void SaveRecordsToFlash(void);
void LoadRecordsFromFlash(void);

/* Private user code ---------------------------------------------------------*/

/* 主函数 */
int main(void)
{
  /* MCU 初始化 */
  HAL_Init();
  SystemClock_Config();

  /* 初始化所有外设 */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();   // SPI 用于 W25Q64 Flash 存储

  /* 延时等待各外设稳定 */
  HAL_Delay(100);
  OLED_Init();
  ADC_Init();

  /* 开启 UART DMA 空闲接收，用于蓝牙指令 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveData, sizeof(receiveData));
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  /* 获取初始体重计数 */
  weight_first = ReadCount();

  /* 初始化 MAX30102 传感器 */
  if (MAX30102_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  uint8_t fifoData[6];   // 用于存放 MAX30102 FIFO 数据
  char msg[128];         // UART 发送缓存

  /* 低电量检测变量 */
  uint32_t adcValue = 0;
  float voltage = 0.0f;
  const float lowVoltageThreshold = 1.7f;  // 根据实际情况调整

  while (1)
  {
    /* 低电量检测 */
    adcValue = ADC_ReadValue();
    voltage = ADC_ConvertToVoltage(adcValue);
    if (voltage < lowVoltageThreshold)
    {
      OLED_NewFrame();
      sprintf(msg, "Low Battery");
      OLED_PrintString(0, 15, msg, &font16x16, OLED_COLOR_NORMAL);
      OLED_ShowFrame();
      HAL_Delay(5000);
      __HAL_RCC_PWR_CLK_ENABLE();
      HAL_PWR_EnterSTANDBYMode();
    }

    /* 体重采集 */
    weight_print = WeightReal();
    float f_w = (float)weight_print;

    /* 心率数据采集 */
    if (MAX30102_ReadFIFO(&hi2c1, fifoData, 6) != HAL_OK)
    {
      Error_Handler();
    }
    uint32_t redVal = (((uint32_t)fifoData[0] << 16) | ((uint32_t)fifoData[1] << 8) | fifoData[2]) & 0x03FFFF;
    uint32_t irVal  = (((uint32_t)fifoData[3] << 16) | ((uint32_t)fifoData[4] << 8) | fifoData[5]) & 0x03FFFF;
    uint32_t currentTime = HAL_GetTick();

    if (irVal < NO_SIGNAL_THRESHOLD)
    {
      bpm = 0;
      lastBeatTime = currentTime;
      beatCount = 0;
      beatIndex = 0;
      prevIR = irVal;
      prevDiff = 0;
      if (currentTime - lastOutputTime >= 500)
      {
        update_display_and_transmit(f_w, bpm);
        lastOutputTime = currentTime;
      }
      HAL_Delay(5);
      continue;
    }

    int32_t diff = (int32_t)irVal - (int32_t)prevIR;
    if ((prevDiff > 0) && (diff < 0) && (prevIR > BEAT_THRESHOLD) &&
        ((currentTime - lastBeatTime) >= MIN_BEAT_INTERVAL))
    {
      beatInterval = currentTime - lastBeatTime;
      if (beatInterval >= MIN_BEAT_INTERVAL && beatInterval <= MAX_BEAT_INTERVAL)
      {
        lastBeatTime = currentTime;
        beatIntervals[beatIndex] = beatInterval;
        beatIndex = (beatIndex + 1) % MOVING_AVG_SIZE;
        if (beatCount < MOVING_AVG_SIZE)
          beatCount++;
        uint32_t sumIntervals = 0;
        for (uint8_t i = 0; i < beatCount; i++)
        {
          sumIntervals += beatIntervals[i];
        }
        uint32_t avgInterval = sumIntervals / beatCount;
        bpm = (avgInterval > 0) ? (60000 / avgInterval) : 0;
      }
      else
      {
        lastBeatTime = currentTime;
      }
    }
    prevDiff = diff;
    prevIR = irVal;

    /* 每500ms更新OLED显示和UART发送数据 */
    if (currentTime - lastOutputTime >= 500)
    {
      update_display_and_transmit(f_w, bpm);
      lastOutputTime = currentTime;
    }

    /* KEY1: 切换单位（公斤 <-> 磅） */
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
      unit_mode = (unit_mode == 0) ? 1 : 0;
      while (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {}
    }

    /* KEY2: 存储当前数据到数组（最多3组） */
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
    {
      if (recordCount < 3)
      {
        records[recordCount].weight = f_w;
        records[recordCount].bpm = bpm;
        recordCount++;
        char regMsg[50];
        sprintf(regMsg, "记录%d已存储\r\n", recordCount);
        HAL_UART_Transmit(&huart1, (uint8_t*)regMsg, strlen(regMsg), HAL_MAX_DELAY);
      }
      while (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {}
    }

    /* KEY3: 将记录数组保存到 W25Q64 Flash */
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
    {
      SaveRecordsToFlash();
      char infoMsg[] = "Records saved to flash\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*)infoMsg, strlen(infoMsg), HAL_MAX_DELAY);
      while (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {}
    }

    /* 可扩展其他任务 */
  }
}

/**
  * @brief  更新 OLED 显示并通过 UART 输出当前体重、心率数据
  * @param  f_w 当前体重（单位：克）
  * @param  current_bpm 当前心率
  */
static void update_display_and_transmit(float f_w, uint32_t current_bpm)
{
  char message1[50];
  char message2[50];
  char msg[128];

  OLED_NewFrame();
  if (unit_mode == 0)
  {
    sprintf(message1, "体重：%.3fkg", f_w / 1000);
  }
  else
  {
    sprintf(message1, "体重：%.3fLb", (f_w / 1000) * 2.204);
  }
  OLED_PrintString(0, 0, message1, &font16x16, OLED_COLOR_NORMAL);
  sprintf(message2, "心率：%dbpm", current_bpm);
  OLED_PrintString(0, 15, message2, &font16x16, OLED_COLOR_NORMAL);
  OLED_ShowFrame();

  sprintf(msg, "体重：%.3f, 心率:%dbpm\r\n", (unit_mode == 0) ? (f_w/1000) : ((f_w/1000)*2.204), current_bpm);
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/**
  * @brief  根据索引发送记录数据（通过蓝牙命令读取记录）
  * @param  index 记录索引（0 表示第一组）
  */
static void send_record(uint8_t index)
{
  char msg[128];
  if (index < recordCount)
  {
    float weight_in_unit = (unit_mode == 0) ? (records[index].weight / 1000.0f) : ((records[index].weight / 1000.0f) * 2.204);
    const char* unitStr = (unit_mode == 0) ? "kg" : "Lb";
    sprintf(msg, "记录%d: 体重: %.3f%s, 心率:%dbpm\r\n", index + 1, weight_in_unit, unitStr, records[index].bpm);
  }
  else
  {
    sprintf(msg, "记录%d不存在\r\n", index + 1);
  }
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/**
  * @brief 将记录数组中的数据写入 W25Q64 Flash
  * 每组记录占用 12 字节：4字节体重 + 4字节保留 + 4字节心率
  */
void SaveRecordsToFlash(void)
{
  uint8_t buffer[12];
  for (uint8_t i = 0; i < recordCount; i++)
  {
    uint32_t address = W25Q64_BASE_ADDR + (i * 12);
    memcpy(&buffer[0], &records[i].weight, 4);
    memset(&buffer[4], 0, 4); // 保留
    memcpy(&buffer[8], &records[i].bpm, 4);
    W25Q64_WriteData(address, buffer, 12);
  }
}

/**
  * @brief 从 W25Q64 Flash 中读取记录至数组
  */
void LoadRecordsFromFlash(void)
{
  uint8_t buffer[12];
  for (uint8_t i = 0; i < 3; i++)
  {
    uint32_t address = W25Q64_BASE_ADDR + (i * 12);
    W25Q64_ReadData(address, buffer, 12);
    memcpy(&records[i].weight, &buffer[0], 4);
    memcpy(&records[i].bpm, &buffer[8], 4);
  }
  recordCount = 3;
}

/**
  * @brief UART DMA 空闲接收回调函数，用于解析蓝牙命令“b1”, “b2”, “b3”, 以及 “load”
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1)
  {
    if (Size >= CMD_LENGTH)
    {
      char cmd[10] = {0};
      strncpy(cmd, (char*)receiveData, Size);
      if (strncmp(cmd, "b1", 2) == 0)
      {
        send_record(0);
      }
      else if (strncmp(cmd, "b2", 2) == 0)
      {
        send_record(1);
      }
      else if (strncmp(cmd, "b3", 2) == 0)
      {
        send_record(2);
      }
      else if (strncmp(cmd, "load", 4) == 0)
      {
        LoadRecordsFromFlash();
        char loadMsg[] = "Records loaded\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)loadMsg, strlen(loadMsg), HAL_MAX_DELAY);
      }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveData, sizeof(receiveData));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  }
}

/**
  * @brief 系统时钟配置（此处代码基于 CubeMX 生成，请根据实际情况调整）
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** 配置 RCC 振荡器 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** 初始化 CPU, AHB 和 APB 总线时钟 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  错误处理函数
  */
void Error_Handler(void)
{
  __disable_irq();
  while(1)
  {
    /* 用户可添加 LED 闪烁或其他调试指示 */
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* 用户可根据需要添加调试信息 */
}
#endif
