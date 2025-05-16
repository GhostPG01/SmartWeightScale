#include "hx711.h"
#include "stdio.h"
#include "stdint.h"
#include "gpio.h"

uint8_t ReceiveBuff[BUFFERSIZE]; //接收缓冲区
uint8_t recv_end_flag = 0,Rx_len;//接收完成中断标志，接收到字符长度

uint32_t weight_first = 0;//首次质量
uint32_t weight_real = 0;//每次读取的实际值

/***********************************************************************************************
*Name : ReadCount
*Function : 读取传感器数值，通道A，增益128
*Parameter :    count 计数值
                i    24位数据计数，改变i的值可选择不同通道和增益24~26
***********************************************************************************************/
uint32_t ReadCount(void)
{

	uint32_t Count = 0;
	uint8_t i = 0;
	HAL_Delay(10);
	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);

	while(HAL_GPIO_ReadPin(DT_GPIO_Port, DT_Pin));//判断AD转换器是否准备好（DOUT应为低电平）
	for(i = 0; i <24; i++)//24
	{

	  HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
		Count = Count << 1;
		if(HAL_GPIO_ReadPin(DT_GPIO_Port, DT_Pin))
		{

		    Count ++;
		}
		HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
	Count = Count ^ 0x800000;
	HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
	return Count;
}

/***********************************************************************************************
*Name : ReadWeight
*Function : 读取拉力值
*Parameter :   weight_real          实际拉力值
               weight_first         传感器首次数据
***********************************************************************************************/
uint32_t WeightReal(void)
{

   weight_real = ReadCount();//实际质量计算
		if(weight_first>=weight_real)
		{

			weight_real = weight_first-weight_real;
			weight_real = weight_real/100;
			//printf("重量为：%dg\n",weight_real);
		}
		else
		{

			weight_real = weight_real-weight_first;
			weight_real = weight_real/100;
			//printf("重量为：-%dg\n",weight_real);
		}
		return weight_real;
}
