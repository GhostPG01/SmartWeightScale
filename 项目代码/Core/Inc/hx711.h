#ifndef HX711
#define HX711
#include "stdint.h"

#define BUFFERSIZE 255           //可以接收的最大字符个数
uint32_t ReadCount(void);
uint32_t ReadWeight(void);

#endif
