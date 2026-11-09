#ifndef __KEY_H
#define __KEY_H

#include "main.h" // HAL库工程通常包含 main.h 即可获取引脚定义
#include <stdint.h>

// 保留必要的引脚映射宏，方便后续修改
#define KEY1_PIN    GPIO_PIN_12
#define KEY1_PORT   GPIOB

#define KEY2_PIN    GPIO_PIN_13
#define KEY2_PORT   GPIOB

#define KEY3_PIN    GPIO_PIN_14
#define KEY3_PORT   GPIOB

#define KEY4_PIN    GPIO_PIN_15
#define KEY4_PORT   GPIOB

void KEY_Init(void);
uint8_t KEY_Scan(void); // 返回按键值：1~4，无按键返回 0

#endif