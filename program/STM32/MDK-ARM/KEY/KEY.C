#include "key.h"

// 按键初始化函数
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. 使能 GPIOB 时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // 2. 配置引脚为上拉输入
    GPIO_InitStruct.Pin   = KEY1_PIN | KEY2_PIN | KEY3_PIN | KEY4_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP; // 内部上拉，按下接地变低电平
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// 按键扫描函数（阻塞式，含消抖和释放等待）
uint8_t KEY_Scan(void)
{
    // 检测 KEY1
    if (HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN) == GPIO_PIN_RESET) {
        HAL_Delay(10); // HAL库自带的毫秒级延时
        if (HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN) == GPIO_PIN_RESET) {
            while (HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN) == GPIO_PIN_RESET);
            return 1;
        }
    }
    // 检测 KEY2
    if (HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN) == GPIO_PIN_RESET) {
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN) == GPIO_PIN_RESET) {
            while (HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN) == GPIO_PIN_RESET);
            return 2;
        }
    }
    // 检测 KEY3
    if (HAL_GPIO_ReadPin(KEY3_PORT, KEY3_PIN) == GPIO_PIN_RESET) {
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(KEY3_PORT, KEY3_PIN) == GPIO_PIN_RESET) {
            while (HAL_GPIO_ReadPin(KEY3_PORT, KEY3_PIN) == GPIO_PIN_RESET);
            return 3;
        }
    }
    // 检测 KEY4
    if (HAL_GPIO_ReadPin(KEY4_PORT, KEY4_PIN) == GPIO_PIN_RESET) {
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(KEY4_PORT, KEY4_PIN) == GPIO_PIN_RESET) {
            while (HAL_GPIO_ReadPin(KEY4_PORT, KEY4_PIN) == GPIO_PIN_RESET);
            return 4;
        }
    }
    
    return 0; // 无按键按下
}