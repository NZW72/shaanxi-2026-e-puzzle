#include "delay.h"
#include "delay.h"

void Delay_init(void)
{
    // 只初始化，不启动！让 delay_us 来控制启动和停止
    __HAL_TIM_SET_COUNTER(&htim1, 0);
}

void delay_us(uint16_t us)
{
    // 清空计数器
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    
    // 启动定时器
    HAL_TIM_Base_Start(&htim1);
    
    // 等待计数器到达目标值（1MHz = 1us/计数）
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
    
    // 停止定时器
    HAL_TIM_Base_Stop(&htim1);
}

void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
    {
        delay_us(1000);
    }
}