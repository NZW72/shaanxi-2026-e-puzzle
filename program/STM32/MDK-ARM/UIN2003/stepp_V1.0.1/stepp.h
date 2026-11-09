#ifndef __STEPPER_H
#define __STEPPER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* 电机参数：4000 步 = 一圈 = 360° */
#define STEPS_PER_REVOLUTION   4000
#define DEGREE_TO_STEP(deg)    ((int32_t)((deg) * (float)STEPS_PER_REVOLUTION / 360.0f + 0.5f))

/**
  * @brief 初始化步进电机 GPIO（PA0, PA1, PA4, PA5）
  */
void stepper_init(void);

/**
  * @brief 设置当前位置（软件归零）
  */
void stepper_set_position(int32_t pos);

/**
  * @brief 获取当前步数位置
  */
int32_t stepper_get_position(void);

/**
  * @brief 相对移动指定步数（正数正转，负数反转）
  * @param steps     步数（可正可负）
  * @param delay_us  每步延时（微秒），控制转速
  * @note  阻塞执行，直到走完
  */
void stepper_move_relative(int32_t steps, uint32_t delay_us);

/**
  * @brief 旋转指定角度（正数正转，负数反转）
  * @param angle_deg 角度（度），可正可负
  * @param delay_us  每步延时（微秒）
  */
void stepper_rotate(float angle_deg, uint32_t delay_us);

/**
  * @brief 将当前位置设为 0（软件归零）
  */
void stepper_reset_zero(void);

/**
  * @brief 获取当前是否正在运行（阻塞模式下始终返回 0）
  */
uint8_t stepper_is_running(void);

#endif