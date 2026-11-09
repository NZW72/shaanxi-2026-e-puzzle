//====================================================================
// A4988.c - 双路步进电机驱动
// 功能：PWM调速、脉冲计数、限位检测、归零校准、行程保护
// 修改：增加移动偏移宏，复位只回退，不处理工作偏移
// 修改：校准值改为双电机独立宏（DEFAULT_PULSES_PER_CM_X/Y）
//====================================================================

#include "A4988.h"
#include "tim.h"          // htim3, htim4
#include "stdio.h"
#include "COMMEND.h"      // motor_flag1 定义

//====================== 引脚定义 ======================
#define MS1_PIN     GPIO_PIN_13
#define MS1_PORT    GPIOC
#define MS2_PIN     GPIO_PIN_14
#define MS2_PORT    GPIOC
#define MS3_PIN     GPIO_PIN_15
#define MS3_PORT    GPIOC

#define DIR1_PIN    GPIO_PIN_7
#define DIR1_PORT   GPIOA
#define DIR2_PIN    GPIO_PIN_5
#define DIR2_PORT   GPIOB

#define LIMIT1_PIN  GPIO_PIN_8
#define LIMIT1_PORT GPIOA
#define LIMIT2_PIN  GPIO_PIN_0
#define LIMIT2_PORT GPIOB

//====================== 全局变量 ======================
static MotorPulseCtl_t motor_ctl[2] = {0};
static float g_pulses_per_cm_x = 0.0f;      // 电机1校准值
static float g_pulses_per_cm_y = 0.0f;      // 电机2校准值
extern motor_flag motor_flag1;

//====================== 私有函数 ======================
static void my_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = MS1_PIN | MS2_PIN | MS3_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DIR1_PIN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = DIR2_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LIMIT1_PIN | LIMIT2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

//====================== 公有函数 ======================
void A4988_Init(void)
{
    my_GPIO_Init();
    A4988_SetMicrostep(MICROSTEP_1);
    A4988_SetDirection(MOTOR_1, DIR_CW);
    A4988_SetDirection(MOTOR_2, DIR_CW);
    for (int i = 0; i < 2; i++) {
        motor_ctl[i].target_pulses = 0;
        motor_ctl[i].current_pulses = 0;
        motor_ctl[i].running = 0;
    }
    // 分别初始化两个电机的校准值
    g_pulses_per_cm_x = DEFAULT_PULSES_PER_CM_X;
    g_pulses_per_cm_y = DEFAULT_PULSES_PER_CM_Y;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Instance->ARR / 2);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, htim4.Instance->ARR / 2);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
}

void A4988_SetMicrostep(MicrostepMode_t mode)
{
    uint8_t ms1, ms2, ms3;
    switch (mode) {
        case MICROSTEP_1:  ms1=0; ms2=0; ms3=0; break;
        case MICROSTEP_2:  ms1=1; ms2=0; ms3=0; break;
        case MICROSTEP_4:  ms1=0; ms2=1; ms3=0; break;
        case MICROSTEP_8:  ms1=1; ms2=1; ms3=0; break;
        case MICROSTEP_16: ms1=1; ms2=1; ms3=1; break;
        default:           ms1=0; ms2=0; ms3=0; break;
    }
    HAL_GPIO_WritePin(MS1_PORT, MS1_PIN, ms1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MS2_PORT, MS2_PIN, ms2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MS3_PORT, MS3_PIN, ms3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void A4988_SetDirection(MotorId_t motor, uint8_t dir)
{
    if (motor == MOTOR_1) {
        HAL_GPIO_WritePin(DIR1_PORT, DIR1_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(DIR2_PORT, DIR2_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

uint8_t A4988_GetLimitSwitch1(void)
{
    return (HAL_GPIO_ReadPin(LIMIT1_PORT, LIMIT1_PIN) == GPIO_PIN_RESET) ? 1 : 0;
}
uint8_t A4988_GetLimitSwitch2(void)
{
    return (HAL_GPIO_ReadPin(LIMIT2_PORT, LIMIT2_PIN) == GPIO_PIN_RESET) ? 1 : 0;
}

void A4988_StartMotor(MotorId_t motor)
{
    if (motor == MOTOR_1) {
        motor_ctl[MOTOR_1].target_pulses = 0;
        motor_ctl[MOTOR_1].current_pulses = 0;
        motor_ctl[MOTOR_1].running = 1;
        motor_flag1.motor1_flag = 1;
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    } else {
        motor_ctl[MOTOR_2].target_pulses = 0;
        motor_ctl[MOTOR_2].current_pulses = 0;
        motor_ctl[MOTOR_2].running = 1;
        motor_flag1.motor2_flag = 1;
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    }
}

void A4988_StartMotorPulses(MotorId_t motor, uint32_t pulses)
{
    if (pulses == 0) return;
    if (motor == MOTOR_1) {
        motor_ctl[MOTOR_1].target_pulses = pulses;
        motor_ctl[MOTOR_1].current_pulses = 0;
        motor_ctl[MOTOR_1].running = 1;
        motor_flag1.motor1_flag = 1;
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    } else {
        motor_ctl[MOTOR_2].target_pulses = pulses;
        motor_ctl[MOTOR_2].current_pulses = 0;
        motor_ctl[MOTOR_2].running = 1;
        motor_flag1.motor2_flag = 1;
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    }
}

// 移动函数：限幅使用回退距离（物理零点到限位的距离），不处理工作偏移
float A4988_MoveDistance(MotorId_t motor, float distance_cm)
{
    if (distance_cm == 0.0f) return 0.0f;

    float calib = (motor == MOTOR_1) ? X_CALIB_DIST : Y_CALIB_DIST;
    float max_forward = MAX_TRAVEL_CM - calib;   // 正向最大移动（从物理零点出发）
    float max_backward = calib;                  // 负向最大移动（回到限位）
    float actual_distance = distance_cm;

    if (distance_cm > 0) {
        if (distance_cm > max_forward) {
            actual_distance = max_forward;
            printf("Distance limited to %.2f cm (max forward)\n", max_forward);
        }
    } else {
        if (-distance_cm > max_backward) {
            actual_distance = -max_backward;
            printf("Distance limited to %.2f cm (max backward)\n", -max_backward);
        }
    }

    // 限位预检
    if ((motor == MOTOR_1 && A4988_GetLimitSwitch1()) ||
        (motor == MOTOR_2 && A4988_GetLimitSwitch2())) {
        printf("Limit switch active, cannot move!\n");
        return 0.0f;
    }

    // 根据电机选择对应的脉冲/厘米校准值
    float ppcm = (motor == MOTOR_1) ? g_pulses_per_cm_x : g_pulses_per_cm_y;
    uint32_t pulses = (uint32_t)(fabsf(actual_distance) * ppcm + 0.5f);
    if (pulses == 0) return actual_distance;

    if (actual_distance > 0) {
        A4988_SetDirection(motor, DIR_CW);
    } else {
        A4988_SetDirection(motor, DIR_CCW);
    }
    A4988_StartMotorPulses(motor, pulses);
    return actual_distance;
}

void A4988_SetPulsesPerCm(float pulses_per_cm)
{
    if (pulses_per_cm > 0) {
        g_pulses_per_cm_x = pulses_per_cm;
        g_pulses_per_cm_y = pulses_per_cm;   // 同时设置两个电机（保持接口不变）
    }
}

void A4988_StopMotor(MotorId_t motor)
{
    if (motor == MOTOR_1) {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        motor_ctl[MOTOR_1].running = 0;
        motor_flag1.motor1_flag = 0;
    } else {
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
        motor_ctl[MOTOR_2].running = 0;
        motor_flag1.motor2_flag = 0;
    }
}

void A4988_CheckLimitsAndStop(void)
{
    if (A4988_GetLimitSwitch1()) A4988_StopMotor(MOTOR_1);
    if (A4988_GetLimitSwitch2()) A4988_StopMotor(MOTOR_2);
}

uint8_t A4988_IsRunning(MotorId_t motor)
{
    return motor_ctl[motor].running;
}

// 复位：碰限位 → 回退 CALIB_DIST，停在物理零点
void A4988_res(void)
{
    A4988_SetDirection(MOTOR_1, DIR_CCW);
    A4988_SetDirection(MOTOR_2, DIR_CCW);
    A4988_StartMotor(MOTOR_1);
    A4988_StartMotor(MOTOR_2);

    while (1) {
        if (A4988_GetLimitSwitch1()) A4988_StopMotor(MOTOR_1);
        if (A4988_GetLimitSwitch2()) A4988_StopMotor(MOTOR_2);
        if (A4988_GetLimitSwitch1() && A4988_GetLimitSwitch2()) break;
        HAL_Delay(10);
    }

    HAL_Delay(100);
    A4988_SetDirection(MOTOR_1, DIR_CW);
    A4988_SetDirection(MOTOR_2, DIR_CW);

    // 分别使用各自的校准值计算回退脉冲
    uint32_t release_pulses_x = (uint32_t)(X_CALIB_DIST * g_pulses_per_cm_x + 0.5f);
    uint32_t release_pulses_y = (uint32_t)(Y_CALIB_DIST * g_pulses_per_cm_y + 0.5f);

    if (release_pulses_x > 0) A4988_StartMotorPulses(MOTOR_1, release_pulses_x);
    if (release_pulses_y > 0) A4988_StartMotorPulses(MOTOR_2, release_pulses_y);

    while (motor_flag1.motor1_flag || motor_flag1.motor2_flag) {
        HAL_Delay(10);
    }
}

//====================== 定时器中断回调 ======================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        if (motor_ctl[MOTOR_1].running) {
            motor_ctl[MOTOR_1].current_pulses++;
            if (motor_ctl[MOTOR_1].target_pulses != 0 &&
                motor_ctl[MOTOR_1].current_pulses >= motor_ctl[MOTOR_1].target_pulses) {
                HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
                motor_ctl[MOTOR_1].running = 0;
                motor_flag1.motor1_flag = 0;
            }
        }
    }
    if (htim->Instance == TIM4) {
        if (motor_ctl[MOTOR_2].running) {
            motor_ctl[MOTOR_2].current_pulses++;
            if (motor_ctl[MOTOR_2].target_pulses != 0 &&
                motor_ctl[MOTOR_2].current_pulses >= motor_ctl[MOTOR_2].target_pulses) {
                HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
                motor_ctl[MOTOR_2].running = 0;
                motor_flag1.motor2_flag = 0;
            }
        }
    }
}