#ifndef __A4988_H
#define __A4988_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

//====================== 用户可调宏定义 ======================

// 校准距离：从限位触发点到物理零点的偏移（单位 cm）
// 复位时电机先碰限位，然后正向回退该距离，使限位释放，该点定义为物理零点
#define X_CALIB_DIST  1.0f    // X轴（电机1）回退距离
#define Y_CALIB_DIST  1.0f    // Y轴（电机2）回退距离

#define MAX_TRAVEL_CM 40.0f   // 物理最大行程（总行程，单位 cm）

/* 双电机独立脉冲/厘米校准值（默认值，可根据实测分别修改） */
#define DEFAULT_PULSES_PER_CM_X   833.45f   // 电机1
#define DEFAULT_PULSES_PER_CM_Y   840.4f   // 电机2

/* 电机编号 */
typedef enum {
    MOTOR_1 = 0,
    MOTOR_2 = 1
} MotorId_t;

/* 细分模式 */
typedef enum {
    MICROSTEP_1   = 0,
    MICROSTEP_2   = 1,
    MICROSTEP_4   = 2,
    MICROSTEP_8   = 3,
    MICROSTEP_16  = 4
} MicrostepMode_t;

/* 电机运行标志结构体 */
typedef struct motor_flag {
    uint8_t motor1_flag;
    uint8_t motor2_flag;
} motor_flag;
extern motor_flag motor_flag1;

/* 脉冲控制结构体 */
typedef struct {
    uint32_t target_pulses;   // 目标脉冲数（0 表示连续运行）
    uint32_t current_pulses;  // 当前已发脉冲数
    uint8_t running;          // 运行标志（1:运行中, 0:停止）
} MotorPulseCtl_t;

/* 方向常量 */
#define DIR_CW    0   // 正转
#define DIR_CCW   1   // 反转

/* 公有函数声明 */
void A4988_Init(void);
void A4988_SetMicrostep(MicrostepMode_t mode);
void A4988_SetDirection(MotorId_t motor, uint8_t dir);
uint8_t A4988_GetLimitSwitch1(void);
uint8_t A4988_GetLimitSwitch2(void);
void A4988_StartMotor(MotorId_t motor);
void A4988_StartMotorPulses(MotorId_t motor, uint32_t pulses);
float A4988_MoveDistance(MotorId_t motor, float distance_cm);
void A4988_SetPulsesPerCm(float pulses_per_cm);   // 同时设置两个电机（保持兼容）
void A4988_StopMotor(MotorId_t motor);
void A4988_CheckLimitsAndStop(void);
uint8_t A4988_IsRunning(MotorId_t motor);
void A4988_res(void);   // 仅回退到物理零点（限位 + CALIB_DIST）

#endif