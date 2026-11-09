#include "stepp.h"

/* 外部微秒延时（由用户提供） */
extern void delay_us(uint32_t us);

/*------------------ 引脚定义 ------------------*/
#define IN1_PIN   GPIO_PIN_0
#define IN1_PORT  GPIOA
#define IN2_PIN   GPIO_PIN_1
#define IN2_PORT  GPIOA
#define IN3_PIN   GPIO_PIN_4
#define IN3_PORT  GPIOA
#define IN4_PIN   GPIO_PIN_5
#define IN4_PORT  GPIOA

/*------------------ 电机状态 ------------------*/
typedef struct {
    int32_t target_pos;      // 目标步数位置
    int32_t current_pos;     // 当前步数位置
    uint8_t running;         // 运行标志（阻塞模式下仅用于查询）
    uint8_t phase_index;     // 当前相序索引 (0~7, 8=停止)
    uint8_t direction;       // 0=正转, 1=反转
} stepper_t;

static stepper_t motor = {0};

/* 相序表（四相八拍） */
static const uint8_t phase_table[8][4] = {
    {1,0,0,0},
    {1,1,0,0},
    {0,1,0,0},
    {0,1,1,0},
    {0,0,1,0},
    {0,0,1,1},
    {0,0,0,1},
    {1,0,0,1}
};

/*------------------ 私有函数 ------------------*/

// 设置四个引脚电平
static void set_phase(uint8_t idx)
{
    if (idx >= 8) {
        HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
        return;
    }
    HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, phase_table[idx][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, phase_table[idx][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, phase_table[idx][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, phase_table[idx][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// 执行一步（更新位置和相序）
static void stepper_step(void)
{
    if (!motor.running) {
        set_phase(8);
        return;
    }

    // 检查是否到达目标
    if (motor.current_pos == motor.target_pos) {
        motor.running = 0;
        set_phase(8);
        return;
    }

    // 根据方向更新相序
    if (motor.direction == 0) {   // 正转
        motor.current_pos++;
        motor.phase_index++;
        if (motor.phase_index >= 8) motor.phase_index = 0;
    } else {                      // 反转
        motor.current_pos--;
        if (motor.phase_index == 0) motor.phase_index = 8;
        motor.phase_index--;
    }
    set_phase(motor.phase_index);
}

// 阻塞式运行，直到到达目标
static void block_run(uint32_t us)
{
    while (motor.running) {
        stepper_step();
        delay_us(us);
    }
    // 停止后强制断电
    set_phase(8);
}

/*------------------ 公有函数 ------------------*/

void stepper_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin  = IN1_PIN | IN2_PIN | IN3_PIN | IN4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    motor.current_pos = 0;
    motor.target_pos = 0;
    motor.running = 0;
    motor.phase_index = 0;
    motor.direction = 0;
    set_phase(8);
}

void stepper_set_position(int32_t pos)
{
    motor.current_pos = pos;
}

int32_t stepper_get_position(void)
{
    return motor.current_pos;
}

void stepper_move_relative(int32_t steps, uint32_t delay_us)
{A
    if (steps == 0) return;

    // 设置目标位置
    motor.target_pos = motor.current_pos + steps;

    // 确定方向
    if (steps > 0) {
        motor.direction = 0;   // 正转
    } else {
        motor.direction = 1;   // 反转
    }

    motor.running = 1;
    block_run(delay_us);
}

void stepper_rotate(float angle_deg, uint32_t delay_us)
{
    if (angle_deg == 0.0f) return;
    int32_t steps = DEGREE_TO_STEP(angle_deg);
    if (steps == 0) return;
    stepper_move_relative(steps, delay_us);
}

void stepper_reset_zero(void)
{
    stepper_set_position(0);
    motor.target_pos = 0;
}

uint8_t stepper_is_running(void)
{
    return motor.running;
}