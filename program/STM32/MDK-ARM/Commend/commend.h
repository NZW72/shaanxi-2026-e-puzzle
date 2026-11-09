#ifndef __COMMEND_H
#define __COMMEND_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

extern int commend_count;
typedef struct data
{
float x;//目标的x轴位置
float y;//目标的y轴位置
float z;//目标的角度位置
float get_x;//搬动x的位置
float get_y;//搬动y的位置
}data;
extern data my_data[5];
void commend_Init(void);

#endif