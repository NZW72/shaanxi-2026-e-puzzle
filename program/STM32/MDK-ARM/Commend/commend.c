#include "commend.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#define RX_BUF_SIZE  512
int commend_count=0;//获取到的碎片数量
static uint8_t rx_buffer[RX_BUF_SIZE] = {0};

void get_data(uint16_t recv_len);

void commend_Init(void)
{
    memset(rx_buffer, 0, RX_BUF_SIZE);
    commend_count=0;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUF_SIZE);
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
						printf("进入中断\r\n");
        get_data(Size);   // 传入 Size
        // 重新启动 DMA 接收

			memset(rx_buffer, 0, RX_BUF_SIZE);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer, RX_BUF_SIZE);
    }
}

data my_data[5]={0};

void get_data(uint16_t recv_len)   // 传入实际接收长度
{
    char *buf = (char*)rx_buffer;
    int offset = 0;
    int consumed = 0;
    int id;
    data p;

    while (offset < recv_len)
    {
        // 尝试解析一条帧，格式：N%d,P1,%f,%f,%f,P2,%f,%f,F\r\n
        int ret = sscanf(buf + offset,
                         "N%d,P1,%f,%f,%f,P2,%f,%f,F\r\n%n",
                         &id, &p.x, &p.y, &p.z, &p.get_x, &p.get_y,
                         &consumed);

        if (ret == 6 && consumed > 0)   // 6个匹配项（1个int + 5个float）
        {
            if (id >= 1 && id <= 5)
            {
                my_data[id - 1] = p;
                // 打印解析结果（通过串口输出）
                char msg[80];
                snprintf(msg, sizeof(msg),
                         "Parsed N%d: x=%.1f y=%.1f z=%.1f get_x=%.1f get_y=%.1f\r\n",
                         id, p.x, p.y, p.z, p.get_x, p.get_y);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
								commend_count++;
            }
            offset += consumed;   // 移动指针到下一帧开头
        }
        else
        {
            // 解析失败，为了防止死循环，跳过1个字符尝试（或直接退出）
            // 更健壮的做法：查找下一个 'N' 字符
            char *next_n = strchr(buf + offset, 'N');
            if (next_n != NULL)
                offset = next_n - buf;
            else
                break;   // 没有更多 'N'，退出
        }
    }
		
}