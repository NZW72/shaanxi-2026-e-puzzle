#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "delay.h"

/*-------4线oled显示屏使用方法------
说明：共4个引脚，需接4个， GND-GND  VCC-3.3V  SCL-PB9  SDA-PB8
    显示汉字需要取模，软件：PCtoLCD2002完美版-（字符模式）；选择格式：选项——自定义格式——下拉菜单C51格式——确定
注意事项：汉字显示位置分别为0，16，32，48，64，80，96，112
     英文字符显示位置0，8，16，24，32，40，48，56，64，72，80，88，96，104，112，120
调用：

   #include "oled.h"
//----------------------------------------OLED初始化和清屏--------------------------------------

   OLED_Init();       //OLED初始化
   OLED_Clear();      //OLED清屏

//----------------------------------------OLED屏幕显示------------------------------------------

       OLED_ShowString(0,0,"tem:",16);                  //显示字符串
       OLED_ShowNum(0,2,num,3,16);                      //显示数字
       OLED_SHOWCH(0,4,"老",1)                          //显示汉字
       OLED_ShowChar(0,6,'1',16);                       //显示字符
       OLED_SHOWCHSTR(0,0,16,"评委老师好评委老师好",1);  //显示一串汉字
*/

// OLED的显存
static unsigned char OLED_buffer[1024] = {0};

void IIC_Start1()
{
    OLED_SCLK_Set();
    delay_us(1);  // 168MHz下需要更短的延时
    OLED_SDIN_Set();
    delay_us(1);
    OLED_SDIN_Clr();
    delay_us(1);
    OLED_SCLK_Clr();
    delay_us(1);
}

/**********************************************
//IIC Stop
**********************************************/
void IIC_Stop1()
{
    OLED_SCLK_Set();
    delay_us(1);
    //    OLED_SCLK_Clr();
    OLED_SDIN_Clr();
    delay_us(1);
    OLED_SDIN_Set();
    delay_us(1);
}

void IIC_Wait_Ack1()
{
    OLED_SCLK_Set();
    delay_us(1);
    OLED_SCLK_Clr();
    delay_us(1);
}

/**********************************************
// IIC Write byte
**********************************************/

void Write_IIC_Byte(unsigned char IIC_Byte)
{
    unsigned char i;
    unsigned char m, da;
    da = IIC_Byte;
    OLED_SCLK_Clr();
    delay_us(1);
    
    for (i = 0; i < 8; i++)
    {
        m = da;
        m = m & 0x80;
        if (m == 0x80)
        {
            OLED_SDIN_Set();
        }
        else
            OLED_SDIN_Clr();
        da = da << 1;
        
        delay_us(1);  // 168MHz下需要延时
        OLED_SCLK_Set();
        delay_us(1);
        OLED_SCLK_Clr();
        delay_us(1);
    }
}
/**********************************************
// IIC Write Command
**********************************************/
void Write_IIC_Command(unsigned char IIC_Command)
{
    IIC_Start1();
    Write_IIC_Byte(0x78); // Slave address,SA0=0
    IIC_Wait_Ack1();
    Write_IIC_Byte(0x00); // write command
    IIC_Wait_Ack1();
    Write_IIC_Byte(IIC_Command);
    IIC_Wait_Ack1();
    IIC_Stop1();
}
/**********************************************
// IIC Write Data
**********************************************/
void Write_IIC_Data(unsigned char IIC_Data)
{
    IIC_Start1();
    Write_IIC_Byte(0x78); // D/C#=0; R/W#=0
    IIC_Wait_Ack1();
    Write_IIC_Byte(0x40); // write data
    IIC_Wait_Ack1();
    Write_IIC_Byte(IIC_Data);
    IIC_Wait_Ack1();
    IIC_Stop1();
}
void OLED_WR_Byte(unsigned dat, unsigned cmd)
{
    if (cmd)
    {
        Write_IIC_Data(dat);
    }
    else
    {
        Write_IIC_Command(dat);
    }
}

/********************************************
// fill_Picture
********************************************/
void fill_picture(unsigned char fill_Data)
{
    unsigned char m, n;
    for (m = 0; m < 8; m++)
    {
        OLED_WR_Byte(0xb0 + m, 0); // page0-page1
        OLED_WR_Byte(0x00, 0);     // low column start address
        OLED_WR_Byte(0x10, 0);     // high column start address
        for (n = 0; n < 128; n++)
        {
            OLED_WR_Byte(fill_Data, 1);
        }
    }
}

/***********************延时函数（适用于168MHz HAL库）************************/
// 如果已经有delay.h，可以直接包含使用
// 如果没有，这里提供简单的延时函数
/*
#include "stm32f4xx_hal.h"

static void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;
    ticks = nus * 168;  // 168MHz下，1us=168个时钟周期
    told = SysTick->VAL;
    while(1)
    {
        tnow = SysTick->VAL;
        if(tnow != told)
        {
            if(tnow < told)
                tcnt += told - tnow;
            else
                tcnt += reload - tnow + told;
            told = tnow;
            if(tcnt >= ticks) break;
        }
    }
}
*/

// 坐标设置
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_WR_Byte(0xb0 + y, OLED_CMD);
    OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte((x & 0x0f), OLED_CMD);
}
// 开启OLED显示
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC命令
    OLED_WR_Byte(0X14, OLED_CMD); // DCDC ON
    OLED_WR_Byte(0XAF, OLED_CMD); // DISPLAY ON
}
// 关闭OLED显示
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD); // SET DCDC命令
    OLED_WR_Byte(0X10, OLED_CMD); // DCDC OFF
    OLED_WR_Byte(0XAE, OLED_CMD); // DISPLAY OFF
}
// 清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD); // 设置页地址（0~7）
        OLED_WR_Byte(0x00, OLED_CMD);     // 设置显示位置—列低地址
        OLED_WR_Byte(0x10, OLED_CMD);     // 设置显示位置—列高地址
        for (n = 0; n < 128; n++)
            OLED_WR_Byte(0, OLED_DATA);
    } // 更新显示
}
void OLED_On(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xb0 + i, OLED_CMD); // 设置页地址（0~7）
        OLED_WR_Byte(0x00, OLED_CMD);     // 设置显示位置—列低地址
        OLED_WR_Byte(0x10, OLED_CMD);     // 设置显示位置—列高地址
        for (n = 0; n < 128; n++)
            OLED_WR_Byte(1, OLED_DATA);
    } // 更新显示
}
// 在指定位置显示一个字符,包括部分字符
// x:0~127
// y:0~63
// mode:0,反白显示;1,正常显示
// size:选择字体 16/12
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size)
{
    unsigned char c = 0, i = 0;
    c = chr - ' '; // 得到偏移后的值
    if (x > YLevel - 1)
    {
        x = 0;
        y = y + 2;
    }
    if (Char_Size == 16)
    {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 8; i++)
            OLED_WR_Byte(F8X16[c * 16 + i], OLED_DATA);
        OLED_Set_Pos(x, y + 1);
        for (i = 0; i < 8; i++)
            OLED_WR_Byte(F8X16[c * 16 + i + 8], OLED_DATA);
    }
    else
    {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 6; i++)
            OLED_WR_Byte(F6x8[c][i], OLED_DATA);
    }
}
// m^n函数
uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--)
        result *= m;
    return result;
}
void OLED_Set_Pixel(unsigned char x, unsigned char y, unsigned char color)
{
    if (color)
    {
        OLED_buffer[(y / SIZE) * X_WIDTH + x] |= (1 << (y % SIZE)) & 0xff;
    }
    else
    {
        OLED_buffer[(y / SIZE) * X_WIDTH + x] &= ~((1 << (y % SIZE)) & 0xff);
    }
}
void OLED_Display(void)
{
    uint8_t i, n;
    for (i = 0; i < SIZE; i++)
    {
        OLED_WR_Byte(YLevel + i, OLED_CMD); // 设置页地址（0~7）
        OLED_WR_Byte(XLevelL, OLED_CMD);    // 设置显示位置—列低地址
        OLED_WR_Byte(XLevelH, OLED_CMD);    // 设置显示位置—列高地址
        for (n = 0; n < X_WIDTH; n++)
        {
            OLED_WR_Byte(OLED_buffer[i * X_WIDTH + n], OLED_DATA);
        }
    } // 更新显示
}
// 显示2个数字
// x,y :起点坐标
// len :数字的位数
// size:字体大小
// mode:模式    0,填充模式;1,叠加模式
// num:数值(0~4294967295);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size2)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (size2 / 2) * t, y, ' ', size2);
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar(x + (size2 / 2) * t, y, temp + '0', size2);
    }
}
// 显示一个字符号串
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t Char_Size)
{
    unsigned char j = 0;
    while (chr[j] != '\0')
    {
        OLED_ShowChar(x, y, chr[j], Char_Size);
        x += 8;
        if (x > 120)
        {
            x = 0;
            y += 2;
        }
        j++;
    }
}

//------------------------显示单个字----------------------
void OLED_SHOWCH(uint8_t x, uint8_t y, uint8_t *s, uint8_t mode)
{
    uint8_t i, j, k, tmp;
    uint16_t num;
    num = sizeof(cfont16) / sizeof(typFNT_GB16);
    for (i = 0; i < num; i++)
    {
        if ((cfont16[i].Index[0] == *s) && (cfont16[i].Index[1] == *(s + 1)))
        {
            for (j = 0; j < 32; j++)
            {
                if (mode)
                {
                    tmp = cfont16[i].Msk[j];
                }
                else
                {
                    tmp = ~(cfont16[i].Msk[j]);
                }
                for (k = 0; k < 8; k++)
                {
                    if (tmp & (0x80 >> k))
                    {
                        OLED_Set_Pixel(x + (j % 2) * 8 + k, y + j / 2, 1);
                    }
                    else
                    {
                        OLED_Set_Pixel(x + (j % 2) * 8 + k, y + j / 2, 0);
                    }
                }
            }
            break;
        }
    }
    OLED_Display();
}

////------------------------显示一串汉字----------------------
void OLED_SHOWCHSTR(uint8_t x, uint8_t y, uint8_t hsize, uint8_t *str, uint8_t mode)
{
    while (*str != '\0')
    {
        if (hsize == 16)
        {
            OLED_SHOWCH(x, y, str, mode);
        }
        else
        {
            return;
        }
        x += hsize;
        if (x > X_WIDTH - hsize)
        {
            x = 0;
            y += hsize;
        }
        str += 2;
    }
}

/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[])
{
    unsigned int j = 0;
    unsigned char x, y;

    if (y1 % 8 == 0)
        y = y1 / 8;
    else
        y = y1 / 8 + 1;
    for (y = y0; y < y1; y++)
    {
        OLED_Set_Pos(x0, y);
        for (x = x0; x < x1; x++)
        {
            OLED_WR_Byte(BMP[j++], OLED_DATA);
        }
    }
}

// 初始化SSD1306
void OLED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    
    // 使能GPIOG时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // 初始化GPIO引脚
    GPIO_InitStructure.Pin = GPIO_PIN_9 | GPIO_PIN_8;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 将引脚设置为高电平
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    
    // 168MHz下需要800ms延时
    HAL_Delay(800);
    
    OLED_WR_Byte(0xAE, OLED_CMD); //--display off
    OLED_WR_Byte(0x00, OLED_CMD); //---set low column address
    OLED_WR_Byte(0x10, OLED_CMD); //---set high column address
    OLED_WR_Byte(0x40, OLED_CMD); //--set start line address
    OLED_WR_Byte(0xB0, OLED_CMD); //--set page address
    OLED_WR_Byte(0x81, OLED_CMD); // contract control
    OLED_WR_Byte(0xFF, OLED_CMD); //--128
    OLED_WR_Byte(0xA1, OLED_CMD); // set segment remap
    OLED_WR_Byte(0xA6, OLED_CMD); //--normal / reverse
    OLED_WR_Byte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3F, OLED_CMD); //--1/32 duty
    OLED_WR_Byte(0xC8, OLED_CMD); // Com scan direction
    OLED_WR_Byte(0xD3, OLED_CMD); //-set display offset
    OLED_WR_Byte(0x00, OLED_CMD); //

    OLED_WR_Byte(0xD5, OLED_CMD); // set osc division
    OLED_WR_Byte(0x80, OLED_CMD); //

    OLED_WR_Byte(0xD8, OLED_CMD); // set area color mode off
    OLED_WR_Byte(0x05, OLED_CMD); //

    OLED_WR_Byte(0xD9, OLED_CMD); // Set Pre-Charge Period
    OLED_WR_Byte(0xF1, OLED_CMD); //

    OLED_WR_Byte(0xDA, OLED_CMD); // set com pin configuartion
    OLED_WR_Byte(0x12, OLED_CMD); //

    OLED_WR_Byte(0xDB, OLED_CMD); // set Vcomh
    OLED_WR_Byte(0x30, OLED_CMD); //

    OLED_WR_Byte(0x8D, OLED_CMD); // set charge pump enable
    OLED_WR_Byte(0x14, OLED_CMD); //

    OLED_WR_Byte(0xAF, OLED_CMD); //--turn on oled panel
    
    OLED_Clear();
}