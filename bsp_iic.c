/*
 * bsp_iic.c
 *
 *  Created on: 2021年7月22日
 *      Author: YAN
 */

#include "bsp_iic.h"

#define SCL_GPIO_CLK	RCC_APB2Periph_GPIOD
#define SCL_PORT   		GPIOD
#define SCL_PIN    		GPIO_Pin_3        //定义SCL引脚  可任意更改为其他IO
#define SDA_GPIO_CLK	RCC_APB2Periph_GPIOG
#define SDA_PORT   		GPIOG
#define SDA_PIN    		GPIO_Pin_13        //定义SDA引脚  可任意更改为其他IO

#define SDA             GPIO_ReadInputDataBit(SDA_PORT, SDA_PIN)
#define SDA0()          GPIO_ResetBits(SDA_PORT, SDA_PIN)      //IO口输出低电平
#define SDA1()          GPIO_SetBits(SDA_PORT, SDA_PIN)        //IO口输出高电平
#define SCL0()          GPIO_ResetBits(SCL_PORT, SCL_PIN)      //IO口输出低电平
#define SCL1()          GPIO_SetBits(SCL_PORT, SCL_PIN)        //IO口输出高电平
#define DIR_OUT()       {SDA_PORT->CRH &= 0XFF0FFFFF; SDA_PORT->CRH |= 0X00300000;}     //SDA输出方向
#define DIR_IN()        {SDA_PORT->CRH &= 0XFF0FFFFF; SDA_PORT->CRH |= 0X00800000;}     //SDA输入方向
 
#define ack    1      //主应答
#define no_ack 0      //从应答

uint16 simiic_delay_time = 20;   //ICM等传感器应设置为20

/**
  * @brief  模拟IIC延时时间设置
  * @note   无
  * @retval 无
  */
void simiic_delay_set(uint16 time)
{
    simiic_delay_time = time;
}

/**
  * @brief  模拟IIC延时
  * @note   无
  * @retval 无
  */
void simiic_delay(void)
{
    uint16 delay_time;
    delay_time = simiic_delay_time;
    while(delay_time--);
}

//内部使用，用户无需调用
void simiic_start(void)
{
    DIR_OUT();
    SDA1();
    SCL1();
    simiic_delay();
    SDA0();
    simiic_delay();
    SCL0();
}

//内部使用，用户无需调用
void simiic_stop(void)
{
    DIR_OUT();
    SCL0();
    SDA0();
    simiic_delay();
    SCL1();
    SDA1();
    simiic_delay();
}

//内部使用，用户无需调用
void simiic_nack()
{
    SCL0();
    DIR_OUT();
    SDA1();    /* CPU驱动SDA = 1 */
    simiic_delay();
    SCL1();    /* CPU产生1个时钟 */
    simiic_delay();
    SCL0();
}

//内部使用，用户无需调用
void simiic_ack()
{
    SCL0();
    DIR_OUT();
    SDA0();    /* CPU驱动SDA = 0 */
    simiic_delay();
    SCL1();    /* CPU产生1个时钟 */
    simiic_delay();
    SCL0();
}

//内部使用，用户无需调用
uint8 simiic_waitack(void)
{
    uint8 ucErrTime = 0;

    DIR_IN();
    SDA1();         /* CPU释放SDA总线 */
    simiic_delay();
    SCL1();         /* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
    simiic_delay();
    while(SDA)        /* CPU读取SDA口线状态 */
    {
        ucErrTime++;
        if(ucErrTime > 100)
        {
            simiic_stop();
            return 1;
        }
    }
    SCL0();
    return 0;
}


//字节发送程序
//发送c(可以是数据也可是地址)，送完后接收从应答
//不考虑从应答位
//内部使用，用户无需调用
void send_ch(uint8 c)
{
    uint8 i = 0;

    DIR_OUT();
    SCL0();
    for (i = 0; i < 8; i++)
    {
        if (c & 0x80)
            SDA1();
        else
            SDA0();
        c <<= 1;
		simiic_delay();
        SCL1();
        simiic_delay();
        SCL0();
        simiic_delay();
    }
}

//字节接收程序
//接收器件传来的数据，此程序应配合|主应答函数|使用
//内部使用，用户无需调用
uint8 read_ch(uint8 ack_x)
{
    uint8 i;
    uint8 c = 0;

    DIR_IN();
    for(i = 0; i < 8; i++)
    {
        SCL0();
        simiic_delay();
        SCL1();
        c <<= 1;
        if(SDA)
            c++;
        simiic_delay();
    }
    if(!ack_x)
        simiic_nack();
    else
        simiic_ack();

    return c;
}

/**
  * @brief  模拟IIC写数据到设备寄存器函数
  * @param  dev_add         设备地址(低七位地址)
  * @param  reg             寄存器地址
  * @param  dat             写入的数据
  * @note   无
  * @retval 无
  */
void simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat)
{
    simiic_start();
    send_ch((dev_add << 1) | 0x00); //发送器件地址加写位
    simiic_waitack();
    send_ch(reg);                  //发送从机寄存器地址
    simiic_waitack();
    send_ch(dat);                  //发送需要写入的数据
    simiic_waitack();
    simiic_stop();
}

/**
  * @brief  模拟IIC写多字节数据
  * @param  dev_add         设备地址(低七位地址)
  * @param  reg             寄存器地址
  * @param  dat             数据地址指针
  * @param  len             写入字节数量
  * @note   无
  * @retval uint8           返回寄存器的数据
  */
void simiic_write_regs(uint8 dev_add, uint8 reg, uint8 *dat, uint8 len)
{
    uint8 i;

    simiic_start();
    send_ch( (dev_add << 1) | 0x00); //发送器件地址加写位
    simiic_waitack();
    send_ch( reg );                  //发送从机寄存器地址
    simiic_waitack();
    for(i = 0; i < len; i++)
    {
        send_ch(dat[i]);
        simiic_waitack();
    }
    simiic_stop();
}

/**
  * @brief  模拟IIC从设备寄存器读取数据
  * @param  dev_add         设备地址(低七位地址)
  * @param  reg             寄存器地址
  * @param  type            选择通信方式是IIC  还是 SCCB
  * @note   无
  * @retval uint8           返回寄存器的数据
  */
uint8 simiic_read_reg(uint8 dev_add, uint8 reg, IIC_type type)
{
    uint8 dat;
    simiic_start();
    send_ch((dev_add << 1) | 0x00);//发送器件地址加写位
    simiic_waitack();
    send_ch(reg);                 //发送从机寄存器地址
    simiic_waitack();
    if(type == SCCB)simiic_stop();

    simiic_start();
    send_ch((dev_add << 1) | 0x01);//发送器件地址加读位
    simiic_waitack();
    dat = read_ch(no_ack);          //读取数据
    simiic_stop();

    return dat;
}

/**
  * @brief  模拟IIC读取多字节数据
  * @param  dev_add         设备地址(低七位地址)
  * @param  reg             寄存器地址
  * @param  dat_add         数据保存的地址指针
  * @param  num             读取字节数量
  * @param  type            选择通信方式是IIC  还是 SCCB
  * @note   无
  * @retval uint8           返回寄存器的数据
  */
void simiic_read_regs(uint8 dev_add, uint8 reg, uint8 *dat_add, uint8 num, IIC_type type)
{
    simiic_start();
    send_ch((dev_add << 1) | 0x00);//发送器件地址加写位
    simiic_waitack();
    send_ch(reg);                 //发送从机寄存器地址
    simiic_waitack();
    if(type == SCCB)
        simiic_stop();

    simiic_start();
    send_ch((dev_add << 1) | 0x01);//发送器件地址加读位
    simiic_waitack();
    while(--num)
    {
        *dat_add = read_ch(ack); //读取数据
        dat_add++;
    }
    *dat_add = read_ch(no_ack); //读取数据
    simiic_stop();
}

/**
  * @brief  模拟IIC端口初始化
  * @note   无
  * @retval 无
  */
void simiic_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure; 	
	
	RCC_APB2PeriphClockCmd(SCL_GPIO_CLK | SDA_GPIO_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  SCL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(SCL_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(SDA_PORT, &GPIO_InitStructure);
    
	simiic_stop();
}
