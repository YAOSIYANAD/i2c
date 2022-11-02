/*
 * bsp_iic.c
 *
 *  Created on: 2021��7��22��
 *      Author: YAN
 */

#include "bsp_iic.h"

#define SCL_GPIO_CLK	RCC_APB2Periph_GPIOD
#define SCL_PORT   		GPIOD
#define SCL_PIN    		GPIO_Pin_3        //����SCL����  ���������Ϊ����IO
#define SDA_GPIO_CLK	RCC_APB2Periph_GPIOG
#define SDA_PORT   		GPIOG
#define SDA_PIN    		GPIO_Pin_13        //����SDA����  ���������Ϊ����IO

#define SDA             GPIO_ReadInputDataBit(SDA_PORT, SDA_PIN)
#define SDA0()          GPIO_ResetBits(SDA_PORT, SDA_PIN)      //IO������͵�ƽ
#define SDA1()          GPIO_SetBits(SDA_PORT, SDA_PIN)        //IO������ߵ�ƽ
#define SCL0()          GPIO_ResetBits(SCL_PORT, SCL_PIN)      //IO������͵�ƽ
#define SCL1()          GPIO_SetBits(SCL_PORT, SCL_PIN)        //IO������ߵ�ƽ
#define DIR_OUT()       {SDA_PORT->CRH &= 0XFF0FFFFF; SDA_PORT->CRH |= 0X00300000;}     //SDA�������
#define DIR_IN()        {SDA_PORT->CRH &= 0XFF0FFFFF; SDA_PORT->CRH |= 0X00800000;}     //SDA���뷽��
 
#define ack    1      //��Ӧ��
#define no_ack 0      //��Ӧ��

uint16 simiic_delay_time = 20;   //ICM�ȴ�����Ӧ����Ϊ20

/**
  * @brief  ģ��IIC��ʱʱ������
  * @note   ��
  * @retval ��
  */
void simiic_delay_set(uint16 time)
{
    simiic_delay_time = time;
}

/**
  * @brief  ģ��IIC��ʱ
  * @note   ��
  * @retval ��
  */
void simiic_delay(void)
{
    uint16 delay_time;
    delay_time = simiic_delay_time;
    while(delay_time--);
}

//�ڲ�ʹ�ã��û��������
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

//�ڲ�ʹ�ã��û��������
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

//�ڲ�ʹ�ã��û��������
void simiic_nack()
{
    SCL0();
    DIR_OUT();
    SDA1();    /* CPU����SDA = 1 */
    simiic_delay();
    SCL1();    /* CPU����1��ʱ�� */
    simiic_delay();
    SCL0();
}

//�ڲ�ʹ�ã��û��������
void simiic_ack()
{
    SCL0();
    DIR_OUT();
    SDA0();    /* CPU����SDA = 0 */
    simiic_delay();
    SCL1();    /* CPU����1��ʱ�� */
    simiic_delay();
    SCL0();
}

//�ڲ�ʹ�ã��û��������
uint8 simiic_waitack(void)
{
    uint8 ucErrTime = 0;

    DIR_IN();
    SDA1();         /* CPU�ͷ�SDA���� */
    simiic_delay();
    SCL1();         /* CPU����SCL = 1, ��ʱ�����᷵��ACKӦ�� */
    simiic_delay();
    while(SDA)        /* CPU��ȡSDA����״̬ */
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


//�ֽڷ��ͳ���
//����c(����������Ҳ���ǵ�ַ)���������մ�Ӧ��
//�����Ǵ�Ӧ��λ
//�ڲ�ʹ�ã��û��������
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

//�ֽڽ��ճ���
//�����������������ݣ��˳���Ӧ���|��Ӧ����|ʹ��
//�ڲ�ʹ�ã��û��������
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
  * @brief  ģ��IICд���ݵ��豸�Ĵ�������
  * @param  dev_add         �豸��ַ(����λ��ַ)
  * @param  reg             �Ĵ�����ַ
  * @param  dat             д�������
  * @note   ��
  * @retval ��
  */
void simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat)
{
    simiic_start();
    send_ch((dev_add << 1) | 0x00); //����������ַ��дλ
    simiic_waitack();
    send_ch(reg);                  //���ʹӻ��Ĵ�����ַ
    simiic_waitack();
    send_ch(dat);                  //������Ҫд�������
    simiic_waitack();
    simiic_stop();
}

/**
  * @brief  ģ��IICд���ֽ�����
  * @param  dev_add         �豸��ַ(����λ��ַ)
  * @param  reg             �Ĵ�����ַ
  * @param  dat             ���ݵ�ַָ��
  * @param  len             д���ֽ�����
  * @note   ��
  * @retval uint8           ���ؼĴ���������
  */
void simiic_write_regs(uint8 dev_add, uint8 reg, uint8 *dat, uint8 len)
{
    uint8 i;

    simiic_start();
    send_ch( (dev_add << 1) | 0x00); //����������ַ��дλ
    simiic_waitack();
    send_ch( reg );                  //���ʹӻ��Ĵ�����ַ
    simiic_waitack();
    for(i = 0; i < len; i++)
    {
        send_ch(dat[i]);
        simiic_waitack();
    }
    simiic_stop();
}

/**
  * @brief  ģ��IIC���豸�Ĵ�����ȡ����
  * @param  dev_add         �豸��ַ(����λ��ַ)
  * @param  reg             �Ĵ�����ַ
  * @param  type            ѡ��ͨ�ŷ�ʽ��IIC  ���� SCCB
  * @note   ��
  * @retval uint8           ���ؼĴ���������
  */
uint8 simiic_read_reg(uint8 dev_add, uint8 reg, IIC_type type)
{
    uint8 dat;
    simiic_start();
    send_ch((dev_add << 1) | 0x00);//����������ַ��дλ
    simiic_waitack();
    send_ch(reg);                 //���ʹӻ��Ĵ�����ַ
    simiic_waitack();
    if(type == SCCB)simiic_stop();

    simiic_start();
    send_ch((dev_add << 1) | 0x01);//����������ַ�Ӷ�λ
    simiic_waitack();
    dat = read_ch(no_ack);          //��ȡ����
    simiic_stop();

    return dat;
}

/**
  * @brief  ģ��IIC��ȡ���ֽ�����
  * @param  dev_add         �豸��ַ(����λ��ַ)
  * @param  reg             �Ĵ�����ַ
  * @param  dat_add         ���ݱ���ĵ�ַָ��
  * @param  num             ��ȡ�ֽ�����
  * @param  type            ѡ��ͨ�ŷ�ʽ��IIC  ���� SCCB
  * @note   ��
  * @retval uint8           ���ؼĴ���������
  */
void simiic_read_regs(uint8 dev_add, uint8 reg, uint8 *dat_add, uint8 num, IIC_type type)
{
    simiic_start();
    send_ch((dev_add << 1) | 0x00);//����������ַ��дλ
    simiic_waitack();
    send_ch(reg);                 //���ʹӻ��Ĵ�����ַ
    simiic_waitack();
    if(type == SCCB)
        simiic_stop();

    simiic_start();
    send_ch((dev_add << 1) | 0x01);//����������ַ�Ӷ�λ
    simiic_waitack();
    while(--num)
    {
        *dat_add = read_ch(ack); //��ȡ����
        dat_add++;
    }
    *dat_add = read_ch(no_ack); //��ȡ����
    simiic_stop();
}

/**
  * @brief  ģ��IIC�˿ڳ�ʼ��
  * @note   ��
  * @retval ��
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
