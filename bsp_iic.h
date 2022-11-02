/*
 * bsp_iic.h
 *
 *  Created on: 2021年7月22日
 *      Author: YAN
 */

#ifndef BSP_LIBRARIES_API_IIC_H_
#define BSP_LIBRARIES_API_IIC_H_

#include "common.h"

typedef enum IIC       //IIC类型枚举
{
    SIMIIC,
    SCCB
} IIC_type;

void  simiic_init(void);
void  simiic_delay_set(uint16 time);
void  simiic_start(void);

void  simiic_write_reg(uint8 dev_add, uint8 reg, uint8 dat);
void simiic_write_regs(uint8 dev_add, uint8 reg, uint8 *dat, uint8 len);
uint8 simiic_read_reg(uint8 dev_add, uint8 reg, IIC_type type);
void  simiic_read_regs(uint8 dev_add, uint8 reg, uint8 *dat_add, uint8 num, IIC_type type);

#endif /* BSP_LIBRARIES_API_IIC_H_ */
