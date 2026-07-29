/**
 * @file    bsp_gray_sensor.h
 * @brief   八路灰度传感器驱动
 * @details 通过AD0/AD1/AD2三路GPIO选择CD4051通道，
 *          Trail_Out读取单一数字输出。
 *          当前gpio.c配置为GPIO_PULLUP → 黑线输出低电平(0)
 */

#ifndef __BSP_GRAY_SENSOR_H
#define __BSP_GRAY_SENSOR_H

#include "stdint.h"

/* ============================================================
 * 硬件连接 (已定义在 main.h / gpio.c)
 *   Trail_AD0 → PA11
 *   Trail_AD1 → PA12
 *   Trail_AD2 → PA15
 *   Trail_Out → PC10
 *
 * CD4051 通道分配
 *   CH0 = 传感器0 (最左)  A=0 B=0 C=0
 *   CH1 = 传感器1         A=1 B=0 C=0
 *   CH2 = 传感器2         A=0 B=1 C=0
 *   CH3 = 传感器3         A=1 B=1 C=0
 *   CH4 = 传感器4         A=0 B=0 C=1
 *   CH5 = 传感器5         A=1 B=0 C=1
 *   CH6 = 传感器6         A=0 B=1 C=1
 *   CH7 = 传感器7 (最右)  A=1 B=1 C=1
 * ============================================================ */

/* 灰度传感器八路数据 (0=黑线, 1=白底) */
extern uint8_t gray_data[8];

/* ============================================================
 * 函数声明
 * ============================================================ */

/**
 * @brief  读取八路灰度传感器
 * @note   自动读取所有8路并存入全局 gray_data[]
 *         当前GPIO_PULLUP → 黑线=0, 白底=1
 */
void GraySensor_Read(void);

/**
 * @brief  计算循迹偏差
 * @retval 偏差值 -7 ~ +7
 *         负数: 偏左 | 正数: 偏右 | 0: 居中/未检测
 */
int8_t GraySensor_CalculateError(void);

/**
 * @brief  获取黑线位置
 * @retval 0(最左) ~ 1000(最右), 500=中间
 */
int16_t GraySensor_GetPosition(void);

/**
 * @brief  初始化灰度传感器 (预读一次)
 */
void GraySensor_Init(void);

#endif
