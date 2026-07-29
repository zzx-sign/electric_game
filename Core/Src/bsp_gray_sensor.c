/**
 * @file    bsp_gray_sensor.c
 * @brief   八路灰度传感器驱动
 * @details 通过AD0/AD1/AD2三路GPIO选择CD4051通道，
 *          Trail_Out读取单一数字输出。
 *
 * GPIO配置约定 (gpio.c):
 *   Trail_Out → GPIO_PULLUP
 *   → 黑线输出低电平(0), 白底输出高电平(1)
 *
 *   gray_data[i] = 1 表示检测到白底 (未压线)
 *   gray_data[i] = 0 表示检测到黑线 (已压线)
 */

#include "bsp_gray_sensor.h"
#include "main.h"

/* ============================================================
 * 全局数据
 * ============================================================ */
uint8_t gray_data[8] = {0};

/* ============================================================
 * 硬件层: 通道切换
 * ============================================================ */
#define SEL_C(channel)  (((channel) >> 2) & 0x01)   /* AD2 → PC15 */
#define SEL_B(channel)  (((channel) >> 1) & 0x01)   /* AD1 → PA12 */
#define SEL_A(channel)  ( (channel)       & 0x01)   /* AD0 → PA11 */

static void SelectChannel(uint8_t channel)
{
    HAL_GPIO_WritePin(Trail_AD2_GPIO_Port, Trail_AD2_Pin,
                      SEL_C(channel) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Trail_AD1_GPIO_Port, Trail_AD1_Pin,
                      SEL_B(channel) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Trail_AD0_GPIO_Port, Trail_AD0_Pin,
                      SEL_A(channel) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ============================================================
 * 延时: 简单C循环，CD4051切换时间 < 1us，10us足够
 *       在168MHz主频下 500次循环约10us
 * ============================================================ */
static void Delay_US(uint32_t us)
{
    volatile uint32_t cnt = us * 50;  /* 粗略估算: 168MHz/50 ≈ 3.36MHz */
    while (cnt--) { __NOP(); }
}

/* ============================================================
 * 公开API
 * ============================================================ */

/**
 * @brief  读取八路灰度传感器
 * @note   存入全局 gray_data[]
 *         读取引脚实际电平 (PULLUP → 黑=0, 白=1)
 */
void GraySensor_Read(void)
{
    for (uint8_t ch = 0; ch < 8; ch++) {
        SelectChannel(ch);
        Delay_US(10);

        /* 直接存 PinState，黑线=0(PULLUP下拉低), 白底=1 */
        gray_data[ch] = (HAL_GPIO_ReadPin(Trail_Out_GPIO_Port, Trail_Out_Pin)
                         == GPIO_PIN_SET) ? 1U : 0U;
    }
}

/**
 * @brief  计算循迹偏差
 * @retval 偏差 -7 ~ +7
 *         负数=偏左, 正数=偏右, 0=居中/未检测
 *
 * 权重策略:
 *   传感器索引 0~3 (左侧) → 权重 -4 -3 -2 -1
 *   传感器索引 4~7 (右侧) → 权重  1  2  3  4
 *
 *   使用加权平均，保证返回值始终在 [-7, +7] 范围内
 */
int8_t GraySensor_CalculateError(void)
{
    /* 权重表，对应索引 0~7 */
    static const int8_t W[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

    int16_t weighted_sum = 0;
    int16_t sensor_count = 0;

    for (uint8_t i = 0; i < 8; i++) {
        /* gray_data[i] == 0 表示该路检测到黑线 */
        if (gray_data[i] == 0) {
            weighted_sum += W[i];
            sensor_count++;
        }
    }

    if (sensor_count == 0) {
        return 0;  /* 未检测到黑线，维持当前方向 */
    }

    return (int8_t)(weighted_sum / sensor_count);
}

/**
 * @brief  获取黑线位置百分比
 * @retval 0(最左) ~ 1000(最右), 500=中间
 *
 * 加权位置表:
 *   索引:  0     1     2     3     4     5     6     7
 *   位置: 0    143   286   429   571   714   857  1000
 */
int16_t GraySensor_GetPosition(void)
{
    static const uint16_t POS_W[8] = {0, 143, 286, 429, 571, 714, 857, 1000};

    int32_t weighted_sum = 0;
    int32_t sensor_count = 0;

    for (uint8_t i = 0; i < 8; i++) {
        if (gray_data[i] == 0) {
            weighted_sum += POS_W[i];
            sensor_count++;
        }
    }

    if (sensor_count == 0) {
        return 500;  /* 默认返回中间位置 */
    }

    return (int16_t)(weighted_sum / sensor_count);
}

/**
 * @brief  初始化灰度传感器
 * @note   AD通道在 gpio.c 已配置完毕，这里预读一次数据
 */
void GraySensor_Init(void)
{
    GraySensor_Read();  /* 预读一次，消除上电首次读数不稳定 */
}
