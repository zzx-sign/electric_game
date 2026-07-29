#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

typedef struct {
    /* 目标值 */
    float target;
    /* 实际值（由调用者填充） */
    float actual;
    /* 比例/积分/微分系数 */
    float kp;
    float ki;
    float kd;
    /* 积分限幅阈值（Ki=0时自动清除积分） */
    float err_int_max;
    float err_int_threshold;
    /* 输出限幅 */
    float out_min;
    float out_max;
    /* 误差状态（ErrInt方式用） */
    float err0;
    float err1;
    float err_int;
    /* 微分方式用的旧误差 */
    float err;
    float err_last;
    float integral;
    /* 输出 */
    float output;
} PID_t;

void PID_Init(PID_t *p, float kp, float ki, float kd, float out_min, float out_max);
void PID_Clear(PID_t *p);
void PID_Update(PID_t *p);
float PID_Calc(PID_t *p, float measured, float dt);

#endif