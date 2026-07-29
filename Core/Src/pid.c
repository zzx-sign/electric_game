#include "pid.h"

void PID_Init(PID_t *p, float kp, float ki, float kd, float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_min = out_min;
    p->out_max = out_max;
    p->target = 0.0f;
    p->actual = 0.0f;
    p->err0 = 0.0f;
    p->err1 = 0.0f;
    p->err_int = 0.0f;
    p->err = 0.0f;
    p->err_last = 0.0f;
    p->integral = 0.0f;
    p->output = 0.0f;
    p->err_int_max = 1000.0f;
    p->err_int_threshold = 0.0f;
}

void PID_Clear(PID_t *p)
{
    p->err0 = 0.0f;
    p->err1 = 0.0f;
    p->err_int = 0.0f;
    p->output = 0.0f;
}

/* ============================================================
 * ErrInt方式PID（无dt参数，适合固定周期调用）
 *   err_int_threshold > 0 时，只在误差小于阈值时积分
 * ============================================================ */
void PID_Update(PID_t *p)
{
    p->err1 = p->err0;
    p->err0 = p->target - p->actual;

    if (p->ki != 0.0f) {
        if (p->err_int_threshold == 0.0f) {
            p->err_int += p->err0;
        } else {
            float abs_err = (p->err0 >= 0.0f) ? p->err0 : -p->err0;
            if (abs_err < p->err_int_threshold) {
                p->err_int += p->err0;
            } else {
                p->err_int = 0.0f;
            }
        }
        if (p->err_int > p->err_int_max) p->err_int = p->err_int_max;
        if (p->err_int < -p->err_int_max) p->err_int = -p->err_int_max;
    } else {
        p->err_int = 0.0f;
    }

    p->output = p->kp * p->err0
              + p->ki * p->err_int
              + p->kd * (p->err0 - p->err1);

    if (p->output > p->out_max) p->output = p->out_max;
    if (p->output < p->out_min) p->output = p->out_min;
}

/* ============================================================
 * 微分先行PID（保留原有用法，带dt参数）
 * ============================================================ */
float PID_Calc(PID_t *p, float measured, float dt)
{
    if (dt <= 0.0f) dt = 0.001f;
    p->err = p->target - measured;

    p->integral += p->err * dt;
    if (p->integral > p->out_max) p->integral = p->out_max;
    if (p->integral < p->out_min) p->integral = p->out_min;

    float derivative = (p->err - p->err_last) / dt;
    p->err_last = p->err;

    p->output = p->kp * p->err + p->ki * p->integral + p->kd * derivative;
    if (p->output > p->out_max) p->output = p->out_max;
    if (p->output < p->out_min) p->output = p->out_min;

    return p->output;
}
