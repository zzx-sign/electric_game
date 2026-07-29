#include "motor.h"
#include "tim.h"
#include "gpio.h"

Motor_t motor3 = {
    .pwm_tim = &htim1, .pwm_channel = TIM_CHANNEL_3,
    .ain1_port = AIN3_1_GPIO_Port, .ain1_pin = AIN3_1_Pin,
    .ain2_port = AIN3_2_GPIO_Port, .ain2_pin = AIN3_2_Pin,
    .enc_tim = &htim4
};
Motor_t motor4 = {
    .pwm_tim = &htim1, .pwm_channel = TIM_CHANNEL_4,
    .ain1_port = AIN4_1_GPIO_Port, .ain1_pin = AIN4_1_Pin,
    .ain2_port = AIN4_2_GPIO_Port, .ain2_pin = AIN4_2_Pin,
    .enc_tim = &htim2
};

static void SetDir(Motor_t *m, MotorDir_t d)
{
    switch (d) {
    case MOTOR_FORWARD:
        HAL_GPIO_WritePin(m->ain1_port, m->ain1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->ain2_port, m->ain2_pin, GPIO_PIN_RESET);
        break;
    case MOTOR_REVERSE:
        HAL_GPIO_WritePin(m->ain1_port, m->ain1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->ain2_port, m->ain2_pin, GPIO_PIN_SET);
        break;
    case MOTOR_STOP_BRAKE:
        HAL_GPIO_WritePin(m->ain1_port, m->ain1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->ain2_port, m->ain2_pin, GPIO_PIN_SET);
        break;
    case MOTOR_STOP_COAST:
    default:
        HAL_GPIO_WritePin(m->ain1_port, m->ain1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->ain2_port, m->ain2_pin, GPIO_PIN_RESET);
        break;
    }
}

void Motor_InitAll(void)
{
    HAL_TIM_PWM_Start(motor3.pwm_tim, motor3.pwm_channel);
    HAL_TIM_PWM_Start(motor4.pwm_tim, motor4.pwm_channel);

    HAL_TIM_Encoder_Start(motor3.enc_tim, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(motor4.enc_tim, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(motor3.enc_tim, 0);
    __HAL_TIM_SET_COUNTER(motor4.enc_tim, 0);

    Motor_StopAll();

    __HAL_TIM_MOE_ENABLE(&htim1);
}

void Motor_SetPwm(Motor_t *m, uint32_t pwm)
{
    if (pwm > 1000) pwm = 1000;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(m->pwm_tim);
    uint32_t ccr = (pwm * arr) / 1000;
    __HAL_TIM_SET_COMPARE(m->pwm_tim, m->pwm_channel, ccr);
}

void Motor_DriveAll(uint32_t pwm, MotorDir_t dir)
{
    SetDir(&motor3, dir);
    SetDir(&motor4, dir);
    Motor_SetPwm(&motor3, pwm);
    Motor_SetPwm(&motor4, pwm);
}

void Motor_StopAll(void)
{
    Motor_SetPwm(&motor3, 0);
    Motor_SetPwm(&motor4, 0);
    SetDir(&motor3, MOTOR_STOP_COAST);
    SetDir(&motor4, MOTOR_STOP_COAST);
}

/* ============================================================
 * 差分驱动：左右轮独立PWM
 *   left_pwm / right_pwm: 绝对值表示速度，负数=后退
 *   左轮: motor3 | 右轮: motor4
 * ============================================================ */
void Motor_DriveDiff(int16_t left_pwm, int16_t right_pwm)
{
    MotorDir_t ldir = (left_pwm >= 0) ? MOTOR_FORWARD : MOTOR_REVERSE;
    MotorDir_t rdir = (right_pwm >= 0) ? MOTOR_FORWARD : MOTOR_REVERSE;
    uint32_t lp = (left_pwm >= 0) ? (uint32_t)left_pwm : (uint32_t)(-left_pwm);
    uint32_t rp = (right_pwm >= 0) ? (uint32_t)right_pwm : (uint32_t)(-right_pwm);

    SetDir(&motor3, ldir);
    SetDir(&motor4, rdir);

    Motor_SetPwm(&motor3, lp);
    Motor_SetPwm(&motor4, rp);
}

void Motor_EncoderUpdate(Motor_t *m)
{
    int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(m->enc_tim);
    int16_t delta = (int16_t)(now - m->enc_last);
    m->enc_last = now;
    m->enc_total += delta;
    m->speed_pps = delta;
}

int32_t Motor_GetTotalPulses(const Motor_t *m)
{
    return m->enc_total;
}

int16_t Motor_GetSpeedPps(const Motor_t *m)
{
    return m->speed_pps;
}

void Motor_ResetAllEncoders(void)
{
    motor3.enc_total = 0;
    motor4.enc_total = 0;
    __HAL_TIM_SET_COUNTER(motor3.enc_tim, 0);
    __HAL_TIM_SET_COUNTER(motor4.enc_tim, 0);
    motor3.enc_last = 0;
    motor4.enc_last = 0;
}