#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>
#include "main.h"

typedef enum {
    MOTOR_FORWARD = 0,
    MOTOR_REVERSE = 1,
    MOTOR_STOP_BRAKE = 2,
    MOTOR_STOP_COAST = 3
} MotorDir_t;

typedef struct {
    TIM_HandleTypeDef *pwm_tim;
    uint16_t pwm_channel;
    GPIO_TypeDef *ain1_port;
    uint16_t ain1_pin;
    GPIO_TypeDef *ain2_port;
    uint16_t ain2_pin;
    TIM_HandleTypeDef *enc_tim;
    int16_t enc_last;
    int32_t enc_total;
    int16_t speed_pps;   /* pulses per 10ms */
} Motor_t;

void Motor_InitAll(void);
void Motor_SetPwm(Motor_t *m, uint32_t pwm);
void Motor_DriveAll(uint32_t pwm, MotorDir_t dir);
void Motor_DriveDiff(int16_t left_pwm, int16_t right_pwm);
void Motor_StopAll(void);
void Motor_EncoderUpdate(Motor_t *m);
int32_t Motor_GetTotalPulses(const Motor_t *m);
int16_t Motor_GetSpeedPps(const Motor_t *m);
void Motor_ResetAllEncoders(void);

extern Motor_t motor3;
extern Motor_t motor4;

#endif