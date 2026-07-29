#ifndef __APP_H__
#define __APP_H__

#include <stdint.h>

typedef enum {
    APP_STATE_IDLE = 0,
    APP_STATE_RUNNING,
    APP_STATE_DONE
} AppState_t;

void App_Init(void);
void App_ControlLoop(void);
void App_KeyScan(void);
void Cascade_Update(void);

extern volatile AppState_t g_app_state;
extern volatile int8_t g_track_error;
extern volatile float g_left_speed_mmps;
extern volatile float g_right_speed_mmps;
extern volatile int16_t g_left_pwm;
extern volatile int16_t g_right_pwm;
extern volatile uint8_t g_speed_ctrl_enable;

extern volatile float g_m3_speed_mmps;
extern volatile float g_m4_speed_mmps;

extern volatile int16_t g_m3_speed_pps;
extern volatile int16_t g_m4_speed_pps;

#endif
