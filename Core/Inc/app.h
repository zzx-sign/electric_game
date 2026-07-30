#ifndef __APP_H__
#define __APP_H__

#include <stdint.h>

typedef enum {
    APP_STATE_IDLE = 0,
    APP_STATE_RUNNING,
    APP_STATE_DONE
} AppState_t;

typedef enum {
    APP_MODE_KEY2 = 0,   /* key_2：60 pps 循迹，丢线停车 */
    APP_MODE_KEY3,       /* key_3：20 pps 循迹，8s 自动停，关闭丢线停车 */
    APP_MODE_KEY4,       /* key_4：25 pps 循迹，30s 自动停，关闭丢线停车 */
} AppMode_t;

void App_Init(void);
void App_ControlLoop(void);
void App_KeyScan(void);
void Cascade_Update(void);

extern volatile AppState_t g_app_state;
extern volatile AppMode_t   g_app_mode;
extern volatile int8_t  g_track_error;
extern volatile uint32_t g_run_start_tick;  /* 本次运行的启动时刻（HAL_GetTick ms） */
extern volatile uint32_t g_run_stop_tick;   /* 触发丢线停车 / 8s 到点的时刻；0 表示未结束 */
extern volatile int8_t   g_run_had_finish;  /* 是否已结束（用于 main 显示时间） */
extern volatile int8_t   g_run_started;     /* 是否曾经启动过（用于 main 区分"还没跑过"） */
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
