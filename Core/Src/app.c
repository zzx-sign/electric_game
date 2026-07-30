#include "app.h"
#include "main.h"
#include "motor.h"
#include "pid.h"
#include "oled.h"
#include "gpio.h"
#include "bsp_gray_sensor.h"
#include "usart.h"

#define CTRL_PERIOD_S       0.01f

 /* ============================================================ */
#define BASE_SPEED_PULSE    60

/* ============================================================
 * 速度PID参数（内环）- 使用TI参考值
 * ============================================================ */
#define SPEED_KP            20.0f
#define SPEED_KI            2.0f
#define SPEED_KD            2.0f
#define SPEED_ERRINT_MAX    500.0f
#define SPEED_ERRINT_THRESH 100.0f
#define SPEED_OUT_MAX       1000.0f

/* ============================================================
 * 转向PID参数（外环）
 * ============================================================ */
#define STEER_KP            3.8f
#define STEER_KI            0.06f  
#define STEER_KD            0.0f
#define STEER_ERRINT_MAX    100.0f
#define STEER_ERRINT_THRESH 3.0f
#define STEER_OUT_MAX       50.0f
#define STEER_OUT_MIN       (-50.0f)

/* ============================================================
 * 灰度传感器加权权重（-7~+7）
 * 中间传感器(3,4)权重为0，避免死区
 * ============================================================ */
static const int8_t TRACK_WEIGHT[8] = {-3, -2, -1, 0, 0, 1, 2, 3};

/* ============================================================
 * 转向死区阈值：误差小于此值时不转向（已弃用，改用渐进式差速）
 * ============================================================ */
#define STEER_DEAD_ZONE      0

/* ============================================================
 * 渐进式差速系数：0.5~1.0，越小转向越温和
 * ============================================================ */
#define STEER_DIFF_RATIO     0.6f

/* ============================================================
 * 丢线停车阈值：检测到白线的传感器数量 >= 此值时停车
 * 8 路总览，>=4 代表已经有一半传感器识别到白底，通常意味着严重偏离赛道
 * ============================================================ */
#define LOOSE_LINE_WHITE_THRESHOLD  4

/* ============================================================
 * key_3 模式：8 秒自动停车（HAL_GetTick 单位 ms）
 * key_4 模式：30 秒自动停车
 * 两者都关闭丢线停车。
 * ============================================================ */
#define APP_KEY3_RUN_DURATION_MS    8000U
#define APP_KEY4_RUN_DURATION_MS    30000U

/* 0 表示"无定时停车限制"（key_2 模式），非 0 表示达到该毫秒数自动停车。 */
static const uint32_t APP_NO_TIMEOUT_MS = 0U;

/* ============================================================
 * 运行模式参数（profile）
 * - base_speed：阶段一目标脉冲/控制周期（key_2 默认前 15s 全速）
 * - phase2_switch_ms：进入阶段二的时刻（HAL_GetTick 相对 run_start_tick 的 ms），
 *                    0 = 不切换。key_2 模式配置为 15000。
 * - phase2_speed：阶段二目标脉冲/控制周期。key_2 配置为 20。
 * - stop_white_threshold：丢线停车阈值（key_3 / key_4 模式不使用）
 * - run_duration_ms：自动停车时间（HAL_GetTick ms），0 = 不限时
 * - steer_diff_ratio：差速折扣
 * - speed_kp/ki/kd：内环速度 PID 参数（阶段一）
 * - phase2_speed_kp/ki/kd：内环速度 PID 参数（阶段二）
 * - steer_kp/ki：外环（转向）PID 参数（阶段一）
 * - phase2_steer_kp/ki：外环（转向）PID 参数（阶段二）
 * ============================================================ */
typedef struct {
    int16_t base_speed;
    uint16_t phase2_switch_ms;
    int16_t phase2_speed;
    uint8_t stop_white_threshold;
    uint32_t run_duration_ms;
    float   steer_diff_ratio;
    float   speed_kp;
    float   speed_ki;
    float   speed_kd;
    float   steer_kp;
    float   steer_ki;
    float   phase2_speed_kp;
    float   phase2_speed_ki;
    float   phase2_speed_kd;
    float   phase2_steer_kp;
    float   phase2_steer_ki;
} AppProfile_t;

/* key_2：前 15s 全速 50 pps，之后切到 20 pps；丢线停车，无限时 */
static const AppProfile_t PROFILE_KEY2 = {
    .base_speed          = 50,
    .phase2_switch_ms    = 15000,
    .phase2_speed        = 20,
    .stop_white_threshold = 4,
    .run_duration_ms     = APP_NO_TIMEOUT_MS,
    .steer_diff_ratio    = 0.6f,
    /* 阶段一（50 pps）速度环 */
    .speed_kp            = SPEED_KP,
    .speed_ki            = SPEED_KI,
    .speed_kd            = SPEED_KD,
    /* 阶段一（50 pps）转向环 */
    .steer_kp            = 5.5f,
    .steer_ki            = 0.1f,
    /* 阶段二（20 pps）速度环：低速适当降低增益 */
    .phase2_speed_kp     = 7.0f,
    .phase2_speed_ki     = 0.5f,
    .phase2_speed_kd     = 1.0f,
    /* 阶段二（20 pps）转向环：降低增益减少过冲 */
    .phase2_steer_kp     = 4.5f,
    .phase2_steer_ki     = 0.08f,
};

/* key_3：25 pps，关闭丢线停车，8 秒后自动停 */
static const AppProfile_t PROFILE_KEY3 = {
    .base_speed          = 25,
    .stop_white_threshold = 0,            /* 0 = 禁用丢线停车 */
    .run_duration_ms     = APP_KEY3_RUN_DURATION_MS,
    .steer_diff_ratio    = 0.6f,          /* 慢速，沿用 0.6 方便对比调试 */
    .speed_kp            = SPEED_KP,
    .speed_ki            = SPEED_KI,
    .speed_kd            = SPEED_KD,
    .steer_kp            = 3.5f,
    .steer_ki            = 0.06f,
};

/* key_4：29 pps，关闭丢线停车，30 秒后自动停 */
static const AppProfile_t PROFILE_KEY4 = {
    .base_speed          = 29,
    .stop_white_threshold = 4,            /* 0 = 禁用丢线停车 */
    .run_duration_ms     = APP_KEY4_RUN_DURATION_MS,
    .steer_diff_ratio    = 0.5f,          /* 与 key_3 同设置，需要时再单独调 */
    .speed_kp            = SPEED_KP,
    .speed_ki            = SPEED_KI,
    .speed_kd            = SPEED_KD,
    .steer_kp            = 3.8f,
    .steer_ki            = 0.06f,
};

/* ============================================================
 * PID实例
 * ============================================================ */
static PID_t pid_L = {
    .target = 0.0f,
    .kp = SPEED_KP, .ki = SPEED_KI, .kd = SPEED_KD,
    .err_int_max = SPEED_ERRINT_MAX,
    .err_int_threshold = SPEED_ERRINT_THRESH,
    .out_max = SPEED_OUT_MAX, .out_min = 0.0f,
};
static PID_t pid_R = {
    .target = 0.0f,
    .kp = SPEED_KP, .ki = SPEED_KI, .kd = SPEED_KD,
    .err_int_max = SPEED_ERRINT_MAX,
    .err_int_threshold = SPEED_ERRINT_THRESH,
    .out_max = SPEED_OUT_MAX, .out_min = 0.0f,
};
static PID_t pid_Steer = {
    .target = 0.0f,
    .kp = STEER_KP, .ki = STEER_KI, .kd = STEER_KD,
    .err_int_max = STEER_ERRINT_MAX,
    .err_int_threshold = STEER_ERRINT_THRESH,
    .out_max = STEER_OUT_MAX, .out_min = STEER_OUT_MIN,
};

/* ============================================================
 * 全局状态
 * ============================================================ */
volatile AppState_t  g_app_state = APP_STATE_IDLE;
volatile AppMode_t    g_app_mode  = APP_MODE_KEY2;
static uint8_t stopped_by_sensor = 0;  /* 标记是否由传感器触发停车（按下按钮即可恢复运行） */
volatile int8_t      g_track_error = 0;
volatile float       g_left_speed_mmps = 0.0f;
volatile float       g_right_speed_mmps = 0.0f;
volatile int16_t     g_left_pwm = 0;
volatile int16_t     g_right_pwm = 0;
volatile uint8_t     g_speed_ctrl_enable = 0;
volatile float       g_m3_speed_mmps = 0.0f;
volatile float       g_m4_speed_mmps = 0.0f;
volatile int16_t     g_m3_speed_pps = 0;
volatile int16_t     g_m4_speed_pps = 0;

static uint8_t key_prev = 0;
static uint8_t key_cnt = 0;
static uint8_t key_confirmed = 0;  /* 消抖确认后的状态 */
static uint8_t key3_prev = 0;
static uint8_t key3_cnt = 0;
static uint8_t key3_confirmed = 0;
static uint8_t key4_prev = 0;
static uint8_t key4_cnt = 0;
static uint8_t key4_confirmed = 0;
#define KEY_DEBOUNCE_CNT  5  /* 连续5次检测到相同状态才确认（约50ms消抖）*/
static int16_t last_err = 0;
static uint8_t track_detected_prev = 0;

volatile uint32_t g_run_start_tick   = 0;  /* 按下按键启动的时刻（HAL_GetTick ms） */
volatile uint32_t g_run_stop_tick    = 0;  /* 触发丢线停车 / 8s 到点的时刻；为 0 表示还在运行 */
volatile int8_t   g_run_had_finish   = 0;  /* 是否已经触发过丢线停车（用于 main 显示时间） */
volatile int8_t   g_run_started      = 0;  /* 是否已经开始过运行（用于 main 是否进入计时） */
static uint32_t run_start_tick = 0;

/* 当前生效的运行参数 */
static const AppProfile_t *g_profile = &PROFILE_KEY2;
/* 两阶段速度切换标志：到达 phase2_switch_ms 后置 1，Start_Run 复位为 0 */
static uint8_t g_profile_stage2_active = 0;

/* 重新载入 PID 参数与积分（模式切换 / 启动时调用） */
static void Apply_Profile(const AppProfile_t *p)
{
    g_profile = p;
    pid_L.kp = p->speed_kp;
    pid_L.ki = p->speed_ki;
    pid_L.kd = p->speed_kd;
    pid_R.kp = p->speed_kp;
    pid_R.ki = p->speed_ki;
    pid_R.kd = p->speed_kd;
}

/* ============================================================
 * VOFA+ 串口输出
 * ============================================================ */
#define UART_TX_BUF_LEN  160
static uint8_t  uart_tx_buf[UART_TX_BUF_LEN];
volatile uint8_t uart_tx_busy = 0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
    uart_tx_busy = 0;
}

static int append_fixed2(char *buf, int pos, int end, float v)
{
    if (v < -100000.0f) v = -100000.0f;
    if (v >  100000.0f) v =  100000.0f;
    int int_part = (int)v;
    int frac_part;
    if (v >= 0.0f) {
        frac_part = (int)((v - int_part) * 100.0f + 0.5f);
        if (frac_part >= 100) { int_part += 1; frac_part -= 100; }
    } else {
        frac_part = (int)((-(v - int_part)) * 100.0f + 0.5f);
        if (frac_part >= 100) { int_part += 1; frac_part -= 100; }
    }
    char tmp[16];
    int len = 0;
    if (int_part == 0 && v < 0.0f) { tmp[len++] = '-'; }
    int ip = (int_part < 0) ? -int_part : int_part;
    if (ip == 0) { tmp[len++] = '0'; }
    while (ip > 0) { tmp[len++] = (char)('0' + ip % 10); ip /= 10; }
    if (int_part < 0 && v >= 0.0f) { tmp[len++] = '-'; }
    for (int i = 0; i < len; i++) buf[pos++] = tmp[len - 1 - i];
    buf[pos++] = '.';
    buf[pos++] = (char)('0' + (frac_part / 10) % 10);
    buf[pos++] = (char)('0' + frac_part % 10);
    return pos;
}

static void Vofa_Send(float t1, float a1, float t3, float a3, float t2, float a2, float t4, float a4)
{
    if (uart_tx_busy) return;
    int pos = 0;
    float vals[8] = { t1, a1, t3, a3, t2, a2, t4, a4 };
    for (int i = 0; i < 8; i++) {
        pos = append_fixed2((char*)uart_tx_buf, pos, UART_TX_BUF_LEN, vals[i]);
        if (pos + 2 >= UART_TX_BUF_LEN) return;
        if (i < 7) uart_tx_buf[pos++] = ',';
    }
    if (pos + 1 >= UART_TX_BUF_LEN) return;
    uart_tx_buf[pos++] = '\n';
    if (HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, (uint16_t)pos) == HAL_OK)
        uart_tx_busy = 1;
}

/* ============================================================
 * 计算转向偏差（加权平均，-7~+7）
 * ============================================================ */
static int8_t Calc_TrackError(void)
{
    GraySensor_Read();
    int16_t weighted_sum = 0;
    int16_t count = 0;
    for (int i = 0; i < 8; i++) {
        if (gray_data[i] != 0) {
            weighted_sum += TRACK_WEIGHT[i];
            count++;
        }
    }
    if (count == 0) return 0;
    last_err = (int16_t)(weighted_sum / count);
    return (int8_t)last_err;
}

/* ============================================================
 * 初始化
 * ============================================================ */
void App_Init(void)
{
    Motor_InitAll();
    Motor_ResetAllEncoders();
    GraySensor_Init();
    PID_Clear(&pid_L);
    PID_Clear(&pid_R);
    PID_Clear(&pid_Steer);
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "READY", OLED_8X16);
    OLED_Update();
    Motor_StopAll();
    g_app_state = APP_STATE_IDLE;
    g_app_mode  = APP_MODE_KEY2;
    g_profile   = &PROFILE_KEY2;
    Apply_Profile(g_profile);
}

/* 启动运行（在 IDLE 或 stopped_by_sensor 状态下调用） */
static void Start_Run(void)
{
    Motor_ResetAllEncoders();
    PID_Clear(&pid_L);
    PID_Clear(&pid_R);
    PID_Clear(&pid_Steer);
    Apply_Profile(g_profile);   /* 用当前模式的阶段一速度环 PID 参数 */
    /* 阶段一切换时会把 steer 放到 Cascade_Update，这里统一补一次 */
    pid_Steer.kp = g_profile->steer_kp;
    pid_Steer.ki = g_profile->steer_ki;
    pid_Steer.kd = 0.0f;
    g_left_speed_mmps  = 0.0f;
    g_right_speed_mmps = 0.0f;
    g_left_pwm  = 0;
    g_right_pwm = 0;
    g_speed_ctrl_enable = 1;
    g_app_state = APP_STATE_RUNNING;
    stopped_by_sensor = 0;
    uint32_t now = HAL_GetTick();
    run_start_tick = now;
    g_run_start_tick = now;
    g_run_stop_tick  = 0;
    g_run_had_finish = 0;
    g_run_started    = 1;
    g_profile_stage2_active = 0;  /* 重新进入阶段一 */
    track_detected_prev = 0;
}

void App_KeyScan(void)
{
    /* ---- key_2 ---- */
    uint8_t now = (HAL_GPIO_ReadPin(key_2_GPIO_Port, key_2_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    if (now == key_prev) {
        key_cnt++;
        if (key_cnt >= KEY_DEBOUNCE_CNT) {
            key_cnt = KEY_DEBOUNCE_CNT;
            if (key_confirmed != now) {
                key_confirmed = now;
                if (now == 1) {
                    /* key_2：选择 key_2 模式 */
                    g_app_mode = APP_MODE_KEY2;
                    Apply_Profile(&PROFILE_KEY2);

                    if (stopped_by_sensor) {
                        /* 由传感器触发停车后，按一次按钮即恢复运行 */
                        stopped_by_sensor = 0;
                        Start_Run();
                    } else if (g_app_state == APP_STATE_IDLE) {
                        Start_Run();
                    } else if (g_app_state == APP_STATE_RUNNING) {
                        g_speed_ctrl_enable = 0;
                        g_app_state = APP_STATE_IDLE;
                        Motor_StopAll();
                    }
                }
            }
        }
    } else {
        key_cnt = 0;
        key_prev = now;
    }

    /* ---- key_3 ---- */
    uint8_t now3 = (HAL_GPIO_ReadPin(key_3_GPIO_Port, key_3_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    if (now3 == key3_prev) {
        key3_cnt++;
        if (key3_cnt >= KEY_DEBOUNCE_CNT) {
            key3_cnt = KEY_DEBOUNCE_CNT;
            if (key3_confirmed != now3) {
                key3_confirmed = now3;
                if (now3 == 1) {
                    /* key_3：选择 key_3 模式 */
                    g_app_mode = APP_MODE_KEY3;
                    Apply_Profile(&PROFILE_KEY3);

                    if (stopped_by_sensor) {
                        stopped_by_sensor = 0;
                        Start_Run();
                    } else if (g_app_state == APP_STATE_IDLE) {
                        Start_Run();
                    } else if (g_app_state == APP_STATE_RUNNING) {
                        /* 运行中按 key_3：停止当前任务，停在 IDLE（mode 已切） */
                        g_speed_ctrl_enable = 0;
                        g_app_state = APP_STATE_IDLE;
                        Motor_StopAll();
                    }
                }
            }
        }
    } else {
        key3_cnt = 0;
        key3_prev = now3;
    }

    /* ---- key_4 ----（25 pps，30 秒后自动停，关闭丢线停车） */
    uint8_t now4 = (HAL_GPIO_ReadPin(key_4_GPIO_Port, key_4_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    if (now4 == key4_prev) {
        key4_cnt++;
        if (key4_cnt >= KEY_DEBOUNCE_CNT) {
            key4_cnt = KEY_DEBOUNCE_CNT;
            if (key4_confirmed != now4) {
                key4_confirmed = now4;
                if (now4 == 1) {
                    /* key_4：选择 key_4 模式 */
                    g_app_mode = APP_MODE_KEY4;
                    Apply_Profile(&PROFILE_KEY4);

                    if (stopped_by_sensor) {
                        stopped_by_sensor = 0;
                        Start_Run();
                    } else if (g_app_state == APP_STATE_IDLE) {
                        Start_Run();
                    } else if (g_app_state == APP_STATE_RUNNING) {
                        /* 运行中按 key_4：停止当前任务，停在 IDLE（mode 已切） */
                        g_speed_ctrl_enable = 0;
                        g_app_state = APP_STATE_IDLE;
                        Motor_StopAll();
                    }
                }
            }
        }
    } else {
        key4_cnt = 0;
        key4_prev = now4;
    }
}

/* ============================================================
 * 编码器速度计算（mm/s）
 * ============================================================ */
#define ENCODER_PPR_MOTOR   13.0f
#define GEAR_RATIO          28.0f
#define ENCODER_QUAD         4.0f
#define WHEEL_DIAMETER_MM    65.0f
#define ENCODE_PER_MM        (ENCODER_PPR_MOTOR * GEAR_RATIO * ENCODER_QUAD / (3.1415926f * WHEEL_DIAMETER_MM))

static float Motor_SpeedMmps(const Motor_t *m)
{
    return (float)Motor_GetSpeedPps(m) / ENCODE_PER_MM / CTRL_PERIOD_S;
}

static float Motor_AvgSpeedMmps(void)
{
    int32_t sum = (int32_t)Motor_GetSpeedPps(&motor3)
                + (int32_t)Motor_GetSpeedPps(&motor4);
    return (float)sum / 2.0f / ENCODE_PER_MM / CTRL_PERIOD_S;
}

/* ============================================================
 * 串级PID主循环（Cascade_Update）
 * ============================================================ */
void Cascade_Update(void)
{
    /* ---- 传感器触发的锁定停车状态：直到用户再次按下按钮才恢复 ---- */
    if (stopped_by_sensor) {
        return;
    }

    /* ---- 更新编码器 ---- */
    Motor_EncoderUpdate(&motor3);
    Motor_EncoderUpdate(&motor4);

    /* ---- 读取灰度传感器 ---- */
    g_track_error = Calc_TrackError();
    uint8_t track_detected = (g_track_error != 0) ? 1 : 0;

    /* 检测有多少路传感器扫到白线（gray_data[i] != 0 即白底） */
    uint8_t white_count = 0;
    for (int i = 0; i < 8; i++) {
        if (gray_data[i] != 0) {
            white_count++;
        }
    }

    /* 检查是否到达自动停车时间：
     * - key_2：run_duration_ms = 0，跳过此分支（只依赖丢线停车）
     * - key_3：到 8s 自动停
     * - key_4：到 30s 自动停
     */
    uint32_t dur = g_profile->run_duration_ms;
    if (dur > 0) {
        uint32_t now_ms = HAL_GetTick();
        if ((now_ms - run_start_tick) >= dur) {
            pid_L.err_int = 0.0f;
            pid_R.err_int = 0.0f;
            pid_Steer.err_int = 0.0f;
            Motor_StopAll();
            stopped_by_sensor = 1;
            g_speed_ctrl_enable = 0;
            g_app_state = APP_STATE_IDLE;
            g_run_stop_tick  = now_ms;
            g_run_had_finish = 1;
            track_detected_prev = 0;
            return;
        }
    }

    /* 丢线/全白停车（仅在允许时启用）：
     * key_3 / key_4 模式的 stop_white_threshold = 0，整段被跳过；
     * key_2 模式仍按 4 路白线阈值停车。
     */
    uint8_t thr = g_profile->stop_white_threshold;
    if (thr > 0 && (white_count == 8 || white_count >= thr)) {
        pid_L.err_int = 0.0f;
        pid_R.err_int = 0.0f;
        pid_Steer.err_int = 0.0f;
        Motor_StopAll();
        stopped_by_sensor = 1;
        g_speed_ctrl_enable = 0;
        g_app_state = APP_STATE_IDLE;
        g_run_stop_tick  = HAL_GetTick();
        g_run_had_finish = 1;
        track_detected_prev = 0;
        return;
    }

    /* 离开黑线的瞬间清除速度PID积分，防止拿起来后恢复慢 */
    if (track_detected_prev && !track_detected) {
        pid_L.err_int = 0.0f;
        pid_R.err_int = 0.0f;
    }
    track_detected_prev = track_detected;

    /* ---- 按时间切换两阶段参数（key_2：前 15s 50 pps → 之后 20 pps） ---- */
    uint16_t switch_ms = g_profile->phase2_switch_ms;
    if (switch_ms > 0) {
        uint32_t now_ms = HAL_GetTick();
        if (!g_profile_stage2_active && (now_ms - run_start_tick) >= switch_ms) {
            /* 首次进入阶段二：切换速度目标 & PID 参数，并清积分 */
            g_profile_stage2_active = 1;
            pid_L.err_int = 0.0f;
            pid_R.err_int = 0.0f;
            pid_Steer.err_int = 0.0f;
            /* 速度环参数 */
            pid_L.kp = g_profile->phase2_speed_kp;
            pid_L.ki = g_profile->phase2_speed_ki;
            pid_L.kd = g_profile->phase2_speed_kd;
            pid_R.kp = g_profile->phase2_speed_kp;
            pid_R.ki = g_profile->phase2_speed_ki;
            pid_R.kd = g_profile->phase2_speed_kd;
            /* 转向环参数 */
            pid_Steer.kp = g_profile->phase2_steer_kp;
            pid_Steer.ki = g_profile->phase2_steer_ki;
        }
    }

    /* ---- 计算左右轮实际速度（脉冲/控制周期） ---- */
    int16_t p3 = Motor_GetSpeedPps(&motor3);
    int16_t p4 = Motor_GetSpeedPps(&motor4);
    g_m3_speed_pps = p3;
    g_m4_speed_pps = p4;
    float s3 = (float)p3;
    float s4 = (float)p4;
    g_m3_speed_mmps = s3;
    g_m4_speed_mmps = s4;
    float left_speed  = s3;
    float right_speed = s4;
    g_left_speed_mmps = left_speed;
    g_right_speed_mmps = right_speed;

    if (!g_speed_ctrl_enable) return;

    /* ---- 外环: 转向PID ---- */
    /* 速度环参数已在 phase 切换时一次性赋值，阶段一参数在 Apply_Profile 里赋值，
     * 此处只需设置 actual/target 并计算输出。 */
    pid_Steer.actual = (float)g_track_error;
    pid_Steer.target = 0.0f;
    PID_Update(&pid_Steer);
    float steer_out = pid_Steer.output;

    /* 死区：误差很小时不转向，减少抖动 */
    if (g_track_error >= -STEER_DEAD_ZONE && g_track_error <= STEER_DEAD_ZONE) {
        steer_out = 0.0f;
    }

    /* 渐进式差速：把转向输出打个折扣，转向更温和 */
    float steer_diff = steer_out * g_profile->steer_diff_ratio;

    /* ---- 差分速度目标（脉冲/控制周期） ---- */
    int16_t base_i16 = g_profile_stage2_active ? g_profile->phase2_speed
                                               : g_profile->base_speed;
    static int16_t last_base_i16 = 0;
    if (last_base_i16 != 0 && base_i16 != last_base_i16) {
        /* 阶段切换瞬间清速度 PID 积分，避免从全速→低速时积分拖尾造成抖动/反向 */
        pid_L.err_int = 0.0f;
        pid_R.err_int = 0.0f;
    }
    last_base_i16 = base_i16;
    float base = (float)base_i16;
    float target_L = base - steer_diff;
    float target_R = base + steer_diff;

    /* ---- VOFA+ 串口发送（调试用：发原始脉冲数） ---- */
    Vofa_Send(target_L, s3, 0, 0, target_R, (float)p4, 0, 0);

    /* 目标突变时清除积分（防止超调） */
    static float last_target_L = 0.0f, last_target_R = 0.0f;
    if (target_L - last_target_L > 10.0f || target_L - last_target_L < -10.0f ||
        target_R - last_target_R > 10.0f || target_R - last_target_R < -10.0f) {
        pid_L.err_int = 0.0f;
        pid_R.err_int = 0.0f;
    }
    last_target_L = target_L;
    last_target_R = target_R;

    /* ---- 内环: 速度PID（左右独立） ---- */
    pid_L.target = target_L;
    pid_L.actual = left_speed;
    PID_Update(&pid_L);

    pid_R.target = target_R;
    pid_R.actual = right_speed;
    PID_Update(&pid_R);

    /* ---- 赋值PWM ---- */
    g_left_pwm  = (int16_t)pid_L.output;
    g_right_pwm = (int16_t)pid_R.output;

    Motor_DriveDiff(g_left_pwm, g_right_pwm);
}

/* ============================================================
 * 兼容旧接口（main.c还在调这个）
 * ============================================================ */
void App_ControlLoop(void)
{
    Cascade_Update();
}
