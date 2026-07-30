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
#define STEER_KP            5.5f
#define STEER_KI            0.1f  
#define STEER_KD            0.0f
#define STEER_ERRINT_MAX    100.0f
#define STEER_ERRINT_THRESH 3.0f
#define STEER_OUT_MAX       50.0f
#define STEER_OUT_MIN       (-50.0f)

/* ============================================================
 * 灰度传感器加权权重（-7~+7）
 * 中间传感器(3,4)权重为0，避免死区
 * ============================================================ */
static const int8_t TRACK_WEIGHT[8] = {-4, -3, -1, 0, 0, 1, 3, 4};

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
#define KEY_DEBOUNCE_CNT  5  /* 连续5次检测到相同状态才确认（约50ms消抖）*/
static int16_t last_err = 0;
static uint8_t track_detected_prev = 0;

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
}

void App_KeyScan(void)
{
    uint8_t now = (HAL_GPIO_ReadPin(key_2_GPIO_Port, key_2_Pin) == GPIO_PIN_RESET) ? 1 : 0;

    if (now == key_prev) {
        key_cnt++;
        if (key_cnt >= KEY_DEBOUNCE_CNT) {
            key_cnt = KEY_DEBOUNCE_CNT;
            if (key_confirmed != now) {
                key_confirmed = now;
                if (now == 1) {
                    if (stopped_by_sensor) {
                        /* 由传感器触发停车后，按一次按钮即恢复运行 */
                        stopped_by_sensor = 0;
                        Motor_ResetAllEncoders();
                        PID_Clear(&pid_L);
                        PID_Clear(&pid_R);
                        PID_Clear(&pid_Steer);
                        g_left_speed_mmps = 0.0f;
                        g_right_speed_mmps = 0.0f;
                        g_left_pwm = 0;
                        g_right_pwm = 0;
                        g_speed_ctrl_enable = 1;
                        g_app_state = APP_STATE_RUNNING;
                    } else if (g_app_state == APP_STATE_IDLE) {
                        Motor_ResetAllEncoders();
                        PID_Clear(&pid_L);
                        PID_Clear(&pid_R);
                        PID_Clear(&pid_Steer);
                        g_left_speed_mmps = 0.0f;
                        g_right_speed_mmps = 0.0f;
                        g_left_pwm = 0;
                        g_right_pwm = 0;
                        g_speed_ctrl_enable = 1;
                        g_app_state = APP_STATE_RUNNING;
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

    /* 触发条件 1：全白（兜底） */
    /* 触发条件 2：白线数量超过阈值（丢线停车） */
    if (white_count == 8 || white_count >= LOOSE_LINE_WHITE_THRESHOLD) {
        /* 清除积分，防止下次出弯时突然往前冲 */
        pid_L.err_int = 0.0f;
        pid_R.err_int = 0.0f;
        pid_Steer.err_int = 0.0f;
        Motor_StopAll();
        /* 锁定停车状态：禁用速度控制并切到 IDLE，
         * 直到用户再次按下按钮（App_KeyScan 中检测 stopped_by_sensor 后恢复） */
        stopped_by_sensor = 1;
        g_speed_ctrl_enable = 0;
        g_app_state = APP_STATE_IDLE;
        track_detected_prev = 0;
        return;
    }

    /* 离开黑线的瞬间清除速度PID积分，防止拿起来后恢复慢 */
    if (track_detected_prev && !track_detected) {
        pid_L.err_int = 0.0f;
        pid_R.err_int = 0.0f;
    }
    track_detected_prev = track_detected;

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
    pid_Steer.actual = (float)g_track_error;
    pid_Steer.target = 0.0f;
    PID_Update(&pid_Steer);
    float steer_out = pid_Steer.output;

    /* 死区：误差很小时不转向，减少抖动 */
    if (g_track_error >= -STEER_DEAD_ZONE && g_track_error <= STEER_DEAD_ZONE) {
        steer_out = 0.0f;
    }

    /* 渐进式差速：把转向输出打个折扣，转向更温和 */
    float steer_diff = steer_out * STEER_DIFF_RATIO;

    /* ---- 差分速度目标（脉冲/控制周期） ---- */
    float target_L = (float)BASE_SPEED_PULSE - steer_diff;
    float target_R = (float)BASE_SPEED_PULSE + steer_diff;

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
