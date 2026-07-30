# electric_game

基于 STM32F407 的寻迹小车项目，支持多速度档位与两阶段速度控制。

## 硬件平台

- **主控**：STM32F407
- **电机**：两路直流电机（左右独立驱动）
- **传感器**：四路灰度传感器（巡线检测）
- **显示**：OLED 屏幕
- **调试**：串口 + VOFA+ 上位机

## 项目结构

```
Core/
  Inc/           头文件
  Src/           源代码
    app.c        主逻辑：PID级联控制、模式切换、状态机
    motor.c      电机驱动：PWM输出、编码器测速
    pid.c        PID控制器实现
    gray_sensor.c 四路灰度传感器读取
    oled.c       OLED显示驱动
    oled_data.c  OLED字模数据
    main.c       入口、外设初始化、按键与定时器中断
    usart.c      串口DMA发送（VOFA+协议）
    gpio.c       GPIO配置
    tim.c        定时器配置（PWM输出）
    dma.c        DMA配置
    stm32f4xx_*.c STM32 HAL库文件

MDK-ARM/         Keil工程文件
  electric_game.uvprojx   工程文件
  electric_game.hex        编译产物（烧录用）
```

## 运行模式

通过 Key_2 / Key_3 / Key_4 三档拨码开关选择模式：

| 档位 | 基础速度 | 丢线停车 | 超时机制 | 备注 |
|------|:---:|:---:|:---:|------|
| Key_2（默认） | 50 pps | 是 | 无 | 前15s全速，之后自动切换到20pps |
| Key_3 | 25 pps | 否 | 8s 后自动停 | 调试用 |
| Key_4 | 29 pps | 是 | 30s 后自动停 | 留空赛道用 |

> 丢线停车：四路灰度传感器中有≥4路检测到白区时立即停车。

## 两阶段速度控制（Key_2）

Key_2 模式内置两阶段速度策略：

| 阶段 | 时间段 | 目标速度 | 速度环 Kp/Ki/Kd | 转向环 Kp/Ki |
|------|--------|---------|----------------|-------------|
| 阶段一 | 0 ~ 15s | 50 pps | 20 / 2.0 / 2.0 | 5.5 / 0.1 |
| 阶段二 | 15s ~ | 20 pps | 7 / 0.5 / 1.0 | 4.5 / 0.08 |

切换瞬间会清空三个 PID 控制器的积分项，防止积分拖尾引起速度抖动。

## PID 级联控制

```
外环（转向PID）→ 差分速度 → 目标速度
内环（速度PID）→ PWM占空比 → 电机
```

- **控制周期**：10ms（由 TIM3 100Hz 中断驱动）
- **外环输入**：循迹误差（`g_track_error`，范围 -500 ~ 500）
- **内环输入**：编码器脉冲/控制周期（pps）

## VOFA+ 调试协议

串口配置：115200 8N1，FireWater 协议（JustFloat）。

| 通道 | 内容 |
|------|------|
| ch0 | 左轮目标速度（pps） |
| ch1 | 左轮实际速度（pps） |
| ch2 | 右轮目标速度（pps） |
| ch3 | 右轮实际速度（pps） |

## 关键参数调优

调参建议（仅供参考，需根据实际赛道微调）：

| 参数 | 作用 | 典型范围 |
|------|------|---------|
| `speed_kp` | 速度环比例增益 | 15 ~ 25 |
| `speed_ki` | 速度环积分增益 | 1.0 ~ 3.0 |
| `speed_kd` | 速度环微分增益 | 0.5 ~ 3.0 |
| `steer_kp` | 转向环比例增益 | 3.0 ~ 6.0 |
| `steer_ki` | 转向环积分增益 | 0.05 ~ 0.2 |
| `steer_diff_ratio` | 差速折扣（越大转向越猛） | 0.4 ~ 0.7 |

## 编译与烧录

1. 用 Keil 打开 `MDK-ARM/electric_game.uvprojx`
2. 点击 Build（F7）编译
3. 点击 Download（F8）烧录，或使用 `MDK-ARM/electric_game/electric_game.hex`

## 串口接线

- USART1（烧录调试串口）：PA9（TX） / PA10（RX）
- VOFA+ 串口（PA2 TX / PA3 RX）：连接电脑 USB 转串口，VOFA+ 打开对应 COM 口即可看波形

## 核心文件说明

- `Core/Src/app.c`：应用逻辑核心，包含模式定义、PID 参数表、级联控制、状态机
- `Core/Src/motor.c`：电机驱动，封装 `Motor_DriveDiff()` 差速函数
- `Core/Src/pid.c`：通用增量式 PID 控制器
- `Core/Src/gray_sensor.c`：四路灰度传感器采集，`Get_TrackError()` 计算循迹偏差
