#ifndef MEASUREMENT_HW_H
#define MEASUREMENT_HW_H

#include <stdint.h>

/* TIM2 在普通时间戳模式下直接使用 72 MHz 计数时钟。 */
#define MEASUREMENT_TIM2_COUNTER_HZ 72000000ULL
#define MEASUREMENT_NANOSECONDS_PER_SECOND 1000000000ULL

/** @brief 启动测量链使用到的 TIM1/TIM2/TIM3/TIM4。 */
void MeasurementHw_Init(void);

/** @brief 停止并清理 TIM1 + TIM4 硬件闸门链。 */
void MeasurementHw_GateStop(void);

/** @brief 从干净状态启动新一轮 1 秒硬件闸门。 */
void MeasurementHw_GateStartFresh(void);

/**
 * @brief 如果 1 秒 Gate 已完成，则读取本轮频率并立即重启下一轮 Gate。
 * @param frequency_hz 输出本轮 1 秒内统计到的脉冲总数，单位 Hz。
 * @retval 1=本次取到了新结果；0=Gate 尚未结束。
 */
uint8_t MeasurementHw_GateTakeFrequency(uint32_t *frequency_hz);

/**
 * @brief 把 TIM2 配置为自由运行时间轴 + 输入捕获模式。
 * @param enable_ch2_interrupt 0=只开 CH1；1=同时开 CH1/CH2。
 */
void MeasurementHw_TIM2ConfigureTimestampCapture(uint8_t enable_ch2_interrupt);

/** @brief 把 TIM2 配置为 PWM Input，占空比模式使用。 */
void MeasurementHw_TIM2ConfigureDutyCapture(uint16_t prescaler);

/** @brief DUTY 自动量程时只更新 TIM2 PSC，并重新同步硬件计数。 */
void MeasurementHw_TIM2ApplyPrescaler(uint16_t prescaler);

/**
 * @brief 读取 TIM2 某捕获通道，并结合软件溢出计数构造 64 位扩展时间戳。
 * @param channel TIM_CHANNEL_1 或 TIM_CHANNEL_2。
 */
uint64_t MeasurementHw_TIM2ReadCapturedTimestamp(uint32_t channel);

/**
 * @brief 读取 PWM Input 的一组 CCR1/CCR2。
 * @retval 1=同时获得新的周期和高电平捕获；0=数据尚不完整。
 */
uint8_t MeasurementHw_TIM2ReadPwmCapture(uint32_t *period_ticks,
                                         uint32_t *high_ticks);

/** @brief 打开或关闭 TIM2 CH1 捕获中断；切换时自动清除旧 CC1 标志。 */
void MeasurementHw_TIM2SetCh1Interrupt(uint8_t enable);

/** @brief 暂时屏蔽 TIM2_IRQn，用于软件状态与 ISR 的竞态保护。 */
void MeasurementHw_TIM2IrqDisable(void);

/** @brief 清 Pending 后重新打开 TIM2_IRQn。 */
void MeasurementHw_TIM2IrqEnable(void);

#endif /* MEASUREMENT_HW_H */
