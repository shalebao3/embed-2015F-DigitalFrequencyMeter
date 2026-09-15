#ifndef MEASUREMENT_HW_H
#define MEASUREMENT_HW_H

#include <stdint.h>

/* TIM2 在普通时间戳模式下直接使用 72 MHz 计数时钟。 */
#define MEASUREMENT_TIM2_COUNTER_HZ 72000000ULL
#define MEASUREMENT_NANOSECONDS_PER_SECOND 1000000000ULL

/**
 * @brief 启动测量链使用到的 TIM1/TIM2/TIM3/TIM4。
 * @note CubeMX 的 MX_TIMx_Init() 必须已经执行。
 * @note HAL Start 只在初始化阶段调用一次；后续模式切换通过寄存器重新配置。
 */
void MeasurementHw_Init(void);

/**
 * @brief 停止并清理 TIM1 + TIM4 硬件 Gate 测频链。
 * @note 会清除 Gate 完成标志、TIM1 软件溢出计数，并把 TIM1/TIM4 CNT 归零。
 */
void MeasurementHw_GateStop(void);

/**
 * @brief 从干净状态启动新一轮 1 秒硬件 Gate。
 * @note 先准备 TIM1，再启动 TIM4 One Pulse，避免窗口起点丢脉冲。
 */
void MeasurementHw_GateStartFresh(void);

/**
 * @brief 在 Gate 已完成时读取本轮频率，并立即准备下一轮 Gate。
 * @param frequency_hz 输出参数，用于返回本轮 1 秒 Gate 内统计到的脉冲总数，单位 Hz。
 * @retval 1=本次取得了新的 Gate 结果；0=输出指针为空或当前 Gate 尚未结束。
 */
uint8_t MeasurementHw_GateTakeFrequency(uint32_t *frequency_hz);

/**
 * @brief 把 TIM2 配置为自由运行时间轴 + 双路输入捕获模式。
 * @param enable_ch2_interrupt 0=只使能 CH1 捕获中断；非 0=额外使能 CH2 捕获中断。
 * @note CH1 固定为 Direct TI1，CH2 固定为 Direct TI2；该参数只控制 CH2 中断，不关闭 CH2 捕获硬件。
 */
void MeasurementHw_TIM2ConfigureTimestampCapture(uint8_t enable_ch2_interrupt);

/**
 * @brief 把 TIM2 配置为 PWM Input，供 DUTY 模式测量周期和高电平时间。
 * @param prescaler TIM2 预分频寄存器 PSC 的目标值。
 * @note CH1 Direct TI1 捕获上升沿并得到周期，CH2 Indirect TI1 捕获下降沿并得到高电平时间。
 */
void MeasurementHw_TIM2ConfigureDutyCapture(uint16_t prescaler);

/**
 * @brief DUTY 自动量程时更新 TIM2 PSC，并重新同步硬件计数。
 * @param prescaler 新的 TIM2 预分频寄存器 PSC 值。
 */
void MeasurementHw_TIM2ApplyPrescaler(uint16_t prescaler);

/**
 * @brief 读取 TIM2 某捕获通道，并结合软件溢出计数构造 64 位扩展时间戳。
 * @param channel TIM_CHANNEL_1 或 TIM_CHANNEL_2。
 * @retval 64 位扩展时间戳，单位为 TIM2 tick。
 */
uint64_t MeasurementHw_TIM2ReadCapturedTimestamp(uint32_t channel);

/**
 * @brief 读取 PWM Input 的一组 CCR1/CCR2 捕获结果。
 * @param period_ticks 输出完整周期 tick 数，对应 CCR1。
 * @param high_ticks 输出高电平持续 tick 数，对应 CCR2。
 * @retval 1=同时取得新的周期和高电平捕获；0=输出指针为空或数据尚不完整。
 */
uint8_t MeasurementHw_TIM2ReadPwmCapture(uint32_t *period_ticks,
                                         uint32_t *high_ticks);

/**
 * @brief 打开或关闭 TIM2 CH1 捕获中断。
 * @param enable 0=关闭 CC1 中断；非 0=打开 CC1 中断。
 * @note 切换前会先关闭 CC1 中断并清除旧 CC1 标志，避免处理陈旧捕获事件。
 */
void MeasurementHw_TIM2SetCh1Interrupt(uint8_t enable);

/**
 * @brief 在 NVIC 层暂时屏蔽 TIM2_IRQn。
 * @note 用于保护与 TIM2 ISR 共享的软件状态，避免主循环与中断竞争。
 */
void MeasurementHw_TIM2IrqDisable(void);

/**
 * @brief 清除 TIM2_IRQn Pending 状态后重新使能 TIM2 中断。
 */
void MeasurementHw_TIM2IrqEnable(void);

#endif /* MEASUREMENT_HW_H */
