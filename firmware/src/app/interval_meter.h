#ifndef INTERVAL_METER_H
#define INTERVAL_METER_H

#include <stdint.h>

/**
 * @brief 启动 A->B 时间间隔测量。
 * @note 会停止 TIM1+TIM4 Gate，并把 TIM2 配置为 CH1/CH2 双路时间戳捕获。
 */
void IntervalMeter_Start(void);

/**
 * @brief INTERVAL 模式主任务。
 * @note 当 A 已到达但 B 长时间未到达时，负责超时清理并重新同步。
 */
void IntervalMeter_Task(void);

/**
 * @brief 处理 A 事件，即 TIM2_CH1 上升沿捕获。
 * @param timestamp A 到达时的 64 位 TIM2 扩展时间戳，单位为 TIM2 tick。
 */
void IntervalMeter_OnCaptureA(uint64_t timestamp);

/**
 * @brief 处理 B 事件，即 TIM2_CH2 上升沿捕获，并完成 B-A 计算。
 * @param timestamp B 到达时的 64 位 TIM2 扩展时间戳，单位为 TIM2 tick。
 */
void IntervalMeter_OnCaptureB(uint64_t timestamp);

/**
 * @brief 获取最近一次有效的 A->B 时间间隔。
 * @retval A->B 时间间隔，单位 ns；尚无有效结果时通常为 0。
 */
uint64_t IntervalMeter_GetNs(void);

/**
 * @brief 判断当前 A->B 时间间隔结果是否有效。
 * @retval 1=存在完整且未超时的 A->B 配对结果；0=结果无效。
 */
uint8_t IntervalMeter_IsValid(void);

#endif /* INTERVAL_METER_H */
