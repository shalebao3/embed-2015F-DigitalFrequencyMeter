#ifndef DUTY_METER_H
#define DUTY_METER_H

#include <stdint.h>

/**
 * @brief 启动 DUTY 占空比测量。
 * @param frequency_hint_hz 切换前已知的频率提示，单位 Hz；为 0 时使用默认安全 PSC。
 * @note 会根据频率提示选择初始 PSC，并把 TIM2 配置为 PWM Input。
 */
void DutyMeter_Start(uint32_t frequency_hint_hz);

/**
 * @brief DUTY 模式主任务。
 * @note 轮询 TIM2 PWM Input 的 CCR1/CCR2，计算占空比并处理捕获超时。
 */
void DutyMeter_Task(void);

/**
 * @brief 接收一轮 TIM1+TIM4 硬件 Gate 粗频率结果。
 * @param frequency_hz 本轮 Gate 测得的频率，单位 Hz。
 * @note 该频率用于辅助频率/周期输出，并驱动 TIM2 PSC 自动量程。
 */
void DutyMeter_OnGateMeasurement(uint32_t frequency_hz);

/**
 * @brief 获取最终占空比结果。
 * @retval 占空比千分数，0~1000 对应 0.0%~100.0%。
 */
uint16_t DutyMeter_GetPermille(void);

/**
 * @brief 获取 DUTY 模式下的辅助频率结果。
 * @retval 当前频率，单位 Hz；尚无 Gate 结果时为 0。
 */
uint32_t DutyMeter_GetFrequencyHz(void);

/**
 * @brief 获取 DUTY 模式下的辅助周期结果。
 * @retval 当前周期，单位 ns；尚无有效辅助频率时为 0。
 */
uint64_t DutyMeter_GetPeriodNs(void);

/**
 * @brief 判断当前占空比结果是否有效。
 * @retval 1=已经获得完整且未超时的 PWM Input 结果；0=结果无效。
 */
uint8_t DutyMeter_IsValid(void);

#endif /* DUTY_METER_H */
