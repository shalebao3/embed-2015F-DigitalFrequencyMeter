#ifndef DUTY_METER_H
#define DUTY_METER_H

#include <stdint.h>

/**
 * @brief 启动 DUTY 测量。
 * @param frequency_hint_hz 切换前已知的频率提示；为 0 时使用安全默认 PSC。
 */
void DutyMeter_Start(uint32_t frequency_hint_hz);

/** @brief 主循环轮询 PWM Input CCR，并维护占空比有效性。 */
void DutyMeter_Task(void);

/** @brief 1 秒硬件闸门粗频率事件，用于频率显示和 TIM2 PSC 自动量程。 */
void DutyMeter_OnGateMeasurement(uint32_t frequency_hz);

/** @brief 最终占空比千分数，0~1000 对应 0.0%~100.0%。 */
uint16_t DutyMeter_GetPermille(void);

/** @brief DUTY 模式辅助频率结果，单位 Hz。 */
uint32_t DutyMeter_GetFrequencyHz(void);

/** @brief DUTY 模式辅助周期结果，单位 ns。 */
uint64_t DutyMeter_GetPeriodNs(void);

/** @brief 当前占空比结果是否有效。 */
uint8_t DutyMeter_IsValid(void);

#endif /* DUTY_METER_H */
