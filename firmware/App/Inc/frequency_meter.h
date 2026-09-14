#ifndef FREQUENCY_METER_H
#define FREQUENCY_METER_H

#include <stdint.h>

/** @brief 初始化并启动 FREQUENCY / PERIOD 共用测量引擎。 */
void FrequencyMeter_Start(void);

/** @brief 周期法软件任务：超时、方法切换和最终结果整理。 */
void FrequencyMeter_Task(void);

/** @brief TIM2_CH1 捕获事件入口；timestamp 为 64 位扩展时间戳。 */
void FrequencyMeter_OnCapture(uint64_t timestamp);

/** @brief 1 秒硬件闸门测量完成事件入口。 */
void FrequencyMeter_OnGateMeasurement(uint32_t frequency_hz);

/** @brief 当前最终频率结果，单位 Hz。 */
uint32_t FrequencyMeter_GetFrequencyHz(void);

/** @brief 当前最终周期结果，单位 ns。 */
uint64_t FrequencyMeter_GetPeriodNs(void);

/** @brief 当前最终结果是否有效。 */
uint8_t FrequencyMeter_IsValid(void);

#endif /* FREQUENCY_METER_H */
