#ifndef FREQUENCY_METER_H
#define FREQUENCY_METER_H

#include <stdint.h>

/**
 * @brief 初始化并启动 FREQUENCY / PERIOD 共用测量引擎。
 * @note 会清空软件状态、把 TIM2 配置为时间戳捕获，并启动 TIM1+TIM4 Gate。
 */
void FrequencyMeter_Start(void);

/**
 * @brief 频率测量主任务。
 * @note 负责周期法超时检测、周期法/闸门法策略切换，以及最终结果整理。
 */
void FrequencyMeter_Task(void);

/**
 * @brief 处理 TIM2_CH1 上升沿捕获事件，完成周期法测量。
 * @param timestamp 由 measurement_hw 扩展后的 64 位 TIM2 时间戳，单位为 TIM2 tick。
 */
void FrequencyMeter_OnCapture(uint64_t timestamp);

/**
 * @brief 接收一轮 TIM1+TIM4 硬件 Gate 测频结果。
 * @param frequency_hz 本轮 1 秒 Gate 得到的频率，单位 Hz。
 * @note 闸门法运行时会更新最终频率，并在频率下降到迟滞阈值后重新启用周期法。
 */
void FrequencyMeter_OnGateMeasurement(uint32_t frequency_hz);

/**
 * @brief 获取 FREQUENCY / PERIOD 共用的最终频率结果。
 * @retval 当前频率，单位 Hz；尚无有效结果时通常为 0。
 */
uint32_t FrequencyMeter_GetFrequencyHz(void);

/**
 * @brief 获取 FREQUENCY / PERIOD 共用的最终周期结果。
 * @retval 当前周期，单位 ns；尚无有效结果时通常为 0。
 */
uint64_t FrequencyMeter_GetPeriodNs(void);

/**
 * @brief 判断 FREQUENCY / PERIOD 共用的最终结果是否有效。
 * @retval 1=存在有效周期法或闸门法结果；0=当前没有有效结果。
 */
uint8_t FrequencyMeter_IsValid(void);

#endif /* FREQUENCY_METER_H */
