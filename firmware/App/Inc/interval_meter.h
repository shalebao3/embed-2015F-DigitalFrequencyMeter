#ifndef INTERVAL_METER_H
#define INTERVAL_METER_H

#include <stdint.h>

/** @brief 启动 A->B 时间间隔测量。 */
void IntervalMeter_Start(void);

/** @brief 主循环超时任务：A 到达后长期等不到 B 时重新同步。 */
void IntervalMeter_Task(void);

/** @brief A(CH1) 上升沿捕获事件。 */
void IntervalMeter_OnCaptureA(uint64_t timestamp);

/** @brief B(CH2) 上升沿捕获事件。 */
void IntervalMeter_OnCaptureB(uint64_t timestamp);

/** @brief 当前 A->B 时间间隔，单位 ns。 */
uint64_t IntervalMeter_GetNs(void);

/** @brief 当前 A->B 结果是否有效。 */
uint8_t IntervalMeter_IsValid(void);

#endif /* INTERVAL_METER_H */
