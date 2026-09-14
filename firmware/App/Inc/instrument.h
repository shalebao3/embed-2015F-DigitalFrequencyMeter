#ifndef INSTRUMENT_H
#define INSTRUMENT_H

#include <stdint.h>

/**
 * @brief 仪器对外提供的功能模式。
 * @note 这个枚举描述“用户想测什么”，不是底层采用哪种测量算法。
 */
typedef enum
{
  INSTRUMENT_MODE_FREQUENCY = 0, // 频率模式
  INSTRUMENT_MODE_PERIOD,        // 周期模式
  INSTRUMENT_MODE_DUTY,          // 占空比模式
  INSTRUMENT_MODE_INTERVAL       // A -> B 时间间隔模式
} InstrumentMode;

/**
 * @brief 初始化测量硬件和默认仪器模式。
 * @note 必须在 MX_TIMx_Init() 完成后调用。
 */
void Instrument_Init(void);

/**
 * @brief 仪器主任务；在 while(1) 中持续调用。
 */
void Instrument_Task(void);

/**
 * @brief 切换仪器功能模式。
 * @param new_mode 目标模式。
 */
void Instrument_SetMode(InstrumentMode new_mode);

/** @brief 获取当前仪器功能模式。 */
InstrumentMode Instrument_GetMode(void);

/** @brief 获取当前频率结果，单位 Hz。无频率语义的模式返回 0。 */
uint32_t Instrument_GetFrequencyHz(void);

/** @brief 获取当前周期结果，单位 ns。无周期语义的模式返回 0。 */
uint64_t Instrument_GetPeriodNs(void);

/** @brief 获取当前占空比千分数，0~1000 对应 0.0%~100.0%。 */
uint16_t Instrument_GetDutyPermille(void);

/** @brief 获取当前 A->B 时间间隔，单位 ns。 */
uint64_t Instrument_GetIntervalNs(void);

/** @brief 当前模式的最终结果是否有效。 */
uint8_t Instrument_IsResultValid(void);

#endif /* INSTRUMENT_H */
