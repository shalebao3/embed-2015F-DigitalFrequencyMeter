#ifndef INSTRUMENT_H
#define INSTRUMENT_H

#include <stdint.h>

/**
 * @brief 仪器对外提供的功能模式。
 * @note 这个枚举描述“用户想测什么”，不是底层采用哪种测量算法。
 */
typedef enum
{
  INSTRUMENT_MODE_FREQUENCY = 0, // 频率模式，默认模式
  INSTRUMENT_MODE_PERIOD,        // 周期模式
  INSTRUMENT_MODE_DUTY,          // 占空比模式
  INSTRUMENT_MODE_INTERVAL       // A -> B 时间间隔模式
} InstrumentMode;

/**
 * @brief 初始化测量硬件，并让仪器默认进入 FREQUENCY 模式。
 * @note 必须在 MX_TIM1_Init() ~ MX_TIM4_Init() 完成后调用。
 * @note 该函数只负责一次性初始化；后续持续测量由 Instrument_Task() 驱动。
 */
void Instrument_Init(void);

/**
 * @brief 仪器应用层主任务，应在 while(1) 中持续调用。
 * @note 根据当前功能模式执行对应测量任务，并分发 TIM1+TIM4 的 Gate 测频结果。
 */
void Instrument_Task(void);

/**
 * @brief 切换仪器功能模式。
 * @param new_mode 目标模式，取 InstrumentMode 中的一个枚举值。
 * @note FREQUENCY 与 PERIOD 共享同一套测量硬件，仅切换上层结果语义。
 * @note 切换到 DUTY 或 INTERVAL 时会重新配置 TIM2 的工作方式。
 */
void Instrument_SetMode(InstrumentMode new_mode);

/**
 * @brief 获取当前仪器功能模式。
 * @retval 当前 InstrumentMode 枚举值。
 */
InstrumentMode Instrument_GetMode(void);

/**
 * @brief 获取当前频率结果。
 * @retval 当前频率，单位 Hz；当前模式没有频率语义时返回 0。
 */
uint32_t Instrument_GetFrequencyHz(void);

/**
 * @brief 获取当前周期结果。
 * @retval 当前周期，单位 ns；当前模式没有周期语义时返回 0。
 */
uint64_t Instrument_GetPeriodNs(void);

/**
 * @brief 获取当前占空比结果。
 * @retval 占空比千分数，0~1000 对应 0.0%~100.0%；非 DUTY 模式返回 0。
 */
uint16_t Instrument_GetDutyPermille(void);

/**
 * @brief 获取当前 A->B 时间间隔结果。
 * @retval A->B 时间间隔，单位 ns；非 INTERVAL 模式返回 0。
 */
uint64_t Instrument_GetIntervalNs(void);

/**
 * @brief 判断当前功能模式的最终测量结果是否有效。
 * @retval 1=当前结果有效；0=尚未得到完整结果、已超时或当前结果无效。
 */
uint8_t Instrument_IsResultValid(void);

#endif /* INSTRUMENT_H */
