#include "frequency_meter.h"

#include "main.h"
#include "measurement_hw.h"

/* 1 Hz 周期约 1000 ms；留 500 ms 余量判断输入断开。 */
#define FREQUENCY_TIMEOUT_MS 1500U

/* 72 MHz 下 10 kHz 周期约 7200 tick：周期法 -> 闸门法。 */
#define PERIOD_TO_GATE_TICKS 7200ULL

/* 与 10 kHz 形成迟滞：闸门法 <= 7 kHz 后切回周期法。 */
#define GATE_TO_PERIOD_HZ 7000U

/**
 * @brief FREQUENCY / PERIOD 内部测量策略。
 */
typedef enum
{
  FREQUENCY_METHOD_PERIOD = 0, // TIM2 相邻上升沿周期法
  FREQUENCY_METHOD_GATE        // TIM1 + TIM4 1 秒闸门法
} FrequencyMethod;

/* ==================== TIM2 周期法状态 ==================== */
static volatile uint64_t period_ticks = 0;        // 一个完整周期，单位 TIM2 tick
static volatile uint64_t period_ns = 0;           // 周期，单位 ns
static volatile uint64_t timestamp1 = 0;          // 上一次 CH1 64 位扩展时间戳
static volatile uint64_t timestamp2 = 0;          // 当前 CH1 64 位扩展时间戳
static volatile uint8_t capture_state = 0;        // 0=等待第一沿，1=已有上一时间戳
static volatile uint32_t frequency_hz = 0;        // 周期法整数 Hz 结果
static volatile uint64_t frequency_millihz = 0;   // 周期法 mHz 结果，保留低频小数精度
static volatile uint32_t last_capture_tick_ms = 0; // 最近一次 CH1 边沿的 HAL tick
static volatile uint8_t frequency_valid = 0;      // 周期法结果是否有效
static volatile uint8_t period_requests_gate = 0; // 周期太短，请求切到闸门法

/* ==================== 自动策略和最终输出 ==================== */
static volatile FrequencyMethod frequency_method = FREQUENCY_METHOD_PERIOD;
static volatile uint32_t gate_frequency_hz = 0;   // 最近一次 1 秒 Gate 结果
static volatile uint8_t gate_frequency_valid = 0; // 是否至少完成一轮 Gate
static volatile uint32_t measured_frequency_hz = 0; // 最终对上层提供的 Hz
static volatile uint64_t measured_period_ns = 0;    // 最终对上层提供的 ns

/** @brief 清空频率模块软件状态，不碰硬件。 */
static void FrequencyMeter_ResetState(void)
{
  period_ticks = 0;
  period_ns = 0;
  timestamp1 = 0;
  timestamp2 = 0;
  capture_state = 0;
  frequency_hz = 0;
  frequency_millihz = 0;
  last_capture_tick_ms = 0;
  frequency_valid = 0;
  period_requests_gate = 0;

  frequency_method = FREQUENCY_METHOD_PERIOD;
  gate_frequency_hz = 0;
  gate_frequency_valid = 0;
  measured_frequency_hz = 0;
  measured_period_ns = 0;
}

/**
 * @brief 启动 FREQUENCY / PERIOD 共用测量引擎。
 */
void FrequencyMeter_Start(void)
{
  FrequencyMeter_ResetState();
  MeasurementHw_TIM2ConfigureTimestampCapture(0U);
  MeasurementHw_GateStartFresh();
}

/**
 * @brief TIM2_CH1 上升沿捕获事件；完成周期法计算。
 * @param timestamp 已经由 measurement_hw 扩展好的 64 位 TIM2 时间戳。
 */
void FrequencyMeter_OnCapture(uint64_t timestamp)
{
  last_capture_tick_ms = HAL_GetTick();

  if (capture_state == 0U)
  {
    timestamp1 = timestamp;
    capture_state = 1U;
    return;
  }

  timestamp2 = timestamp;
  period_ticks = timestamp2 - timestamp1;

  if (period_ticks != 0ULL)
  {
    frequency_hz =
        (uint32_t)((MEASUREMENT_TIM2_COUNTER_HZ + period_ticks / 2ULL) /
                   period_ticks);

    frequency_millihz =
        (MEASUREMENT_TIM2_COUNTER_HZ * 1000ULL +
         period_ticks / 2ULL) /
        period_ticks;

    period_ns =
        (period_ticks * MEASUREMENT_NANOSECONDS_PER_SECOND +
         MEASUREMENT_TIM2_COUNTER_HZ / 2ULL) /
        MEASUREMENT_TIM2_COUNTER_HZ;

    frequency_valid = 1U;

    /* 约 >=10 kHz 时立即关 CC1 中断，避免高频 ISR 风暴。 */
    if (period_ticks <= PERIOD_TO_GATE_TICKS)
    {
      period_requests_gate = 1U;
      MeasurementHw_TIM2SetCh1Interrupt(0U);
    }
  }

  timestamp1 = timestamp2;
}

/**
 * @brief 接收一轮 1 秒闸门测量结果。
 */
void FrequencyMeter_OnGateMeasurement(uint32_t new_frequency_hz)
{
  gate_frequency_hz = new_frequency_hz;
  gate_frequency_valid = 1U;

  if (frequency_method == FREQUENCY_METHOD_GATE)
  {
    measured_frequency_hz = gate_frequency_hz;

    /* 下降到 7 kHz 或更低后重新启用周期法。 */
    if (gate_frequency_hz <= GATE_TO_PERIOD_HZ)
    {
      frequency_method = FREQUENCY_METHOD_PERIOD;
      frequency_valid = 0;
      period_requests_gate = 0;
      capture_state = 0;
      period_ticks = 0;
      period_ns = 0;
      timestamp1 = 0;
      timestamp2 = 0;

      MeasurementHw_TIM2SetCh1Interrupt(1U);
    }
  }
}

/**
 * @brief 频率模块主任务：处理超时、策略切换和最终结果整理。
 */
void FrequencyMeter_Task(void)
{
  /* PERIOD 方法下长时间没有新边沿，则让旧周期结果失效。 */
  if ((frequency_method == FREQUENCY_METHOD_PERIOD) &&
      frequency_valid &&
      ((uint32_t)(HAL_GetTick() - last_capture_tick_ms) > FREQUENCY_TIMEOUT_MS))
  {
    MeasurementHw_TIM2IrqDisable();

    /* 关 IRQ 后再次判断，避免刚好有新 Capture 与超时判断竞争。 */
    if ((frequency_method == FREQUENCY_METHOD_PERIOD) &&
        frequency_valid &&
        ((uint32_t)(HAL_GetTick() - last_capture_tick_ms) > FREQUENCY_TIMEOUT_MS))
    {
      frequency_valid = 0;
      frequency_hz = 0;
      frequency_millihz = 0;
      period_ticks = 0;
      period_ns = 0;
      timestamp1 = 0;
      timestamp2 = 0;
      capture_state = 0;
      period_requests_gate = 0;
    }

    MeasurementHw_TIM2IrqEnable();
  }

  /* ISR 已经立即关闭 CC1；主循环只完成 PERIOD -> GATE 软件状态切换。 */
  if ((frequency_method == FREQUENCY_METHOD_PERIOD) && period_requests_gate)
  {
    measured_frequency_hz = frequency_hz;
    frequency_method = FREQUENCY_METHOD_GATE;
    period_requests_gate = 0;
    frequency_valid = 0;
    capture_state = 0;
  }

  /* FREQUENCY / PERIOD 共用最终结果。 */
  if (frequency_method == FREQUENCY_METHOD_PERIOD)
  {
    if (frequency_valid)
    {
      measured_frequency_hz = frequency_hz;
      measured_period_ns = period_ns;
    }
    else if (gate_frequency_valid)
    {
      /* 保留最近 Gate 频率作为周期法重新同步期间的频率 fallback。 */
      measured_frequency_hz = gate_frequency_hz;
    }
    else
    {
      measured_frequency_hz = 0;
      measured_period_ns = 0;
    }
  }
  else if (gate_frequency_valid)
  {
    measured_frequency_hz = gate_frequency_hz;
  }

  /* 高频闸门法只有频率计数，周期由最终频率反算。 */
  if ((frequency_method == FREQUENCY_METHOD_GATE) &&
      (measured_frequency_hz != 0U))
  {
    measured_period_ns =
        (MEASUREMENT_NANOSECONDS_PER_SECOND + measured_frequency_hz / 2U) /
        measured_frequency_hz;
  }
}

uint32_t FrequencyMeter_GetFrequencyHz(void)
{
  return measured_frequency_hz;
}

uint64_t FrequencyMeter_GetPeriodNs(void)
{
  return measured_period_ns;
}

uint8_t FrequencyMeter_IsValid(void)
{
  if (frequency_method == FREQUENCY_METHOD_PERIOD)
  {
    return (uint8_t)(frequency_valid || gate_frequency_valid);
  }

  return gate_frequency_valid;
}
