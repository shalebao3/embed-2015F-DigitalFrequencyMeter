#include "duty_meter.h"

#include "main.h"
#include "measurement_hw.h"

/* 目标让一个 PWM 周期不超过约 60000 tick，为 16 位 CNT 留余量。 */
#define DUTY_TARGET_PERIOD_TICKS 60000ULL
#define DUTY_DEFAULT_PRESCALER 1199U
#define DUTY_TIMEOUT_MS 1500U

/* ==================== PWM Input 状态 ==================== */
static volatile uint32_t duty_period_ticks = 0;      // CCR1：完整周期 tick
static volatile uint32_t duty_high_ticks = 0;        // CCR2：高电平时间 tick
static volatile uint16_t measured_duty_permille = 0; // 0~1000 对应 0.0%~100.0%
static volatile uint16_t duty_prescaler = DUTY_DEFAULT_PRESCALER;
static volatile uint32_t last_duty_capture_tick_ms = 0;
static volatile uint8_t duty_capture_synced = 0; // 0=等待第一轮完整同步，1=已同步
static volatile uint8_t duty_valid = 0;

/* DUTY 模式中 1 秒 Gate 同时提供辅助频率 / 周期。 */
static volatile uint32_t measured_frequency_hz = 0;
static volatile uint64_t measured_period_ns = 0;

/**
 * @brief 清空占空比捕获软件状态，不修改底层 TIM2 配置。
 * @note PSC 发生变化或捕获超时时调用，旧占空比结果会立即失效。
 */
static void DutyMeter_ResetCaptureState(void)
{
  duty_period_ticks = 0;
  duty_high_ticks = 0;
  measured_duty_permille = 0;
  last_duty_capture_tick_ms = 0;
  duty_capture_synced = 0;
  duty_valid = 0;
}

/**
 * @brief 根据 1 秒 Gate 粗频率计算适合 PWM Input 的 TIM2 PSC。
 * @param gate_frequency Gate 测得的粗频率，单位 Hz。
 * @retval 推荐写入 TIM2 PSC 的 16 位预分频值。
 * @note 使用 gate_count-1 作为保守频率下界，避免低频 ±1 count 误差导致周期溢出。
 */
static uint16_t DutyMeter_CalculatePrescaler(uint32_t gate_frequency)
{
  uint32_t frequency_lower_bound;
  uint64_t denominator;
  uint64_t divider;

  if (gate_frequency == 0U)
  {
    return duty_prescaler;
  }

  frequency_lower_bound = (gate_frequency > 1U) ? (gate_frequency - 1U) : 1U;
  denominator =
      (uint64_t)frequency_lower_bound * DUTY_TARGET_PERIOD_TICKS;
  divider =
      (MEASUREMENT_TIM2_COUNTER_HZ + denominator - 1ULL) / denominator;

  if (divider < 1ULL)
  {
    divider = 1ULL;
  }
  else if (divider > 65536ULL)
  {
    divider = 65536ULL;
  }

  return (uint16_t)(divider - 1ULL);
}

/**
 * @brief 应用新的自动量程 PSC。
 * @param prescaler 目标 TIM2 PSC 值。
 * @note 仅在 PSC 确实变化时重配硬件；时间尺度改变后旧占空比结果立即作废。
 */
static void DutyMeter_ApplyPrescaler(uint16_t prescaler)
{
  if (prescaler == duty_prescaler)
  {
    return;
  }

  MeasurementHw_TIM2ApplyPrescaler(prescaler);
  duty_prescaler = prescaler;
  DutyMeter_ResetCaptureState();
}

/**
 * @brief 启动 DUTY 占空比测量。
 * @param frequency_hint_hz 切换前已知的频率提示，单位 Hz；为 0 时使用默认安全 PSC。
 * @note 配置 TIM2 为 PWM Input，同时启动 TIM1+TIM4 Gate 作为辅助频率和自动量程依据。
 */
void DutyMeter_Start(uint32_t frequency_hint_hz)
{
  duty_prescaler = DUTY_DEFAULT_PRESCALER;

  if (frequency_hint_hz != 0U)
  {
    duty_prescaler = DutyMeter_CalculatePrescaler(frequency_hint_hz);
  }

  DutyMeter_ResetCaptureState();
  measured_frequency_hz = 0;
  measured_period_ns = 0;

  MeasurementHw_TIM2ConfigureDutyCapture(duty_prescaler);
  MeasurementHw_GateStartFresh();
}

/**
 * @brief DUTY 模式主任务，轮询 PWM Input 捕获并计算占空比。
 * @note 第一轮完整捕获仅用于同步；后续使用 CCR2/CCR1 计算占空比，并在长期无新捕获时使结果失效。
 */
void DutyMeter_Task(void)
{
  uint32_t period_capture;
  uint32_t high_capture;

  if (MeasurementHw_TIM2ReadPwmCapture(&period_capture, &high_capture) == 0U)
  {
    if (duty_valid &&
        ((uint32_t)(HAL_GetTick() - last_duty_capture_tick_ms) > DUTY_TIMEOUT_MS))
    {
      DutyMeter_ResetCaptureState();
    }
    return;
  }

  /* 刚进入 PWM Input 或刚换 PSC 时，先丢弃第一轮，等待完整周期同步。 */
  if (duty_capture_synced == 0U)
  {
    duty_capture_synced = 1U;
    return;
  }

  if ((period_capture == 0U) || (high_capture > period_capture))
  {
    duty_valid = 0;
    return;
  }

  duty_period_ticks = period_capture;
  duty_high_ticks = high_capture;

  measured_duty_permille =
      (uint16_t)(((uint64_t)high_capture * 1000ULL +
                  period_capture / 2ULL) /
                 period_capture);

  last_duty_capture_tick_ms = HAL_GetTick();
  duty_valid = 1U;
}

/**
 * @brief 接收一轮 Gate 粗频率，并据此更新辅助频率/周期和 TIM2 自动量程。
 * @param frequency_hz 本轮 Gate 测得的频率，单位 Hz。
 */
void DutyMeter_OnGateMeasurement(uint32_t frequency_hz)
{
  uint16_t new_prescaler = DutyMeter_CalculatePrescaler(frequency_hz);

  measured_frequency_hz = frequency_hz;
  measured_period_ns = (frequency_hz != 0U)
                           ? (MEASUREMENT_NANOSECONDS_PER_SECOND + frequency_hz / 2U) /
                                 frequency_hz
                           : 0ULL;

  DutyMeter_ApplyPrescaler(new_prescaler);
}

/**
 * @brief 获取最终占空比结果。
 * @retval 占空比千分数，0~1000 对应 0.0%~100.0%。
 */
uint16_t DutyMeter_GetPermille(void)
{
  return measured_duty_permille;
}

/**
 * @brief 获取 DUTY 模式下由 Gate 提供的辅助频率结果。
 * @retval 当前频率，单位 Hz；尚无 Gate 结果时为 0。
 */
uint32_t DutyMeter_GetFrequencyHz(void)
{
  return measured_frequency_hz;
}

/**
 * @brief 获取 DUTY 模式下由辅助频率换算得到的周期结果。
 * @retval 当前周期，单位 ns；尚无有效辅助频率时为 0。
 */
uint64_t DutyMeter_GetPeriodNs(void)
{
  return measured_period_ns;
}

/**
 * @brief 判断当前占空比结果是否有效。
 * @retval 1=已经获得完整且未超时的 PWM Input 结果；0=结果无效。
 */
uint8_t DutyMeter_IsValid(void)
{
  return duty_valid;
}
