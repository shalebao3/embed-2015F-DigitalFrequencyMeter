#include "instrument.h"

#include "main.h"
#include "tim.h"

#include "duty_meter.h"
#include "frequency_meter.h"
#include "interval_meter.h"
#include "measurement_hw.h"

/* 当前用户功能模式。 */
static volatile InstrumentMode instrument_mode = INSTRUMENT_MODE_FREQUENCY;   //  当前仪器功能模式，默认 FREQUENCY
static volatile uint8_t instrument_mode_initialized = 0;  //  模式是否已初始化，0=未初始化，1=已初始化

/* 最近一次可用频率；经过 INTERVAL 再切入 DUTY 时也可用于第一次 PSC 选档。 */
static uint32_t last_frequency_hint_hz = 0;

/**
 * @brief 判断给定模式是否属于 FREQUENCY / PERIOD 共用测量引擎。
 * @param mode 待判断的仪器功能模式。
 * @retval 1=FREQUENCY 或 PERIOD；0=其他模式。
 */
static uint8_t Instrument_IsFrequencyPeriodMode(InstrumentMode mode)
{
  return (uint8_t)((mode == INSTRUMENT_MODE_FREQUENCY) ||
                   (mode == INSTRUMENT_MODE_PERIOD));  // 1=FREQUENCY/PERIOD，0=其他模式
}

/**
 * @brief 初始化底层测量硬件，并默认进入 FREQUENCY 模式。
 * @note 先启动共享 Timer，再暂时屏蔽 TIM2_IRQn 完成频率测量引擎初始化，最后恢复中断。
 */
void Instrument_Init(void)
{
  instrument_mode = INSTRUMENT_MODE_FREQUENCY;
  instrument_mode_initialized = 0;
  last_frequency_hint_hz = 0;

  MeasurementHw_Init();  // 启动 TIM1/TIM2/TIM3/TIM4 时钟源

  MeasurementHw_TIM2IrqDisable();  // 临时屏蔽 TIM2_IRQn （ TIM2 中断请求 ），避免频率测量引擎初始化期间被捕获中断打断
  FrequencyMeter_Start();
  instrument_mode_initialized = 1U;
  MeasurementHw_TIM2IrqEnable();
}

/**
 * @brief 统一切换仪器功能模式。
 * @param new_mode 目标 InstrumentMode。
 * @note FREQUENCY <-> PERIOD 共享完全相同的硬件，只改变上层显示语义。
 * @note 其他切换会暂时屏蔽 TIM2_IRQn，再由目标模块负责重配 TIM2 和 Gate 硬件。
 */
void Instrument_SetMode(InstrumentMode new_mode)
{
  uint32_t frequency_hint = last_frequency_hint_hz;
  uint32_t current_frequency;

  if (instrument_mode_initialized && (new_mode == instrument_mode))
  {
    return;
  }

  if (instrument_mode_initialized &&
      Instrument_IsFrequencyPeriodMode(instrument_mode) &&
      Instrument_IsFrequencyPeriodMode(new_mode))
  {
    instrument_mode = new_mode;
    return;
  }

  /* 离开有频率语义的模式前，保存最近频率给 DUTY 首次自动量程使用。 */
  if (Instrument_IsFrequencyPeriodMode(instrument_mode))
  {
    current_frequency = FrequencyMeter_GetFrequencyHz();
    if (current_frequency != 0U)
    {
      frequency_hint = current_frequency;
    }
  }
  else if (instrument_mode == INSTRUMENT_MODE_DUTY)
  {
    current_frequency = DutyMeter_GetFrequencyHz();
    if (current_frequency != 0U)
    {
      frequency_hint = current_frequency;
    }
  }

  last_frequency_hint_hz = frequency_hint;
  MeasurementHw_TIM2IrqDisable();

  if (new_mode == INSTRUMENT_MODE_INTERVAL)
  {
    IntervalMeter_Start();
  }
  else if (new_mode == INSTRUMENT_MODE_DUTY)
  {
    DutyMeter_Start(frequency_hint);
  }
  else
  {
    FrequencyMeter_Start();
  }

  instrument_mode = new_mode;
  instrument_mode_initialized = 1U;
  MeasurementHw_TIM2IrqEnable();
}

/**
 * @brief 仪器应用层主任务，应在 main() 的 while(1) 中持续调用。
 * @note 根据当前功能模式执行对应算法任务，并统一读取和分发 TIM1+TIM4 的 Gate 测频结果。
 */
void Instrument_Task(void)
{
  uint32_t gate_frequency_hz;  //  频率结果
  uint32_t current_frequency;  //  频率结果
  uint8_t gate_updated = 0U;   //  Gate 是否更新

  if (Instrument_IsFrequencyPeriodMode(instrument_mode))
  {
    FrequencyMeter_Task();
  }
  else if (instrument_mode == INSTRUMENT_MODE_DUTY)
  {
    DutyMeter_Task();
  }
  else
  {
    IntervalMeter_Task();
  }

  /* FREQUENCY / PERIOD / DUTY 共用 TIM1+TIM4 的 1 秒 Gate。 */
  if ((instrument_mode != INSTRUMENT_MODE_INTERVAL) &&
      MeasurementHw_GateTakeFrequency(&gate_frequency_hz))
  {
    gate_updated = 1U;

    if (Instrument_IsFrequencyPeriodMode(instrument_mode))
    {
      FrequencyMeter_OnGateMeasurement(gate_frequency_hz);
    }
    else
    {
      DutyMeter_OnGateMeasurement(gate_frequency_hz);
    }
  }

  /* Gate 事件可能改变频率策略或 fallback，立即再整理一次最终输出。 */
  if (gate_updated && Instrument_IsFrequencyPeriodMode(instrument_mode))
  {
    FrequencyMeter_Task();
  }

  if (Instrument_IsFrequencyPeriodMode(instrument_mode))
  {
    current_frequency = FrequencyMeter_GetFrequencyHz();
  }
  else if (instrument_mode == INSTRUMENT_MODE_DUTY)
  {
    current_frequency = DutyMeter_GetFrequencyHz();
  }
  else
  {
    current_frequency = 0U;
  }

  if (current_frequency != 0U)
  {
    last_frequency_hint_hz = current_frequency;
  }
}

/**
 * @brief 获取当前仪器功能模式。
 * @retval 当前 InstrumentMode 枚举值。
 */
InstrumentMode Instrument_GetMode(void)
{
  return instrument_mode;
}

/**
 * @brief 获取当前模式可提供的频率结果。
 * @retval FREQUENCY/PERIOD 模式返回 FrequencyMeter 结果；DUTY 返回辅助频率；INTERVAL 返回 0。
 */
uint32_t Instrument_GetFrequencyHz(void)
{
  if (Instrument_IsFrequencyPeriodMode(instrument_mode))
  {
    return FrequencyMeter_GetFrequencyHz();
  }

  if (instrument_mode == INSTRUMENT_MODE_DUTY)
  {
    return DutyMeter_GetFrequencyHz();
  }

  return 0U;
}

/**
 * @brief 获取当前模式可提供的周期结果。
 * @retval FREQUENCY/PERIOD 模式返回 FrequencyMeter 结果；DUTY 返回辅助周期；INTERVAL 返回 0，单位 ns。
 */
uint64_t Instrument_GetPeriodNs(void)
{
  if (Instrument_IsFrequencyPeriodMode(instrument_mode))
  {
    return FrequencyMeter_GetPeriodNs();
  }

  if (instrument_mode == INSTRUMENT_MODE_DUTY)
  {
    return DutyMeter_GetPeriodNs();
  }

  return 0ULL;
}

/**
 * @brief 获取当前占空比结果。
 * @retval DUTY 模式返回 0~1000 的占空比千分数；其他模式返回 0。
 */
uint16_t Instrument_GetDutyPermille(void)
{
  return (instrument_mode == INSTRUMENT_MODE_DUTY)
             ? DutyMeter_GetPermille()
             : 0U;
}

/**
 * @brief 获取当前 A->B 时间间隔结果。
 * @retval INTERVAL 模式返回时间间隔，单位 ns；其他模式返回 0。
 */
uint64_t Instrument_GetIntervalNs(void)
{
  return (instrument_mode == INSTRUMENT_MODE_INTERVAL)
             ? IntervalMeter_GetNs()
             : 0ULL;
}

/**
 * @brief 判断当前功能模式的最终测量结果是否有效。
 * @retval 1=当前模式存在有效结果；0=当前结果无效。
 */
uint8_t Instrument_IsResultValid(void)
{
  if (Instrument_IsFrequencyPeriodMode(instrument_mode))
  {
    return FrequencyMeter_IsValid();
  }

  if (instrument_mode == INSTRUMENT_MODE_DUTY)
  {
    return DutyMeter_IsValid();
  }

  return IntervalMeter_IsValid();
}

/**
 * @brief TIM2 输入捕获 HAL 回调，把捕获事件路由到当前业务模块。
 * @param htim 产生输入捕获中断的 Timer 句柄。
 * @note 仅处理 TIM2；DUTY 不使用 Capture ISR，而是由主循环轮询 CCR1/CCR2。
 * @note 时间戳扩展由 measurement_hw 完成，算法由 frequency/interval 模块完成。
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  uint64_t timestamp;

  if (htim->Instance != TIM2)
  {
    return;
  }

  /* DUTY 不使用 Capture ISR；CCR1/CCR2 由主循环轮询。 */
  if (instrument_mode == INSTRUMENT_MODE_DUTY)
  {
    return;
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    timestamp = MeasurementHw_TIM2ReadCapturedTimestamp(TIM_CHANNEL_1);

    if (instrument_mode == INSTRUMENT_MODE_INTERVAL)
    {
      IntervalMeter_OnCaptureA(timestamp);
    }
    else
    {
      FrequencyMeter_OnCapture(timestamp);
    }
  }
  else if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) &&
           (instrument_mode == INSTRUMENT_MODE_INTERVAL))
  {
    timestamp = MeasurementHw_TIM2ReadCapturedTimestamp(TIM_CHANNEL_2);
    IntervalMeter_OnCaptureB(timestamp);
  }
}
