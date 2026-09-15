#include "interval_meter.h"

#include "main.h"
#include "measurement_hw.h"

/* 题目 A->B 上限 100 ms；200 ms 后放弃旧 A 并重新同步。 */
#define INTERVAL_TIMEOUT_MS 200U

static volatile uint64_t interval_start_timestamp = 0; // A(CH1) 的 64 位时间戳
static volatile uint64_t interval_end_timestamp = 0;   // B(CH2) 的 64 位时间戳
static volatile uint64_t interval_ticks = 0;           // B-A，单位 TIM2 tick
static volatile uint64_t interval_ns = 0;              // B-A，单位 ns
static volatile uint8_t interval_waiting_ch2 = 0;      // 1=已经有 A，等待 B
static volatile uint32_t interval_start_tick_ms = 0;   // A 到达时 HAL tick
static volatile uint8_t interval_valid = 0;            // 1=当前结果来自完整、未超时 A->B 配对

/**
 * @brief 清空 A->B 时间间隔测量的软件状态，不修改 Timer 配置。
 * @note 清空后重新回到等待下一次 A(CH1) 上升沿的状态。
 */
static void IntervalMeter_ResetState(void)
{
  interval_start_timestamp = 0;
  interval_end_timestamp = 0;
  interval_ticks = 0;
  interval_ns = 0;
  interval_waiting_ch2 = 0;
  interval_start_tick_ms = 0;
  interval_valid = 0;
}

/**
 * @brief 启动 A->B 时间间隔测量。
 * @note 关闭 TIM1+TIM4 频率 Gate，并把 TIM2 配置为 CH1+CH2 双路时间戳捕获。
 */
void IntervalMeter_Start(void)
{
  IntervalMeter_ResetState();
  MeasurementHw_GateStop();
  MeasurementHw_TIM2ConfigureTimestampCapture(1U);
}

/**
 * @brief 处理 A(CH1) 上升沿到达事件。
 * @param timestamp A 到达时由 measurement_hw 扩展后的 64 位 TIM2 时间戳，单位为 TIM2 tick。
 * @note 已经在等待 B 时忽略新的 A；直到 B 到来或超时才重新等待下一次 A。
 */
void IntervalMeter_OnCaptureA(uint64_t timestamp)
{
  if (interval_waiting_ch2 != 0U)
  {
    return;
  }

  /* 新测量开始后旧结果立即失效。 */
  interval_valid = 0;
  interval_ticks = 0;
  interval_ns = 0;
  interval_end_timestamp = 0;

  interval_start_timestamp = timestamp;
  interval_start_tick_ms = HAL_GetTick();
  interval_waiting_ch2 = 1U;
}

/**
 * @brief 处理 B(CH2) 上升沿到达事件，并完成 B-A 时间间隔计算。
 * @param timestamp B 到达时由 measurement_hw 扩展后的 64 位 TIM2 时间戳，单位为 TIM2 tick。
 * @note ISR 内再次检查 200 ms 超时，避免迟到的 B 与旧 A 错配。
 */
void IntervalMeter_OnCaptureB(uint64_t timestamp)
{
  if (interval_waiting_ch2 == 0U)
  {
    return;
  }

  if ((uint32_t)(HAL_GetTick() - interval_start_tick_ms) > INTERVAL_TIMEOUT_MS)
  {
    IntervalMeter_ResetState();
    return;
  }

  interval_end_timestamp = timestamp;
  interval_ticks = interval_end_timestamp - interval_start_timestamp;
  interval_ns =
      (interval_ticks * MEASUREMENT_NANOSECONDS_PER_SECOND +
       MEASUREMENT_TIM2_COUNTER_HZ / 2ULL) /
      MEASUREMENT_TIM2_COUNTER_HZ;

  interval_valid = 1U;
  interval_waiting_ch2 = 0U;
}

/**
 * @brief INTERVAL 模式主循环任务，处理“已有 A、长期没有 B”的超时重新同步。
 * @note 超时判断期间会短暂屏蔽 TIM2_IRQn，并在屏蔽后再次复查状态，避免与 B 捕获中断竞争。
 */
void IntervalMeter_Task(void)
{
  if (interval_waiting_ch2 &&
      ((uint32_t)(HAL_GetTick() - interval_start_tick_ms) > INTERVAL_TIMEOUT_MS))
  {
    MeasurementHw_TIM2IrqDisable();

    /* 屏蔽 TIM2 IRQ 后复查一次，避免 B 恰好在超时判断期间到来。 */
    if (interval_waiting_ch2 &&
        ((uint32_t)(HAL_GetTick() - interval_start_tick_ms) > INTERVAL_TIMEOUT_MS))
    {
      IntervalMeter_ResetState();
    }

    MeasurementHw_TIM2IrqEnable();
  }
}

/**
 * @brief 获取最近一次有效的 A->B 时间间隔结果。
 * @retval A->B 时间间隔，单位 ns；尚无有效结果时通常为 0。
 */
uint64_t IntervalMeter_GetNs(void)
{
  return interval_ns;
}

/**
 * @brief 判断当前 A->B 时间间隔结果是否有效。
 * @retval 1=存在完整且未超时的 A->B 配对结果；0=当前结果无效。
 */
uint8_t IntervalMeter_IsValid(void)
{
  return interval_valid;
}
