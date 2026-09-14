/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
 * InstrumentMode：仪器对外提供的功能模式。
 * 这个枚举回答的是“用户当前想测什么”，而不是“底层具体用哪种算法测”。
 */
typedef enum
{
  INSTRUMENT_MODE_FREQUENCY = 0, // 频率模式：最终输出频率，内部自动选择周期法或闸门法
  INSTRUMENT_MODE_PERIOD,        // 周期模式：复用频率测量引擎，最终输出周期
  INSTRUMENT_MODE_DUTY,          // 占空比模式：TIM2 切换为 PWM Input，测周期和高电平时间
  INSTRUMENT_MODE_INTERVAL       // 时间间隔模式：TIM2_CH1=A、TIM2_CH2=B，测 A -> B 时间差
} InstrumentMode;

/*
 * FrequencyMethod：FREQUENCY / PERIOD 模式内部使用的测量策略。
 * 这个枚举回答的是“当前用什么方法测频率”。
 */
typedef enum
{
  FREQUENCY_METHOD_PERIOD = 0, // 周期法：TIM2 捕获相邻上升沿，低频时分辨率更高
  FREQUENCY_METHOD_GATE        // 闸门法：TIM1 在 TIM4 的 1 秒 Gate 内数脉冲，高频时更合适
} FrequencyMethod;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* TIM2 输入时钟 72 MHz。普通时间戳模式使用 PSC=0。 */
#define TIM2_COUNTER_HZ 72000000ULL

#define NANOSECONDS_PER_SECOND 1000000000ULL

/* 题目最低频率 1 Hz，周期约 1000 ms。 */
#define FREQUENCY_TIMEOUT_MS 1500U

/* 题目 A -> B 时间间隔上限 100 ms；200 ms 后放弃本次 A。 */
#define INTERVAL_TIMEOUT_MS 200U

/* 周期法 -> 闸门法：10 kHz 对应 7200 个 72 MHz tick。 */
#define PERIOD_TO_GATE_TICKS 7200ULL

/* 闸门法 -> 周期法：形成 7~10 kHz 迟滞区。 */
#define GATE_TO_PERIOD_HZ 7000U

/*
 * DUTY 使用 TIM2 PWM Input：
 * CCR1 = 一个完整周期的计数值；CCR2 = 高电平时间计数值。
 * 目标让低频时每周期约不超过 60000 tick，给 16 位 CNT 留余量。
 */
#define DUTY_TARGET_PERIOD_TICKS 60000ULL
#define DUTY_DEFAULT_PRESCALER 1199U
#define DUTY_TIMEOUT_MS 1500U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// ==================== 仪器功能模式状态 ====================
static volatile InstrumentMode instrument_mode = INSTRUMENT_MODE_FREQUENCY; // 当前用户功能模式：频率 / 周期 / 占空比 / A->B
static volatile uint8_t instrument_mode_initialized = 0;                    // 0=尚未完成首次模式配置，1=已经配置过

// ==================== TIM2：周期法 / INTERVAL 共用扩展时间轴 ====================
static volatile uint64_t period_ticks = 0;         // 周期法测得的一个完整周期，单位：TIM2 tick
static volatile uint64_t period_ns = 0;            // period_ticks 换算后的周期，单位：ns
static volatile uint64_t timestamp1 = 0;           // 周期法上一次 CH1 捕获的 64 位扩展时间戳
static volatile uint64_t timestamp2 = 0;           // 周期法当前 CH1 捕获的 64 位扩展时间戳
static volatile uint8_t capture_state = 0;         // 周期法捕获状态：0=等待第一沿，1=已经有上一时间戳
static volatile uint32_t tim2_overflow_count = 0;  // TIM2 16 位 CNT 的软件溢出次数，用于扩展为长时间轴

// ==================== A -> B 时间间隔测量状态 ====================
static volatile uint64_t interval_start_timestamp = 0; // A(CH1) 到达时的 64 位扩展时间戳
static volatile uint64_t interval_end_timestamp = 0;   // B(CH2) 到达时的 64 位扩展时间戳
static volatile uint64_t interval_ticks = 0;           // B-A 的时间差，单位：TIM2 tick
static volatile uint64_t interval_ns = 0;              // interval_ticks 换算后的 A->B 时间间隔，单位：ns
static volatile uint8_t interval_waiting_ch2 = 0;      // 1=已经收到 A，当前正在等待 B；0=等待新的 A
static volatile uint32_t interval_start_tick_ms = 0;   // A 到达时的 HAL_GetTick()，用于判断等待 B 是否超时
static volatile uint8_t interval_valid = 0;            // 1=当前 interval_ns 来自完整且未超时的 A->B 配对

// ==================== TIM2 周期法得到的频率结果 ====================
static volatile uint32_t frequency_hz = 0;          // 周期法频率结果，单位：Hz（整数）
static volatile uint64_t frequency_millihz = 0;     // 周期法频率结果，单位：mHz，用于保留低频小数精度
static volatile uint32_t last_capture_tick_ms = 0;  // 最近一次 CH1 上升沿的 HAL tick，用于无信号超时判断
static volatile uint8_t frequency_valid = 0;        // 1=周期法当前 frequency/period 结果仍然有效
static volatile uint8_t period_requests_gate = 0;   // 1=周期已经短到应从 PERIOD_METHOD 切换到 GATE_METHOD

// ==================== FREQUENCY / PERIOD 共用自动测量策略 ====================
static volatile FrequencyMethod frequency_method = FREQUENCY_METHOD_PERIOD; // 当前内部测频策略：周期法或闸门法
static volatile uint32_t measured_frequency_hz = 0;                         // 对上层提供的最终频率结果，单位：Hz
static volatile uint64_t measured_period_ns = 0;                            // 对上层提供的最终周期结果，单位：ns

// ==================== DUTY：TIM2 PWM Input 硬件锁存结果 ====================
static volatile uint32_t duty_period_ticks = 0;                   // PWM Input 的 CCR1 周期值，单位：当前 TIM2 tick
static volatile uint32_t duty_high_ticks = 0;                     // PWM Input 的 CCR2 高电平时间，单位：当前 TIM2 tick
static volatile uint16_t measured_duty_permille = 0;              // 最终占空比千分数：0~1000 对应 0.0%~100.0%
static volatile uint16_t duty_prescaler = DUTY_DEFAULT_PRESCALER; // DUTY 模式当前 TIM2 PSC，用于自动量程
static volatile uint32_t last_duty_capture_tick_ms = 0;           // 最近一次有效 PWM Input 结果的 HAL tick，用于超时失效
static volatile uint8_t duty_capture_synced = 0;                  // 0=刚进入/重配 PWM Input 尚未同步完整周期，1=已同步
static volatile uint8_t duty_valid = 0;                           // 1=当前 duty_period/high/permille 是可信结果

// ==================== TIM1：1 秒硬件闸门内统计外部脉冲 ====================
static volatile uint32_t gate_frequency_hz = 0;   // TIM1 在 1 秒 Gate 内的总脉冲数；1 秒窗下即约等于 Hz
static volatile uint32_t tim1_overflow_count = 0; // TIM1 16 位外部计数器溢出次数，用于恢复完整脉冲总数
static volatile uint8_t gate_frequency_valid = 0; // 1=至少已经完成一轮有效的 1 秒闸门测量

// ==================== TIM4：1 秒 One Pulse 硬件闸门 ====================
static volatile uint8_t gate_ready = 0; // 1=TIM4 本轮 One Pulse 已结束，主循环可以读取 TIM1 快照

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Instrument_SetMode(InstrumentMode new_mode);                         // 统一仪器模式切换入口
static uint8_t Instrument_IsFrequencyPeriodMode(InstrumentMode mode);            // 判断模式是否属于 FREQUENCY / PERIOD 共享引擎
static void Gate_Stop(void);                                                     // 停止并清理 TIM1 + TIM4 闸门链
static void Gate_StartFresh(void);                                               // 从干净状态启动新一轮 1 秒闸门
static void TIM2_ConfigureTimestampCapture(uint8_t enable_ch2_interrupt);         // 配置 TIM2 普通时间戳输入捕获
static void TIM2_ConfigureDutyCapture(uint16_t prescaler);                        // 配置 TIM2 PWM Input 占空比捕获
static uint16_t Duty_CalculatePrescaler(uint32_t gate_frequency);                 // 根据粗频率计算 DUTY 模式 TIM2 PSC
static void Duty_ApplyPrescaler(uint16_t prescaler);                             // 应用新的 DUTY PSC 并使旧结果失效
static void Duty_ProcessCapture(void);                                            // 主循环读取 PWM Input CCR 并更新占空比
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * HAL / 定时器 API 阅读提示：
 * 1. HAL_xxx(...)：HAL 函数，通常维护软件状态并返回 HAL_StatusTypeDef。
 * 2. __HAL_xxx(...)：更接近寄存器操作的 HAL 宏。
 * 3. 函数名带 _IT：Interrupt，表示配合中断使用。
 */

/**
 * @brief 判断指定仪器模式是否属于 FREQUENCY / PERIOD 共享测量引擎。
 * @param mode 待判断的仪器功能模式。
 * @retval 1 表示是 FREQUENCY 或 PERIOD；0 表示是 DUTY 或 INTERVAL。
 * @note 这是纯判断函数，不修改任何硬件寄存器或测量状态。
 */
static uint8_t Instrument_IsFrequencyPeriodMode(InstrumentMode mode)
{
  return (mode == INSTRUMENT_MODE_FREQUENCY) ||
         (mode == INSTRUMENT_MODE_PERIOD);
}

/**
 * @brief 停止 TIM1 + TIM4 硬件闸门链，并把闸门运行状态清理到可重新启动的状态。
 * @note 会关闭 TIM1/TIM4 的 CEN，清除 Update 标志和 Pending IRQ，并把两个 CNT 清零。
 * @note 同时清除 gate_ready 和 tim1_overflow_count；不会主动清除 gate_frequency_hz。
 * @note 进入 INTERVAL 模式，或 Gate_StartFresh() 重新启动闸门前会调用本函数。
 */
static void Gate_Stop(void)
{
  __HAL_TIM_DISABLE(&htim4);
  __HAL_TIM_DISABLE(&htim1);

  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  HAL_NVIC_ClearPendingIRQ(TIM4_IRQn);
  HAL_NVIC_ClearPendingIRQ(TIM1_UP_IRQn);

  gate_ready = 0;
  tim1_overflow_count = 0;
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
}

/**
 * @brief 从干净状态启动新一轮 1 秒硬件闸门测量。
 * @note 先调用 Gate_Stop() 清理旧状态，再清除旧闸门结果有效性。
 * @note 启动顺序固定为：先使能 TIM1，再使能 TIM4；TIM4 TRGO 拉高后才真正打开 TIM1 Gated Mode。
 * @note 这样可以保证计数器已经准备好，再开始 1 秒时间窗，避免窗口起点丢脉冲。
 */
static void Gate_StartFresh(void)
{
  Gate_Stop();

  gate_frequency_hz = 0;
  gate_frequency_valid = 0;

  /* 先让 TIM1 CEN=1，但此时 TIM4 TRGO=LOW，Gated Mode 仍关闭。 */
  __HAL_TIM_ENABLE(&htim1);

  /* TIM4 开始 One Pulse 后，TRGO 拉高 1 秒，TIM1 Gate 打开。 */
  __HAL_TIM_ENABLE(&htim4);
}

/**
 * @brief 把 TIM2 配置为普通“自由运行时间轴 + 输入捕获”模式。
 * @param enable_ch2_interrupt 0=只启用 CH1 捕获中断，用于 FREQUENCY/PERIOD；1=同时启用 CH2，用于 INTERVAL。
 * @note CH1 直接映射 TI1/PA0，CH2 直接映射 TI2/PA1，两个通道均捕获上升沿。
 * @note TIM2 使用 PSC=0、ARR=65535，并开启 Update 中断，用 tim2_overflow_count 扩展 16 位 CNT。
 * @note 本函数也负责退出 DUTY 使用的 PWM Input Reset Mode，并清除旧标志、CNT 和溢出状态。
 */
static void TIM2_ConfigureTimestampCapture(uint8_t enable_ch2_interrupt)
{
  __HAL_TIM_DISABLE(&htim2);

  htim2.Instance->DIER = 0U;
  htim2.Instance->SR = 0U;

  /* 退出 PWM Input 的 Reset Mode。 */
  htim2.Instance->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_TS);

  /* CH1 直接映射 TI1；CH2 直接映射 TI2；DIV1；Filter=0。 */
  htim2.Instance->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;

  /* 两路都为上升沿输入捕获并保持通道使能。 */
  htim2.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;

  htim2.Init.Prescaler = 0U;
  htim2.Instance->PSC = 0U;
  htim2.Instance->ARR = 65535U;
  htim2.Instance->CNT = 0U;
  htim2.Instance->EGR = TIM_EGR_UG;
  htim2.Instance->SR = 0U;

  tim2_overflow_count = 0;

  /* 64 位扩展时间轴需要 Update 中断；CH1 必开，CH2 只在 INTERVAL 模式打开。 */
  htim2.Instance->DIER = TIM_IT_UPDATE | TIM_IT_CC1;
  if (enable_ch2_interrupt)
  {
    htim2.Instance->DIER |= TIM_IT_CC2;
  }

  HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
  __HAL_TIM_ENABLE(&htim2);
}

/**
 * @brief 把 TIM2 配置为 DUTY 模式使用的 PWM Input。
 * @param prescaler 写入 TIM2 PSC 的值；实际计数时钟分频系数为 prescaler + 1。
 * @note 同一根 PA0/TI1 同时进入两个捕获通道：CH1 上升沿锁存周期，CH2 下降沿锁存高电平时间。
 * @note TIM2 使用 Slave Reset Mode，每个上升沿会把 CNT 自动归零，为下一周期重新计时。
 * @note DUTY 模式关闭 CC1/CC2 与 Update 中断，主循环直接轮询捕获标志和 CCR，避免高频输入产生 ISR 风暴。
 * @note 每次重新配置后会清空 duty_valid，并要求重新同步至少一个完整周期。
 */
static void TIM2_ConfigureDutyCapture(uint16_t prescaler)
{
  __HAL_TIM_DISABLE(&htim2);

  htim2.Instance->DIER = 0U;
  htim2.Instance->SR = 0U;

  /* CH1=Direct TI1；CH2=Indirect TI1。 */
  htim2.Instance->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1;

  /* CH1 上升沿，CH2 下降沿。 */
  htim2.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;

  /* TI1FP1 作为触发源；Reset Mode：每个上升沿 CNT 自动归零。 */
  htim2.Instance->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_TS);
  htim2.Instance->SMCR |= TIM_SLAVEMODE_RESET | TIM_TS_TI1FP1;

  htim2.Init.Prescaler = prescaler;
  htim2.Instance->PSC = prescaler;
  htim2.Instance->ARR = 65535U;
  htim2.Instance->CNT = 0U;
  htim2.Instance->EGR = TIM_EGR_UG;
  htim2.Instance->SR = 0U;

  duty_prescaler = prescaler;
  duty_capture_synced = 0;
  duty_valid = 0;
  duty_period_ticks = 0;
  duty_high_ticks = 0;
  measured_duty_permille = 0;
  last_duty_capture_tick_ms = 0;

  HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
  __HAL_TIM_ENABLE(&htim2);
}

/**
 * @brief 根据 1 秒闸门得到的粗频率，计算 DUTY 模式下一轮应使用的 TIM2 PSC。
 * @param gate_frequency TIM1 + TIM4 1 秒 Gate 得到的粗频率，单位：Hz。
 * @retval 适合写入 TIM2 PSC 寄存器的值，范围 0~65535。
 * @note 目标是让一个 PWM 周期尽量不超过 DUTY_TARGET_PERIOD_TICKS（当前 60000 tick）。
 * @note 对 gate_frequency 使用 -1 的保守下界，避免低频 Gate 的 ±1 count 误差导致 PSC 选得过小而溢出。
 * @note gate_frequency=0 时无法估算新量程，因此保持当前 duty_prescaler 不变。
 */
static uint16_t Duty_CalculatePrescaler(uint32_t gate_frequency)
{
  uint32_t frequency_lower_bound; // 对 gate_frequency 做 -1 后得到的保守频率下界，防止低估所需 PSC
  uint64_t denominator;           // 计算目标分频系数时的分母：频率下界 × 目标周期 tick
  uint64_t divider;               // 实际分频系数 PSC+1，最后会转换回寄存器中的 PSC

  if (gate_frequency == 0U)
  {
    return duty_prescaler;
  }

  frequency_lower_bound = (gate_frequency > 1U) ? (gate_frequency - 1U) : 1U;

  denominator =
      (uint64_t)frequency_lower_bound * DUTY_TARGET_PERIOD_TICKS;

  divider = (TIM2_COUNTER_HZ + denominator - 1ULL) / denominator;

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
 * @brief 在 DUTY 模式中应用新的 TIM2 PSC，并让旧占空比结果立即失效。
 * @param prescaler 新的 TIM2 PSC 寄存器值。
 * @note 如果新 PSC 与当前 duty_prescaler 相同，则直接返回，不重新配置 TIM2。
 * @note PSC 改变后一个 tick 的时间尺度已经变化，因此旧 CCR、旧占空比结果不能继续使用。
 * @note 本函数会重新产生 Update Event 让 PSC 生效，并把 duty_capture_synced 置 0，等待新的完整周期重新同步。
 */
static void Duty_ApplyPrescaler(uint16_t prescaler)
{
  if (prescaler == duty_prescaler)
  {
    return;
  }

  __HAL_TIM_DISABLE(&htim2);

  htim2.Init.Prescaler = prescaler;
  htim2.Instance->PSC = prescaler;
  htim2.Instance->CNT = 0U;
  htim2.Instance->EGR = TIM_EGR_UG;
  htim2.Instance->SR = 0U;

  duty_prescaler = prescaler;
  duty_capture_synced = 0;
  duty_valid = 0;
  duty_period_ticks = 0;
  duty_high_ticks = 0;
  measured_duty_permille = 0;
  last_duty_capture_tick_ms = 0;

  __HAL_TIM_ENABLE(&htim2);
}

/**
 * @brief 在主循环中轮询 TIM2 PWM Input 的捕获结果，并更新占空比测量值。
 * @note 只在 INSTRUMENT_MODE_DUTY 下由 while(1) 调用，不依赖输入捕获 ISR。
 * @note 只有 CC1 和 CC2 都出现新捕获后才同时读取 CCR1/CCR2，分别作为周期和高电平时间。
 * @note 进入 DUTY 或修改 PSC 后的第一轮捕获会被丢弃，用于确保 CCR1/CCR2 已经对应完整且同步的 PWM 周期。
 * @note 有效结果最终写入 duty_period_ticks、duty_high_ticks、measured_duty_permille，并设置 duty_valid=1。
 * @note 超过 DUTY_TIMEOUT_MS 没有新的完整捕获时，会使旧占空比结果失效。
 */
static void Duty_ProcessCapture(void)
{
  uint32_t period_capture; // 本次从 CCR1 读取到的完整 PWM 周期 tick
  uint32_t high_capture;   // 本次从 CCR2 读取到的高电平时间 tick

  /* 必须至少同时看到一次周期捕获和一次下降沿捕获。 */
  if ((__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1) == RESET) ||
      (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC2) == RESET))
  {
    if (duty_valid &&
        ((uint32_t)(HAL_GetTick() - last_duty_capture_tick_ms) > DUTY_TIMEOUT_MS))
    {
      duty_valid = 0;
      duty_period_ticks = 0;
      duty_high_ticks = 0;
      measured_duty_permille = 0;
    }
    return;
  }

  period_capture = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
  high_capture = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);

  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1 | TIM_FLAG_CC2 |
                               TIM_FLAG_CC1OF | TIM_FLAG_CC2OF);

  /*
   * 进入 PWM Input 后第一次 CCR1 可能只是“模式切换时刻 -> 第一个上升沿”，
   * 不是完整周期。先丢弃一轮，下一轮才开始认为 CCR1/CCR2 已同步。
   */
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
 * @brief 统一切换仪器功能模式，并完成对应 Timer 资源的重配置和状态清理。
 * @param new_mode 目标仪器模式：FREQUENCY、PERIOD、DUTY 或 INTERVAL。
 * @note FREQUENCY 与 PERIOD 共用完全相同的硬件测量链，互相切换时只改变 instrument_mode，不重启定时器。
 * @note 切入 DUTY 时 TIM2 改成 PWM Input，TIM1+TIM4 保留 1 秒 Gate 用于粗频率和自动 PSC。
 * @note 切入 INTERVAL 时 TIM2 使用 CH1=A、CH2=B，同时停止 TIM1+TIM4 Gate 链。
 * @note 涉及 TIM2 重配置时会暂时屏蔽 TIM2_IRQn，防止配置中途进入 ISR 产生竞态。
 * @note 这是未来按键、菜单、串口等上层交互切换测量功能时应调用的唯一入口。
 */
static void Instrument_SetMode(InstrumentMode new_mode)
{
  uint32_t frequency_hint = measured_frequency_hz; // 切入 DUTY 前保留上一频率结果，用于第一次选择合适的 TIM2 PSC

  if (instrument_mode_initialized && (new_mode == instrument_mode))
  {
    return;
  }

  /* FREQUENCY <-> PERIOD 共用完全相同的硬件，只改变最终展示含义。 */
  if (instrument_mode_initialized &&
      Instrument_IsFrequencyPeriodMode(instrument_mode) &&
      Instrument_IsFrequencyPeriodMode(new_mode))
  {
    instrument_mode = new_mode;
    return;
  }

  HAL_NVIC_DisableIRQ(TIM2_IRQn);

  /* 切模式前先清掉所有上层状态。 */
  capture_state = 0;
  timestamp1 = 0;
  timestamp2 = 0;
  tim2_overflow_count = 0;
  frequency_valid = 0;
  period_requests_gate = 0;
  period_ticks = 0;
  period_ns = 0;

  interval_waiting_ch2 = 0;
  interval_start_timestamp = 0;
  interval_end_timestamp = 0;
  interval_start_tick_ms = 0;
  interval_ticks = 0;
  interval_ns = 0;
  interval_valid = 0;

  duty_capture_synced = 0;
  duty_valid = 0;
  duty_period_ticks = 0;
  duty_high_ticks = 0;
  measured_duty_permille = 0;
  last_duty_capture_tick_ms = 0;

  if (new_mode == INSTRUMENT_MODE_INTERVAL)
  {
    Gate_Stop();
    TIM2_ConfigureTimestampCapture(1U);
  }
  else if (new_mode == INSTRUMENT_MODE_DUTY)
  {
    /*
     * 如果切换前已经有频率结果，直接用它挑一个更合适的 PSC；
     * 否则用 1199，确保 1 Hz 周期也不会超过 16 位范围。
     */
    duty_prescaler = (frequency_hint != 0U)
                         ? Duty_CalculatePrescaler(frequency_hint)
                         : DUTY_DEFAULT_PRESCALER;

    TIM2_ConfigureDutyCapture(duty_prescaler);

    measured_frequency_hz = 0;
    measured_period_ns = 0;
    gate_frequency_hz = 0;
    gate_frequency_valid = 0;
    Gate_StartFresh();
  }
  else
  {
    frequency_method = FREQUENCY_METHOD_PERIOD;
    measured_frequency_hz = 0;
    measured_period_ns = 0;
    frequency_hz = 0;
    frequency_millihz = 0;
    gate_frequency_hz = 0;
    gate_frequency_valid = 0;

    TIM2_ConfigureTimestampCapture(0U);
    Gate_StartFresh();
  }

  instrument_mode = new_mode;
  instrument_mode_initialized = 1;

  HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* TIM2 先通过 HAL 启动一次；后续 Instrument_SetMode() 只直接切换寄存器工作方式。 */
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /* TIM3_CH1：约 1 kHz、50% 占空比测试 PWM。 */
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* TIM1 / TIM4 的 Update 中断通过 HAL 启动一次。 */
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  HAL_NVIC_ClearPendingIRQ(TIM1_UP_IRQn);

  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_TIM_SET_COUNTER(&htim4, 0);
  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
  HAL_NVIC_ClearPendingIRQ(TIM4_IRQn);

  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  /* 默认进入频率模式；后续按键/菜单统一调用 Instrument_SetMode()。 */
  Instrument_SetMode(INSTRUMENT_MODE_FREQUENCY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* DUTY：PWM Input 全硬件捕获，主循环只读取最新 CCR。 */
    if (instrument_mode == INSTRUMENT_MODE_DUTY)
    {
      Duty_ProcessCapture();
    }

    /* INTERVAL：A 已到但 200 ms 内没有 B，就丢弃本次 A 并重新同步。 */
    if ((instrument_mode == INSTRUMENT_MODE_INTERVAL) &&
        interval_waiting_ch2 &&
        ((uint32_t)(HAL_GetTick() - interval_start_tick_ms) > INTERVAL_TIMEOUT_MS))
    {
      HAL_NVIC_DisableIRQ(TIM2_IRQn);

      if ((instrument_mode == INSTRUMENT_MODE_INTERVAL) &&
          interval_waiting_ch2 &&
          ((uint32_t)(HAL_GetTick() - interval_start_tick_ms) > INTERVAL_TIMEOUT_MS))
      {
        interval_waiting_ch2 = 0;
        interval_valid = 0;
        interval_start_timestamp = 0;
        interval_end_timestamp = 0;
        interval_start_tick_ms = 0;
        interval_ticks = 0;
        interval_ns = 0;
      }

      HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }

    /* FREQUENCY / PERIOD：周期法结果超时。 */
    if (Instrument_IsFrequencyPeriodMode(instrument_mode) &&
        (frequency_method == FREQUENCY_METHOD_PERIOD) &&
        frequency_valid &&
        ((uint32_t)(HAL_GetTick() - last_capture_tick_ms) > FREQUENCY_TIMEOUT_MS))
    {
      HAL_NVIC_DisableIRQ(TIM2_IRQn);

      if (Instrument_IsFrequencyPeriodMode(instrument_mode) &&
          (frequency_method == FREQUENCY_METHOD_PERIOD) &&
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

      HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }

    /* FREQUENCY / PERIOD 内部：周期太短时切到闸门法。 */
    if (Instrument_IsFrequencyPeriodMode(instrument_mode) &&
        (frequency_method == FREQUENCY_METHOD_PERIOD) &&
        period_requests_gate)
    {
      measured_frequency_hz = frequency_hz;

      __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
      __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);

      frequency_method = FREQUENCY_METHOD_GATE;
      period_requests_gate = 0;
      frequency_valid = 0;
      capture_state = 0;
    }

    /* FREQUENCY / PERIOD / DUTY 都需要 TIM1+TIM4 的 1 秒闸门结果。 */
    if ((instrument_mode != INSTRUMENT_MODE_INTERVAL) && gate_ready)
    {
      HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);

      uint32_t overflow_snapshot = tim1_overflow_count;          // 读取快照时已经由 ISR 记录的 TIM1 溢出次数
      uint32_t counter_snapshot = __HAL_TIM_GET_COUNTER(&htim1); // Gate 结束后 TIM1 当前 16 位 CNT 快照

      if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET)
      {
        overflow_snapshot++;
      }

      gate_frequency_hz =
          overflow_snapshot * 65536UL + counter_snapshot;
      gate_frequency_valid = 1;

      if (Instrument_IsFrequencyPeriodMode(instrument_mode))
      {
        if (frequency_method == FREQUENCY_METHOD_GATE)
        {
          measured_frequency_hz = gate_frequency_hz;

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

            __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
            __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
          }
        }
      }
      else if (instrument_mode == INSTRUMENT_MODE_DUTY)
      {
        uint16_t new_prescaler = Duty_CalculatePrescaler(gate_frequency_hz); // 根据本轮粗频率计算 DUTY 下一轮应使用的 TIM2 PSC

        measured_frequency_hz = gate_frequency_hz;
        measured_period_ns = (gate_frequency_hz != 0U)
                                 ? (NANOSECONDS_PER_SECOND + gate_frequency_hz / 2U) /
                                       gate_frequency_hz
                                 : 0ULL;

        Duty_ApplyPrescaler(new_prescaler);
      }

      /* 下一轮 1 秒 Gate。 */
      tim1_overflow_count = 0;
      __HAL_TIM_SET_COUNTER(&htim1, 0);
      __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
      HAL_NVIC_ClearPendingIRQ(TIM1_UP_IRQn);

      gate_ready = 0;
      HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

      __HAL_TIM_SET_COUNTER(&htim4, 0);
      __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
      HAL_NVIC_ClearPendingIRQ(TIM4_IRQn);
      __HAL_TIM_ENABLE(&htim4);
    }

    /* FREQUENCY / PERIOD 的最终统一输出。 */
    if (Instrument_IsFrequencyPeriodMode(instrument_mode))
    {
      if (frequency_method == FREQUENCY_METHOD_PERIOD)
      {
        if (frequency_valid)
        {
          measured_frequency_hz = frequency_hz;
          measured_period_ns = period_ns;
        }
        else if (gate_frequency_valid)
        {
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

      if ((frequency_method == FREQUENCY_METHOD_GATE) &&
          (measured_frequency_hz != 0U))
      {
        measured_period_ns =
            (NANOSECONDS_PER_SECOND + measured_frequency_hz / 2U) /
            measured_frequency_hz;
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief TIM2 输入捕获中断的 HAL 回调，处理周期法和 A->B 时间间隔测量。
 * @param htim 触发本次回调的定时器句柄；本项目只处理 TIM2，其他 TIM 会立即返回。
 * @note FREQUENCY / PERIOD：只使用 CH1，连续两个上升沿构造 64 位时间戳并计算周期、频率。
 * @note INTERVAL：CH1 作为 A 起点，CH2 作为 B 终点，计算 B-A。
 * @note DUTY：不会使用本回调，因为 PWM Input 的 CC1/CC2 中断在 DUTY 模式中被关闭，主循环直接读 CCR。
 * @note 回调内部处理 UIF 与 Capture 几乎同时发生的竞态，避免溢出边界导致 64 位时间戳错一圈 65536 tick。
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM2)
  {
    return;
  }

  if (instrument_mode == INSTRUMENT_MODE_DUTY)
  {
    return;
  }

  /* ==================== TIM2_CH1 / PA0 ==================== */
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uint32_t capture =
        HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // 本次 CH1 边沿到来时硬件锁存在 CCR1 中的 16 位 CNT

    uint32_t overflow_snapshot = tim2_overflow_count; // 与本次 CCR1 配对的软件溢出次数快照

    if ((__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) &&
        (capture < 32768U))
    {
      overflow_snapshot++;
    }

    uint64_t current_timestamp =
        (uint64_t)overflow_snapshot * 65536ULL + capture; // 将 overflow + CCR1 合成为本次 CH1 的 64 位扩展时间戳

    if (instrument_mode == INSTRUMENT_MODE_INTERVAL)
    {
      if (interval_waiting_ch2 == 0U)
      {
        interval_valid = 0;
        interval_ticks = 0;
        interval_ns = 0;
        interval_end_timestamp = 0;
        interval_start_timestamp = current_timestamp;
        interval_start_tick_ms = HAL_GetTick();
        interval_waiting_ch2 = 1U;
      }
      return;
    }

    /* FREQUENCY / PERIOD */
    last_capture_tick_ms = HAL_GetTick();

    if (capture_state == 0U)
    {
      timestamp1 = current_timestamp;
      capture_state = 1U;
    }
    else
    {
      timestamp2 = current_timestamp;
      period_ticks = timestamp2 - timestamp1;

      if (period_ticks != 0ULL)
      {
        frequency_hz =
            (uint32_t)((TIM2_COUNTER_HZ + period_ticks / 2ULL) /
                       period_ticks);

        frequency_millihz =
            (TIM2_COUNTER_HZ * 1000ULL +
             period_ticks / 2ULL) /
            period_ticks;

        period_ns =
            (period_ticks * NANOSECONDS_PER_SECOND +
             TIM2_COUNTER_HZ / 2ULL) /
            TIM2_COUNTER_HZ;

        frequency_valid = 1U;

        if (period_ticks <= PERIOD_TO_GATE_TICKS)
        {
          period_requests_gate = 1U;
          __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
        }
      }

      timestamp1 = timestamp2;
    }
  }

  /* ==================== TIM2_CH2 / PA1 ==================== */
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    if (instrument_mode != INSTRUMENT_MODE_INTERVAL)
    {
      return;
    }

    uint32_t capture =
        HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // 本次 B(CH2) 上升沿到来时 CCR2 锁存的 16 位 CNT

    uint32_t overflow_snapshot = tim2_overflow_count; // 与本次 CCR2 配对的软件溢出次数快照

    if ((__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) &&
        (capture < 32768U))
    {
      overflow_snapshot++;
    }

    uint64_t current_timestamp =
        (uint64_t)overflow_snapshot * 65536ULL + capture; // 将 overflow + CCR2 合成为 B 的 64 位扩展时间戳

    if (interval_waiting_ch2 == 1U)
    {
      if ((uint32_t)(HAL_GetTick() - interval_start_tick_ms) > INTERVAL_TIMEOUT_MS)
      {
        interval_waiting_ch2 = 0;
        interval_valid = 0;
        interval_start_timestamp = 0;
        interval_end_timestamp = 0;
        interval_start_tick_ms = 0;
        interval_ticks = 0;
        interval_ns = 0;
        return;
      }

      interval_end_timestamp = current_timestamp;
      interval_ticks =
          interval_end_timestamp - interval_start_timestamp;

      interval_ns =
          (interval_ticks * NANOSECONDS_PER_SECOND +
           TIM2_COUNTER_HZ / 2ULL) /
          TIM2_COUNTER_HZ;

      interval_valid = 1U;
      interval_waiting_ch2 = 0U;
    }
  }
}

/**
 * @brief HAL 定时器 Update/Period Elapsed 回调，统一处理 TIM1、TIM2、TIM4 的更新事件。
 * @param htim 触发 Update 事件的定时器句柄。
 * @note TIM1：16 位外部脉冲计数器溢出，tim1_overflow_count++。
 * @note TIM2：普通时间戳模式下 16 位 CNT 溢出，tim2_overflow_count++；DUTY 模式不会使用该 Update 中断。
 * @note TIM4：1 秒 One Pulse Gate 结束，置 gate_ready=1，让主循环快照 TIM1 并计算闸门频率。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    tim1_overflow_count++;
  }
  else if (htim->Instance == TIM2)
  {
    /* DUTY 模式关闭了 TIM2 Update 中断，这里理论上不会进入。 */
    if (instrument_mode != INSTRUMENT_MODE_DUTY)
    {
      tim2_overflow_count++;
    }
  }
  else if (htim->Instance == TIM4)
  {
    if (instrument_mode != INSTRUMENT_MODE_INTERVAL)
    {
      gate_ready = 1U;
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: source line number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s, line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
