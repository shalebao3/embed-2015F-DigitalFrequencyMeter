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

typedef enum
{
  INSTRUMENT_MODE_FREQUENCY = 0,
  INSTRUMENT_MODE_PERIOD,
  INSTRUMENT_MODE_DUTY,
  INSTRUMENT_MODE_INTERVAL
} InstrumentMode;

typedef enum
{
  FREQUENCY_METHOD_PERIOD = 0,
  FREQUENCY_METHOD_GATE
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

// 仪器功能模式
static volatile InstrumentMode instrument_mode = INSTRUMENT_MODE_FREQUENCY;
static volatile uint8_t instrument_mode_initialized = 0;

// TIM2：周期法 / 时间间隔共用的扩展时间轴
static volatile uint64_t period_ticks = 0;
static volatile uint64_t period_ns = 0;
static volatile uint64_t timestamp1 = 0;
static volatile uint64_t timestamp2 = 0;
static volatile uint8_t capture_state = 0;
static volatile uint32_t tim2_overflow_count = 0;

// A -> B 时间间隔
static volatile uint64_t interval_start_timestamp = 0;
static volatile uint64_t interval_end_timestamp = 0;
static volatile uint64_t interval_ticks = 0;
static volatile uint64_t interval_ns = 0;
static volatile uint8_t interval_waiting_ch2 = 0;
static volatile uint32_t interval_start_tick_ms = 0;
static volatile uint8_t interval_valid = 0;

// 周期法频率
static volatile uint32_t frequency_hz = 0;
static volatile uint64_t frequency_millihz = 0;
static volatile uint32_t last_capture_tick_ms = 0;
static volatile uint8_t frequency_valid = 0;
static volatile uint8_t period_requests_gate = 0;

// FREQUENCY / PERIOD 共用的自动测量策略
static volatile FrequencyMethod frequency_method = FREQUENCY_METHOD_PERIOD;
static volatile uint32_t measured_frequency_hz = 0;
static volatile uint64_t measured_period_ns = 0;

// DUTY：TIM2 PWM Input 硬件锁存结果
static volatile uint32_t duty_period_ticks = 0;
static volatile uint32_t duty_high_ticks = 0;
static volatile uint16_t measured_duty_permille = 0; // 0~1000，对应 0.0%~100.0%
static volatile uint16_t duty_prescaler = DUTY_DEFAULT_PRESCALER;
static volatile uint32_t last_duty_capture_tick_ms = 0;
static volatile uint8_t duty_capture_synced = 0;
static volatile uint8_t duty_valid = 0;

// TIM1：1 秒硬件闸门内统计外部脉冲
static volatile uint32_t gate_frequency_hz = 0;
static volatile uint32_t tim1_overflow_count = 0;
static volatile uint8_t gate_frequency_valid = 0;

// TIM4：1 秒 One Pulse 闸门
static volatile uint8_t gate_ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Instrument_SetMode(InstrumentMode new_mode);
static uint8_t Instrument_IsFrequencyPeriodMode(InstrumentMode mode);
static void Gate_Stop(void);
static void Gate_StartFresh(void);
static void TIM2_ConfigureTimestampCapture(uint8_t enable_ch2_interrupt);
static void TIM2_ConfigureDutyCapture(uint16_t prescaler);
static uint16_t Duty_CalculatePrescaler(uint32_t gate_frequency);
static void Duty_ApplyPrescaler(uint16_t prescaler);
static void Duty_ProcessCapture(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * HAL / 定时器 API 阅读提示：
 * 1. HAL_xxx(...)：HAL 函数，通常维护软件状态并返回 HAL_StatusTypeDef。
 * 2. __HAL_xxx(...)：更接近寄存器操作的 HAL 宏。
 * 3. 函数名带 _IT：Interrupt，表示配合中断使用。
 */

static uint8_t Instrument_IsFrequencyPeriodMode(InstrumentMode mode)
{
  return (mode == INSTRUMENT_MODE_FREQUENCY) ||
         (mode == INSTRUMENT_MODE_PERIOD);
}

/* 停止 TIM1 + TIM4 硬件闸门链。 */
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

/* 从一个干净状态重新开始 1 秒闸门。 */
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

/*
 * 恢复 CubeMX 的普通 TIM2 输入捕获结构：
 * CH1 = TI1 / PA0 上升沿；CH2 = TI2 / PA1 上升沿；PSC=0；无 Slave Mode。
 *
 * enable_ch2_interrupt=0：FREQUENCY / PERIOD
 * enable_ch2_interrupt=1：INTERVAL
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

/*
 * TIM2 PWM Input（同一根 PA0 / TI1）：
 *   CCR1：相邻两个上升沿之间的周期
 *   CCR2：上升沿到下降沿之间的高电平时间
 *   TI1FP1 上升沿同时把 CNT Reset 为 0
 *
 * 整个过程由硬件持续锁存，不打开 CC1/CC2 中断，因此 MHz 输入不会制造 MHz 级 ISR。
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

/*
 * 根据 1 秒闸门得到的粗频率，自动选择 TIM2 PSC。
 * 使用 gate_count-1 作为保守的频率下界，避免低频 ±1 count 误差导致周期溢出。
 */
static uint16_t Duty_CalculatePrescaler(uint32_t gate_frequency)
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

/* 改 PSC 后必须重新同步 PWM Input，不能继续使用旧 CCR。 */
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

/* 主循环轮询 PWM Input 的 CCR，不使用输入捕获中断。 */
static void Duty_ProcessCapture(void)
{
  uint32_t period_capture;
  uint32_t high_capture;

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

/*
 * 仪器功能层：
 * FREQUENCY / PERIOD -> 普通 TIM2 时间戳 + TIM1/TIM4 自动测频
 * DUTY              -> TIM2 PWM Input + TIM1/TIM4 粗频率/自动 PSC
 * INTERVAL          -> TIM2 CH1(A) + CH2(B)，停止闸门链
 */
static void Instrument_SetMode(InstrumentMode new_mode)
{
  uint32_t frequency_hint = measured_frequency_hz;

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

      uint32_t overflow_snapshot = tim1_overflow_count;
      uint32_t counter_snapshot = __HAL_TIM_GET_COUNTER(&htim1);

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
        uint16_t new_prescaler = Duty_CalculatePrescaler(gate_frequency_hz);

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

/*
 * FREQUENCY / PERIOD：CH1 执行普通输入捕获周期法。
 * INTERVAL：CH1=A，CH2=B。
 * DUTY：不进入本回调；CC1/CC2 中断被关闭，主循环直接读取 PWM Input 的 CCR。
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
        HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    uint32_t overflow_snapshot = tim2_overflow_count;

    if ((__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) &&
        (capture < 32768U))
    {
      overflow_snapshot++;
    }

    uint64_t current_timestamp =
        (uint64_t)overflow_snapshot * 65536ULL + capture;

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
        HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    uint32_t overflow_snapshot = tim2_overflow_count;

    if ((__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) &&
        (capture < 32768U))
    {
      overflow_snapshot++;
    }

    uint64_t current_timestamp =
        (uint64_t)overflow_snapshot * 65536ULL + capture;

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

/*
 * TIM1 -> 16 位外部脉冲计数器溢出
 * TIM2 -> 普通时间戳模式的 16 位 CNT 溢出
 * TIM4 -> 1 秒 One Pulse Gate 结束
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
