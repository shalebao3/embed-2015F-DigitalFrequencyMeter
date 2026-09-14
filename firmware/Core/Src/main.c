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

/* TIM2 当前直接使用 72 MHz 定时器时钟，PSC = 0 */
#define TIM2_COUNTER_HZ 72000000ULL

/* 1 秒 = 1,000,000,000 ns */
#define NANOSECONDS_PER_SECOND 1000000000ULL

/*
 * 题目最低测量频率为 1 Hz，对应周期约 1000 ms。
 * 1500 ms 超时既给 1 Hz 留出余量，又能避免输入断开后长期保留旧结果。
 */
#define FREQUENCY_TIMEOUT_MS 1500U

/*
 * 题目 A -> B 时间间隔上限为 100 ms。
 * 这里给到 200 ms 超时，既覆盖题目范围，又能在 B 丢失时尽快重新等待新的 A。
 */
#define INTERVAL_TIMEOUT_MS 200U

/*
 * 周期法 -> 闸门法切换阈值。
 * TIM2 = 72 MHz，10 kHz 周期对应 7200 tick。
 * 当 period_ticks <= 7200 时，说明输入频率已经达到约 10 kHz 或更高。
 */
#define PERIOD_TO_GATE_TICKS 7200ULL

/*
 * 闸门法 -> 周期法切换阈值。
 * 使用 7 kHz，与 10 kHz 的切换点形成迟滞区，避免临界频率附近来回切换。
 */
#define GATE_TO_PERIOD_HZ 7000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// 仪器功能模式
static volatile InstrumentMode instrument_mode = INSTRUMENT_MODE_FREQUENCY;
static volatile uint8_t instrument_mode_initialized = 0;

// TIM2
static volatile uint64_t period_ticks = 0; // 相邻两次 CH1 捕获之间的 TIM2 tick 数
static volatile uint64_t period_ns = 0;    // 输入信号周期，单位 ns

static volatile uint64_t timestamp1 = 0; // 上一次 CH1 捕获的扩展时间戳
static volatile uint64_t timestamp2 = 0; // 当前 CH1 捕获的扩展时间戳

static volatile uint8_t capture_state = 0;        // 0=等待第一次捕获，1=已有上一时间戳
static volatile uint32_t tim2_overflow_count = 0; // TIM2 16 位 CNT 软件溢出计数

static volatile uint64_t interval_start_timestamp = 0; // CH1：A 信号到达时间
static volatile uint64_t interval_end_timestamp = 0;   // CH2：B 信号到达时间
static volatile uint64_t interval_ticks = 0;           // A → B 的 TIM2 tick 数
static volatile uint64_t interval_ns = 0;              // A → B 时间间隔，单位 ns
static volatile uint8_t interval_waiting_ch2 = 0;      // 1=已经收到 A，等待 B
static volatile uint32_t interval_start_tick_ms = 0;   // A 到达时的 HAL tick，用于等待 B 超时判断
static volatile uint8_t interval_valid = 0;            // 1=当前 interval_ticks/interval_ns 是完整 A->B 测量结果

static volatile uint32_t frequency_hz = 0;             // 周期法计算得到的频率，单位 Hz
static volatile uint64_t frequency_millihz = 0;        // mHz
static volatile uint32_t last_capture_tick_ms = 0;     // 最近一次 TIM2_CH1 上升沿对应的 HAL tick
static volatile uint8_t frequency_valid = 0;           // 1=周期法当前频率结果仍然有效
static volatile uint8_t period_requests_gate = 0;      // 1=周期已经短到应切换到闸门法

// 自动频率测量策略；FREQUENCY/PERIOD 两种仪器模式共用
static volatile FrequencyMethod frequency_method = FREQUENCY_METHOD_PERIOD;
static volatile uint32_t measured_frequency_hz = 0;    // 自动选择后的最终频率结果
static volatile uint64_t measured_period_ns = 0;       // 自动选择后的最终周期结果

// TIM1：1 秒窗口内统计外部脉冲
static volatile uint32_t gate_frequency_hz = 0;      // TIM1 在 1 秒闸门内统计得到的频率，单位 Hz
static volatile uint32_t tim1_overflow_count = 0;    // TIM1 的 16 位 CNT 溢出次数
static volatile uint8_t gate_frequency_valid = 0;    // 1=至少已经完成过一次 1 秒闸门测量

// TIM4：1 秒 One Pulse 硬件闸门
static volatile uint8_t gate_ready = 0;              // 1=本轮闸门完成，主循环可以读取 TIM1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Instrument_SetMode(InstrumentMode new_mode);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * HAL / 定时器 API 阅读提示：
 *
 * 1. HAL_xxx(...)：HAL 库函数，通常会检查参数、维护 HAL 软件状态，并返回 HAL_StatusTypeDef。
 * 2. __HAL_xxx(...)：HAL 宏，通常更直接地读写外设寄存器，本身一般没有 HAL 状态检查。
 * 3. 函数名带 _IT：Interrupt，表示该功能会配合中断使用。
 * 4. htim：Timer Handle（定时器句柄），HAL 通过它知道当前操作的是哪一个 TIM 外设。
 */

/*
 * Instrument_SetMode()
 *
 * 仪器功能层和底层测量方法分开：
 *
 * FREQUENCY / PERIOD：
 *   共用自动测量引擎。
 *   低频使用 TIM2_CH1 周期法，高频使用 TIM1 + TIM4 闸门法。
 *   两种模式的区别主要是最终展示“频率”还是“周期”。
 *
 * INTERVAL：
 *   停止闸门测频，只启用 TIM2_CH1 + TIM2_CH2，专门测 A -> B 时间间隔。
 *
 * 后续增加按键或屏幕菜单时，只需要在主循环上下文调用本函数切换模式。
 */
static void Instrument_SetMode(InstrumentMode new_mode)
{
  if (instrument_mode_initialized && (new_mode == instrument_mode))
  {
    return;
  }

  /* FREQUENCY <-> PERIOD 共用同一个测量引擎，不需要重新启动硬件。 */
  if (instrument_mode_initialized &&
      (instrument_mode != INSTRUMENT_MODE_INTERVAL) &&
      (new_mode != INSTRUMENT_MODE_INTERVAL))
  {
    instrument_mode = new_mode;
    return;
  }

  /* 配置捕获中断前先暂时屏蔽整个 TIM2 IRQ，避免切换过程中进入回调。 */
  HAL_NVIC_DisableIRQ(TIM2_IRQn);

  __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
  __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC2);

  capture_state = 0;
  timestamp1 = 0;
  timestamp2 = 0;
  frequency_valid = 0;
  period_requests_gate = 0;

  interval_waiting_ch2 = 0;
  interval_start_timestamp = 0;
  interval_end_timestamp = 0;
  interval_start_tick_ms = 0;
  interval_ticks = 0;
  interval_ns = 0;
  interval_valid = 0;

  if (new_mode == INSTRUMENT_MODE_INTERVAL)
  {
    /*
     * 时间间隔模式不需要频率闸门。
     * 关闭 TIM4 后 TRGO 变 LOW；同时关闭 TIM1，避免无意义的外部计数。
     */
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

    /* A、B 两路输入捕获都需要中断。 */
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC2);
  }
  else
  {
    /*
     * FREQUENCY / PERIOD 模式：
     * CH1 用于低频周期法；CH2 不参与，避免无意义的 CH2 中断。
     */
    frequency_method = FREQUENCY_METHOD_PERIOD;
    measured_frequency_hz = 0;
    measured_period_ns = 0;
    frequency_hz = 0;
    frequency_millihz = 0;
    period_ticks = 0;
    period_ns = 0;
    gate_frequency_hz = 0;
    gate_frequency_valid = 0;

    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);

    /* 重启 TIM1 + TIM4 硬件闸门测量链。 */
    tim1_overflow_count = 0;
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    HAL_NVIC_ClearPendingIRQ(TIM1_UP_IRQn);

    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
    HAL_NVIC_ClearPendingIRQ(TIM4_IRQn);
    gate_ready = 0;

    /* 先准备脉冲计数器，再开启 One Pulse 闸门。 */
    __HAL_TIM_ENABLE(&htim1);
    __HAL_TIM_ENABLE(&htim4);
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

  /*
   * HAL_TIM_Base_Start_IT(&htim2)
   * 作用：启动 TIM2 的基本计数功能，并使能 TIM2 的 Update（更新/溢出）中断。
   */
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * 先把 CH1、CH2 两个输入捕获通道都启动。
   * Instrument_SetMode() 会根据仪器功能模式决定哪些捕获中断真正保持开启。
   */
  if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /* TIM3_CH1：产生约 1 kHz、50% 占空比的测试方波。 */
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * TIM1 / TIM4 的 Update 中断只在这里通过 HAL 启动一次。
   * 后续模式切换只操作 CEN 和具体中断源，不反复调用 Start_IT，避免 HAL 状态机冲突。
   */
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

  /* 默认进入频率测量模式；后续菜单/按键也统一调用 Instrument_SetMode()。 */
  Instrument_SetMode(INSTRUMENT_MODE_FREQUENCY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /*
     * INTERVAL 模式等待 B 超时处理。
     * A 已经到来但超过 200 ms 仍没有 B，则放弃本次 A，清除旧结果并重新等待新的 A。
     */
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

    /*
     * FREQUENCY / PERIOD 模式都需要周期结果有效性检测。
     * GATE 方法关闭 CH1 捕获是正常行为，因此只有 PERIOD 方法才做超时判断。
     */
    if ((instrument_mode != INSTRUMENT_MODE_INTERVAL) &&
        (frequency_method == FREQUENCY_METHOD_PERIOD) &&
        frequency_valid &&
        ((uint32_t)(HAL_GetTick() - last_capture_tick_ms) > FREQUENCY_TIMEOUT_MS))
    {
      HAL_NVIC_DisableIRQ(TIM2_IRQn);

      if ((instrument_mode != INSTRUMENT_MODE_INTERVAL) &&
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

    /*
     * FREQUENCY / PERIOD 模式内部：PERIOD -> GATE。
     * CC1 中断已经在捕获 ISR 内立即关闭，主循环只负责切换软件状态。
     */
    if ((instrument_mode != INSTRUMENT_MODE_INTERVAL) &&
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

    if ((instrument_mode != INSTRUMENT_MODE_INTERVAL) && gate_ready)
    {
      /* TIM4 已经关闭 Gate，此时 TIM1 不再接收外部脉冲。 */
      HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);

      uint32_t overflow_snapshot = tim1_overflow_count;
      uint32_t counter_snapshot = __HAL_TIM_GET_COUNTER(&htim1);

      /* TIM1 已溢出但 ISR 尚未来得及执行时，补记这一次溢出。 */
      if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET)
      {
        overflow_snapshot++;
      }

      gate_frequency_hz =
          overflow_snapshot * 65536UL + counter_snapshot;
      gate_frequency_valid = 1;

      if (frequency_method == FREQUENCY_METHOD_GATE)
      {
        measured_frequency_hz = gate_frequency_hz;

        /* 下降到 7 kHz 或更低时切回周期法。 */
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

      /* 准备下一轮 1 秒硬件闸门。 */
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

    /*
     * FREQUENCY / PERIOD 共用同一个最终测量结果。
     * PERIOD 模式只是把最终结果转换成周期显示，不重新建立另一套高频测量链。
     */
    if (instrument_mode != INSTRUMENT_MODE_INTERVAL)
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

      /* 高频使用闸门法时，根据最终频率反算周期。 */
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
 * HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
 *
 * FREQUENCY / PERIOD：CH1 负责自动测量引擎中的周期法入口。
 * INTERVAL：CH1 作为 A，CH2 作为 B，只计算 A -> B 时间间隔。
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM2)
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

    /* INTERVAL 模式：CH1 只负责记录 A 到达时刻。 */
    if (instrument_mode == INSTRUMENT_MODE_INTERVAL)
    {
      if (interval_waiting_ch2 == 0)
      {
        /* 新的一次 A 到来后，旧的 A->B 结果立即失效。 */
        interval_valid = 0;
        interval_ticks = 0;
        interval_ns = 0;
        interval_end_timestamp = 0;

        interval_start_timestamp = current_timestamp;
        interval_start_tick_ms = HAL_GetTick();
        interval_waiting_ch2 = 1;
      }
      return;
    }

    /* FREQUENCY / PERIOD：CH1 执行周期法。 */
    last_capture_tick_ms = HAL_GetTick();

    if (capture_state == 0)
    {
      timestamp1 = current_timestamp;
      capture_state = 1;
    }
    else
    {
      timestamp2 = current_timestamp;
      period_ticks = timestamp2 - timestamp1;

      if (period_ticks != 0)
      {
        frequency_hz =
            (TIM2_COUNTER_HZ + period_ticks / 2ULL) /
            period_ticks;

        frequency_millihz =
            (TIM2_COUNTER_HZ * 1000ULL +
             period_ticks / 2ULL) /
            period_ticks;

        period_ns =
            (period_ticks * NANOSECONDS_PER_SECOND +
             TIM2_COUNTER_HZ / 2ULL) /
            TIM2_COUNTER_HZ;

        frequency_valid = 1;

        /*
         * 约 >= 10 kHz 时立即关闭 CC1 中断，避免 MHz 输入淹没 CPU。
         * 主循环随后完成 PERIOD -> GATE 软件状态切换。
         */
        if (period_ticks <= PERIOD_TO_GATE_TICKS)
        {
          period_requests_gate = 1;
          __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
        }
      }

      timestamp1 = timestamp2;
    }
  }

  /* ==================== TIM2_CH2 / PA1 ==================== */
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    /* CH2 只属于 A -> B 时间间隔模式。 */
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

    if (interval_waiting_ch2 == 1)
    {
      /*
       * 即使主循环尚未来得及执行超时处理，也要在 B 到来时复查等待时间。
       * 超过 200 ms 的 B 不能和旧 A 配对。
       */
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

      interval_valid = 1;
      interval_waiting_ch2 = 0;
    }
  }
}

/*
 * HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 * 当前项目中三个来源：
 *   TIM1 -> 16 位外部脉冲计数器溢出。
 *   TIM2 -> 16 位时间计数器溢出。
 *   TIM4 -> 1 秒 One Pulse 硬件闸门结束。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    tim1_overflow_count++;
  }
  else if (htim->Instance == TIM2)
  {
    tim2_overflow_count++;
  }
  else if (htim->Instance == TIM4)
  {
    /* INTERVAL 模式下闸门链已经关闭，不产生测频完成事件。 */
    if (instrument_mode != INSTRUMENT_MODE_INTERVAL)
    {
      gate_ready = 1;
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