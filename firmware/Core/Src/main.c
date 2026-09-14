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
static volatile uint32_t frequency_hz = 0;             // 周期法计算得到的频率，单位 Hz
static volatile uint64_t frequency_millihz = 0;        // mHz
static volatile uint32_t last_capture_tick_ms = 0;     // 最近一次 TIM2_CH1 上升沿对应的 HAL tick
static volatile uint8_t frequency_valid = 0;           // 1=周期法当前频率结果仍然有效
static volatile uint8_t period_requests_gate = 0;      // 1=周期已经短到应切换到闸门法

// 自动测频策略
static volatile FrequencyMethod frequency_method = FREQUENCY_METHOD_PERIOD;
static volatile uint32_t measured_frequency_hz = 0;    // 自动选择后的最终频率结果

// TIM1：1 秒窗口内统计外部脉冲
static volatile uint32_t gate_frequency_hz = 0;      // TIM1 在 1 秒闸门内统计得到的频率，单位 Hz，等于“溢出次数 * 65536 + CNT”
static volatile uint32_t tim1_overflow_count = 0;    // TIM1 的 16 位 CNT 溢出次数，用于扩展外部脉冲计数范围
static volatile uint8_t gate_frequency_valid = 0;    // 1=至少已经完成过一次 1 秒闸门测量

// TIM4：1 秒 One Pulse 硬件闸门
static volatile uint8_t gate_ready = 0;               // TIM4 的 1 秒闸门完成标志：1=本轮测量结果可以读取

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
   * 参数：
   *   &htim2 -> TIM2 的 HAL 句柄地址，告诉 HAL 本次操作的是 TIM2。
   * 返回值：
   *   HAL_OK    -> 启动成功。
   *   非 HAL_OK -> 启动失败，因此进入 Error_Handler()。
   * 当前项目用途：让 TIM2 的 CNT 持续计数，并在 CNT 溢出时累计 tim2_overflow_count。
   */
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1)
   * 作用：启动 TIM2 通道 1 的 Input Capture（输入捕获）并开启捕获中断。
   * 参数：
   *   &htim2        -> TIM2 句柄地址。
   *   TIM_CHANNEL_1 -> PA0 / TIM2_CH1。
   * 当前项目用途：低频/中频时执行周期法测频。
   * 高频切换到闸门法后，只关闭 CC1 中断，不关闭 TIM2 本身。
   */
  if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * 启动 TIM2_CH2 输入捕获中断
   * PA1 上升沿到来时：
   *   TIM2 当前 CNT -> CCR2
   *   然后进入 HAL_TIM_IC_CaptureCallback()
   */
  if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1)
   * 作用：启动 TIM3 通道 1 的 PWM 输出。
   * 参数：
   *   &htim3        -> TIM3 的 HAL 句柄地址。
   *   TIM_CHANNEL_1 -> 使用 TIM3 通道 1，即当前的 PA6 / TIM3_CH1。
   * 当前项目用途：产生约 1 kHz、50% 占空比的测试方波。
   */
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* PWM 已经开始，再准备第一次硬件闸门测量。 */
  tim1_overflow_count = 0;

  /*
   * TIM1：外部脉冲计数器。
   * PA12 / TIM1_ETR 提供被测脉冲；
   * TIM4_TRGO 通过 ITR3 控制 TIM1 的 Gated Mode。
   */
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  HAL_NVIC_ClearPendingIRQ(TIM1_UP_IRQn);

  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * 此时 TIM1 的 CEN 已经打开，
   * 但 TIM4 尚未启动，TRGO 仍为 LOW，
   * 所以 TIM1 的 Gate 关闭，不会统计 PA12 输入脉冲。
   */

  /* TIM4：启动第一次 1 秒 One Pulse 硬件闸门。 */
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
  HAL_NVIC_ClearPendingIRQ(TIM4_IRQn);

  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /*
     * 周期法频率结果超时检测。
     *
     * 只在 PERIOD 模式下执行：GATE 模式会主动关闭 TIM2_CC1 中断，
     * 此时没有新的 CH1 捕获是正常现象，不能当成输入信号丢失。
     */
    if ((frequency_method == FREQUENCY_METHOD_PERIOD) &&
        frequency_valid &&
        ((uint32_t)(HAL_GetTick() - last_capture_tick_ms) > FREQUENCY_TIMEOUT_MS))
    {
      /*
       * TIM2 捕获中断可能与主循环超时判断同时发生。
       * 暂时关闭 TIM2 IRQ 后再复查一次，避免刚到的新边沿被误判为超时。
       */
      HAL_NVIC_DisableIRQ(TIM2_IRQn);

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

      HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }

    /*
     * PERIOD -> GATE：
     * 周期法已经判断 period_ticks <= 7200（约 >= 10 kHz）时，
     * 关闭 TIM2_CC1 捕获中断，避免高频输入产生海量 ISR。
     *
     * 注意这里只关闭 CC1 中断源，不关闭 TIM2：
     * TIM2 的基本计数、Update 中断以及 CH2 仍然保持工作。
     */
    if ((frequency_method == FREQUENCY_METHOD_PERIOD) && period_requests_gate)
    {
      measured_frequency_hz = frequency_hz;

      __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
      __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);

      frequency_method = FREQUENCY_METHOD_GATE;
      period_requests_gate = 0;
      frequency_valid = 0;
      capture_state = 0;
      interval_waiting_ch2 = 0;
    }

    if (gate_ready)
    {
      /* TIM4 已经关闭 Gate，此时 TIM1 不再接收外部脉冲 */

      HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);

      uint32_t overflow_snapshot = tim1_overflow_count;
      uint32_t counter_snapshot = __HAL_TIM_GET_COUNTER(&htim1);

      /*
       * 极端情况：
       * TIM1 已经溢出，但 Update ISR 还没来得及执行。
       */
      if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET)
      {
        overflow_snapshot++;
      }

      gate_frequency_hz =
          overflow_snapshot * 65536UL + counter_snapshot;
      gate_frequency_valid = 1;

      /*
       * 当前使用闸门法时，1 秒闸门结果就是最终测频结果。
       * 当结果下降到 7 kHz 或更低时，切回周期法。
       */
      if (frequency_method == FREQUENCY_METHOD_GATE)
      {
        measured_frequency_hz = gate_frequency_hz;

        if (gate_frequency_hz <= GATE_TO_PERIOD_HZ)
        {
          /*
           * 重新进入周期法前，清掉旧捕获状态。
           * 下一次必须重新收到两个 CH1 上升沿，才能形成新的周期结果。
           */
          frequency_method = FREQUENCY_METHOD_PERIOD;
          frequency_valid = 0;
          period_requests_gate = 0;
          capture_state = 0;
          period_ticks = 0;
          period_ns = 0;
          timestamp1 = 0;
          timestamp2 = 0;
          interval_waiting_ch2 = 0;

          /*
           * GATE 模式期间 CCR1 仍可能被硬件更新并留下 CC1IF。
           * 先清旧标志，再重新允许 CC1 捕获中断。
           */
          __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
          __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
        }
      }

      /* 准备下一轮 */
      tim1_overflow_count = 0;

      __HAL_TIM_SET_COUNTER(&htim1, 0);

      __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
      HAL_NVIC_ClearPendingIRQ(TIM1_UP_IRQn);

      gate_ready = 0;

      HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

      /*
       * TIM1 不需要重新启动：
       * 它的 CEN 仍然开着，只是由于 TIM4 TRGO=LOW，
       * Gated Mode 暂停了计数。
       *
       * 这里只需要重新启动 TIM4 One Pulse。
       */
      __HAL_TIM_SET_COUNTER(&htim4, 0);
      __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
      HAL_NVIC_ClearPendingIRQ(TIM4_IRQn);

      __HAL_TIM_ENABLE(&htim4);
    }

    /*
     * 最终输出选择：
     * - PERIOD 模式且周期结果有效：使用周期法；
     * - PERIOD 模式正在等待两个边沿时：若已有闸门结果，则暂时使用闸门结果兜底；
     * - GATE 模式：使用最近一次 1 秒闸门结果。
     */
    if (frequency_method == FREQUENCY_METHOD_PERIOD)
    {
      if (frequency_valid)
      {
        measured_frequency_hz = frequency_hz;
      }
      else if (gate_frequency_valid)
      {
        measured_frequency_hz = gate_frequency_hz;
      }
      else
      {
        measured_frequency_hz = 0;
      }
    }
    else if (gate_frequency_valid)
    {
      measured_frequency_hz = gate_frequency_hz;
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
 * 作用：HAL 的输入捕获回调函数。当已经启用输入捕获中断的通道发生捕获事件时，HAL 会调用它。
 * 参数：
 *   htim -> 触发本次回调的定时器句柄指针。
 * 当前项目用途：
 *   TIM2_CH1 -> 周期法测频，以及 A 信号时间戳；
 *   TIM2_CH2 -> B 信号时间戳。
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* 这个回调目前只处理 TIM2 */
  if (htim->Instance != TIM2)
  {
    return;
  }

  /* ==================== TIM2_CH1 / PA0 ==================== */
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    /*
     * CH1 输入捕获：
     * PA0 上升沿到来时，硬件已经自动完成：
     *
     * TIM2_CNT -> CCR1
     */
    uint32_t capture =
        HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    /* 拍下当前 TIM2 软件溢出次数 */
    uint32_t overflow_snapshot = tim2_overflow_count;

    /*
     * 处理捕获与溢出几乎同时发生的边界情况。
     *
     * UIF = 1：
     *   TIM2 已经溢出；
     *
     * capture 很小：
     *   说明本次捕获大概率发生在溢出之后。
     */
    if ((__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) &&
        (capture < 32768U))
    {
      overflow_snapshot++;
    }

    /*
     * 把：
     *
     * 溢出次数 + CCR1
     *
     * 拼成同一条连续的 64 位时间轴。
     */
    uint64_t current_timestamp =
        (uint64_t)overflow_snapshot * 65536ULL + capture;

    /* 记录最近一次 CH1 上升沿，用于主循环判断周期法结果是否超时。 */
    last_capture_tick_ms = HAL_GetTick();

    /* ---------- CH1 周期法测频 ---------- */
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

        /* 已经获得完整周期，本次周期法频率结果有效。 */
        frequency_valid = 1;

        /*
         * 周期短到 7200 tick 或更小时，请求主循环切换到闸门法。
         * 这里只置标志，不在 ISR 里直接修改中断配置。
         */
        if (period_ticks <= PERIOD_TO_GATE_TICKS)
        {
          period_requests_gate = 1;
        }
      }

      /* 当前边沿成为下一次测量的“上一次边沿” */
      timestamp1 = timestamp2;
    }

    /* ---------- A -> B 时间间隔测量起点 ---------- */
    if (interval_waiting_ch2 == 0)
    {
      /*
       * CH1 这次上升沿作为 A 信号。
       * 保存 A 到达时刻，然后等待 CH2 的 B 信号。
       */
      interval_start_timestamp = current_timestamp;
      interval_waiting_ch2 = 1;
    }
  }

  /* ==================== TIM2_CH2 / PA1 ==================== */
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    /*
     * CH2 输入捕获：
     * PA1 上升沿到来时，硬件已经自动完成：
     *
     * TIM2_CNT -> CCR2
     */
    uint32_t capture =
        HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    uint32_t overflow_snapshot = tim2_overflow_count;

    /* 和 CH1 一样处理溢出边界 */
    if ((__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) &&
        (capture < 32768U))
    {
      overflow_snapshot++;
    }

    uint64_t current_timestamp =
        (uint64_t)overflow_snapshot * 65536ULL + capture;

    /*
     * 只有已经收到 CH1 的 A 信号，
     * CH2 的 B 信号才有意义。
     */
    if (interval_waiting_ch2 == 1)
    {
      interval_end_timestamp = current_timestamp;

      /*
       * B - A = 两路信号之间经历的 TIM2 tick 数。
       *
       * TIM2 当前计数频率为 72 MHz：
       *
       * 1 tick ≈ 13.8889 ns
       *
       * 因此 interval_ticks 是原始硬件计数值，
       * interval_ns 才是换算后的实际时间。
       */
      interval_ticks =
          interval_end_timestamp - interval_start_timestamp;

      interval_ns =
          (interval_ticks * NANOSECONDS_PER_SECOND +
           TIM2_COUNTER_HZ / 2ULL) /
          TIM2_COUNTER_HZ;

      /*
       * 本次 A -> B 测量结束。
       * 下一次重新等待新的 CH1。
       */
      interval_waiting_ch2 = 0;
    }
  }
}

/*
 * HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 * 作用：HAL 的定时器周期/Update 回调。当启用了 Update 中断的定时器发生更新事件时调用。
 * 参数：
 *   htim -> 发生 Update Event 的定时器句柄指针。
 * 当前项目中三个来源：
 *   TIM1 -> 16 位外部脉冲计数器溢出。
 *   TIM2 -> 16 位时间计数器溢出。
 *   TIM4 -> 1 秒 One Pulse 硬件闸门结束。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    /* TIM1 负责统计外部脉冲；
     * 每溢出一次代表又累计了 65536 个脉冲。
     */
    tim1_overflow_count++;
  }
  else if (htim->Instance == TIM2)
  {
    tim2_overflow_count++;
  }
  else if (htim->Instance == TIM4)
  {
    /*
     * TIM4 的 1 秒 One Pulse 已结束。
     *
     * 注意：
     * 此时硬件已经自动：
     *
     * CEN -> 0
     * TRGO -> LOW
     *
     * TIM1 的 Gate 已经关闭，
     * 所以这里只负责通知主循环读取结果。
     */
    gate_ready = 1;
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
