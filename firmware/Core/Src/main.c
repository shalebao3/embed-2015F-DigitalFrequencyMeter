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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* TIM2 当前直接使用 72 MHz 定时器时钟，PSC = 0 */
#define TIM2_COUNTER_HZ 72000000ULL

/* 1 秒 = 1,000,000,000 ns */
#define NANOSECONDS_PER_SECOND 1000000000ULL

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

static volatile uint32_t frequency_hz = 0; // 周期法计算得到的频率，单位 Hz

static volatile uint8_t capture_state = 0;        // 0=等待第一次捕获，1=已有上一时间戳
static volatile uint32_t tim2_overflow_count = 0; // TIM2 16 位 CNT 软件溢出计数

static volatile uint64_t interval_start_timestamp = 0; // CH1：A 信号到达时间
static volatile uint64_t interval_end_timestamp = 0;   // CH2：B 信号到达时间
static volatile uint64_t interval_ticks = 0;           // A → B 的 TIM2 tick 数
static volatile uint64_t interval_ns = 0;              // A → B 时间间隔，单位 ns
static volatile uint8_t interval_waiting_ch2 = 0;      // 1=已经收到 A，等待 B

// TIM4
static volatile uint32_t gate_frequency_hz = 0;      // TIM4 在 1 秒闸门内统计得到的频率，单位 Hz，等于“溢出次数 * 65536 + CNT”
static volatile uint32_t tim4_overflow_count = 0;    // TIM4 的 16 位 CNT 溢出次数，用于扩展外部脉冲计数范围

// TIM1
static volatile uint8_t gate_ready = 0;              // TIM1 的 1 秒闸门完成标志：1=本轮测量结果可以读取

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
   * HAL_TIM_Base_Start_IT(&htim4)
   * 作用：启动 TIM4 基本计数，并使能 TIM4 Update 中断。
   * 参数：
   *   &htim4 -> TIM4 的 HAL 句柄地址。
   * 当前项目用途：TIM4 工作在 External Clock Mode 1，PB6 每来一个有效脉冲就让 CNT +1；
   *             CNT 溢出时通过中断累计 tim4_overflow_count。
   */
  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1)
   * 作用：启动 TIM2 通道 1 的 Input Capture（输入捕获）并开启捕获中断。
   * 参数：
   *   &htim2        -> TIM2 的 HAL 句柄地址。
   *   TIM_CHANNEL_1 -> 使用 TIM2 的通道 1，也就是当前配置的 PA0 / TIM2_CH1。
   * 当前项目用途：PA0 出现上升沿时，硬件把当前 CNT 锁存进 CCR1，并触发输入捕获回调。
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

  /* PWM 已经开始，再开启第一次测量窗口 */
  tim4_overflow_count = 0;

  /*
   * __HAL_TIM_SET_COUNTER(&htim4, 0)
   * 作用：直接把 TIM4 的 CNT 寄存器写成 0。
   * 参数：
   *   &htim4 -> TIM4 句柄地址。
   *   0      -> 要写入 CNT 的值。
   * 当前项目用途：让第一轮闸门计数从 0 个脉冲开始。
   */
  __HAL_TIM_SET_COUNTER(&htim4, 0);

  /*
   * __HAL_TIM_SET_COUNTER(&htim1, 0)
   * 作用：把 TIM1 的 CNT 清零。
   * 参数：
   *   &htim1 -> TIM1 句柄地址。
   *   0      -> CNT 初始值。
   * 当前项目用途：让 1 秒闸门从 t=0 开始计时。
   */
  __HAL_TIM_SET_COUNTER(&htim1, 0);

  /*
   * HAL_TIM_Base_Start_IT(&htim1)
   * 作用：启动 TIM1 基本计数，并开启 TIM1 Update 中断。
   * 参数：
   *   &htim1 -> TIM1 的 HAL 句柄地址。
   * 当前项目用途：TIM1 每计满 1 秒产生一次 Update Event，作为门控测频的时间基准。
   */
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  uint64_t frequency_millihz;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (gate_ready)
    {
      /*
       * HAL_NVIC_DisableIRQ(TIM4_IRQn)
       * 作用：暂时禁止 NVIC 响应 TIM4 的中断请求。
       * 参数：
       *   TIM4_IRQn -> TIM4 在 NVIC 中对应的中断号。
       * 当前项目用途：读取 TIM4 软件溢出次数和 CNT 时，避免 TIM4 ISR 同时修改数据。
       */
      HAL_NVIC_DisableIRQ(TIM4_IRQn);

      uint32_t overflow_snapshot = tim4_overflow_count;       // 拍下 TIM4 当前已经记录的软件溢出次数
      uint32_t counter_snapshot =                             // 拍下 1 秒闸门结束时 TIM4 当前 CNT 的剩余脉冲数
          /*
           * __HAL_TIM_GET_COUNTER(&htim4)
           * 作用：读取 TIM4 当前 CNT 寄存器值。
           * 参数：
           *   &htim4 -> TIM4 句柄地址。
           * 返回值：当前 TIM4_CNT 的值。
           */
          __HAL_TIM_GET_COUNTER(&htim4);

      /*
       * __HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE)
       * 作用：检查 TIM4 的 Update Flag（UIF）是否已经置位。
       * 参数：
       *   &htim4          -> TIM4 句柄地址。
       *   TIM_FLAG_UPDATE -> 要检查的标志位，这里就是更新/溢出标志 UIF。
       * 返回值：RESET 表示未置位；非 RESET 表示该标志已经置位。
       * 当前项目用途：如果已经发生溢出，但 ISR 还没来得及执行，就手动把这次溢出补进快照。
       */
      if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET)
      {
        overflow_snapshot++;
      }

      gate_frequency_hz =
          overflow_snapshot * 65536UL + counter_snapshot;

      /* 清理下一轮 */
      tim4_overflow_count = 0;

      /*
       * __HAL_TIM_SET_COUNTER(..., 0)
       * 作用：分别把 TIM4、TIM1 的 CNT 清零，为下一轮 1 秒测量重新从 0 开始。
       */
      __HAL_TIM_SET_COUNTER(&htim4, 0);
      __HAL_TIM_SET_COUNTER(&htim1, 0);

      /*
       * __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE)
       * 作用：清除 TIM4 的 Update Flag（UIF）。
       * 参数：
       *   &htim4          -> TIM4 句柄地址。
       *   TIM_FLAG_UPDATE -> 要清除的更新/溢出标志。
       * 当前项目用途：避免上一轮遗留的 UIF 干扰下一轮测量。
       */
      __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

      /*
       * HAL_NVIC_ClearPendingIRQ(TIM4_IRQn)
       * 作用：清除 NVIC 中已经挂起（Pending）的 TIM4 中断请求。
       * 参数：
       *   TIM4_IRQn -> TIM4 的 NVIC 中断号。
       * 注意：它清的是 NVIC 的 Pending 状态，不是 TIM4 外设里的 UIF。
       */
      HAL_NVIC_ClearPendingIRQ(TIM4_IRQn);

      gate_ready = 0;

      /*
       * HAL_NVIC_EnableIRQ(TIM4_IRQn)
       * 作用：重新允许 NVIC 响应 TIM4 中断。
       * 参数：
       *   TIM4_IRQn -> TIM4 的 NVIC 中断号。
       */
      HAL_NVIC_EnableIRQ(TIM4_IRQn);

      /*
       * __HAL_TIM_ENABLE(&htimX)
       * 作用：直接设置对应定时器 CR1 寄存器中的 CEN 位，使计数器继续运行。
       * 参数：
       *   &htim4 -> 恢复 TIM4 外部脉冲计数。
       *   &htim1 -> 恢复 TIM1 的 1 秒闸门计时。
       * 注意：第一次已经通过 HAL_TIM_Base_Start_IT() 开启过中断，
       *      后续这里只恢复硬件计数，不需要再次调用 Start_IT。
       */
      __HAL_TIM_ENABLE(&htim4);
      __HAL_TIM_ENABLE(&htim1);
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
 * 当前项目用途：判断是不是 TIM2_CH1 的捕获事件，然后读取 CCR1 并计算信号周期/频率。
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

    /* ---------- 原来的 CH1 周期法测频 ---------- */
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

        period_ns =
            (period_ticks * NANOSECONDS_PER_SECOND +
             TIM2_COUNTER_HZ / 2ULL) /
            TIM2_COUNTER_HZ;
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
 *   TIM1 -> 1 秒闸门结束。
 *   TIM2 -> 16 位时间计数器溢出。
 *   TIM4 -> 16 位外部脉冲计数器溢出。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    /* 1 秒测量窗口结束 */

    /*
     * __HAL_TIM_DISABLE(&htimX)
     * 作用：清除定时器 CR1 寄存器中的 CEN 位，停止硬件计数。
     * 参数：
     *   &htim1 -> 停止 1 秒闸门计时。
     *   &htim4 -> 停止外部脉冲计数，使这一轮结果冻结。
     */
    __HAL_TIM_DISABLE(&htim1);
    __HAL_TIM_DISABLE(&htim4);

    gate_ready = 1;
  }
  else if (htim->Instance == TIM2)
  {
    tim2_overflow_count++;
  }
  else if (htim->Instance == TIM4)
  {
    tim4_overflow_count++;
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
  * @param  line: assert_param error line source number
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
