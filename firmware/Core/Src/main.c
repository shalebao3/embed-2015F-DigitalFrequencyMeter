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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// TIM2
static volatile uint64_t period_ticks = 0;          // TIM2 相邻两次输入捕获之间的周期计数值，当前 1 tick = 1 us
static volatile uint64_t timestamp1 = 0;            // TIM2 上一次输入边沿的扩展时间戳
static volatile uint64_t timestamp2 = 0;            // TIM2 当前输入边沿的扩展时间戳
static volatile uint32_t frequency_hz = 0;           // TIM2 周期法计算得到的频率，单位 Hz
static volatile uint8_t capture_state = 0;           // TIM2 输入捕获状态：0=等待第一次捕获，1=已经有上一时间戳
static volatile uint32_t tim2_overflow_count = 0;    // TIM2 的 16 位 CNT 溢出次数，用于扩展时间戳范围
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
  if (htim->Instance == TIM2 &&
      htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uint32_t capture =                                  // 本次 PA0 上升沿到来时，TIM2 硬件锁存到 CCR1 的 CNT 值
        /*
         * HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1)
         * 作用：读取输入捕获寄存器 CCR1 中保存的捕获值。
         * 参数：
         *   htim          -> 当前触发回调的定时器句柄，这里实际是 TIM2。
         *   TIM_CHANNEL_1 -> 读取通道 1 对应的 CCR1。
         * 返回值：本次输入边沿到来时硬件锁存的 CNT 值。
         */
        HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    /* 拍一张当前溢出次数的快照 */
    uint32_t overflow_snapshot = tim2_overflow_count;  // 拍下本次捕获对应的 TIM2 软件溢出次数

    /*
     * 如果 CNT 已经发生溢出，但 Update Callback
     * 还没来得及把 tim2_overflow_count +1，
     * 并且本次捕获值很小，说明捕获发生在溢出之后。
     *
     * __HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE)
     * 参数：
     *   htim             -> 当前定时器句柄（TIM2）。
     *   TIM_FLAG_UPDATE  -> 检查 Update Flag / UIF。
     */
    if ((__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) &&
        (capture < 32768U))
    {
      overflow_snapshot++;
    }

    uint64_t current_timestamp =                        // 将“溢出次数 + 本次 CCR1”组合成 64 位扩展时间戳
        (uint64_t)overflow_snapshot * 65536ULL + capture;

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
        frequency_hz = 1000000ULL / period_ticks;
      }

      timestamp1 = timestamp2;
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
