#include "measurement_hw.h"

#include "main.h"
#include "tim.h"

/* TIM2 普通时间戳模式的 16 位 CNT 软件溢出扩展。 */
static volatile uint32_t tim2_overflow_count = 0;

/* TIM1 1 秒 Gate 内的 16 位外部脉冲计数软件溢出扩展。 */
static volatile uint32_t tim1_overflow_count = 0;

/* TIM4 One Pulse 完成本轮 1 秒 Gate 后置 1。 */
static volatile uint8_t gate_ready = 0;

/**
 * @brief 启动测量系统依赖的 TIM1/TIM2/TIM3/TIM4。
 * @note CubeMX 的 MX_TIMx_Init() 必须已经执行。
 * @note HAL Start 只做一次；后续模式切换直接调整中断源、CEN 和相关寄存器。
 */
void MeasurementHw_Init(void)
{
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

  /* TIM3_CH1：项目内部约 1 kHz、50% 占空比测试 PWM。 */
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

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
}

/**
 * @brief 停止 TIM1 + TIM4 硬件 Gate 测频链，并清理本轮运行状态。
 * @note 会清除 Gate 完成标志、TIM1 软件溢出计数，并把 TIM1/TIM4 CNT 归零。
 */
void MeasurementHw_GateStop(void)
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
 * @brief 从干净状态启动一轮新的 1 秒 Gate。
 * @note 先调用 MeasurementHw_GateStop() 清理旧状态，再先准备 TIM1、后启动 TIM4 One Pulse，避免窗口起点丢脉冲。
 */
void MeasurementHw_GateStartFresh(void)
{
  MeasurementHw_GateStop();

  __HAL_TIM_ENABLE(&htim1);
  __HAL_TIM_ENABLE(&htim4);
}

/**
 * @brief 在本轮 Gate 已结束时快照 TIM1 脉冲计数、生成频率结果，并立即准备下一轮 Gate。
 * @param frequency_hz 输出参数，用于返回本轮 1 秒 Gate 内统计到的脉冲总数，单位 Hz。
 * @retval 1=成功取得一轮新的 Gate 结果；0=输出指针为空或 Gate 尚未结束。
 */
uint8_t MeasurementHw_GateTakeFrequency(uint32_t *frequency_hz)
{
  uint32_t overflow_snapshot;
  uint32_t counter_snapshot;

  if ((frequency_hz == 0) || (gate_ready == 0U))
  {
    return 0U;
  }

  /* Gate 已经由 TIM4 TRGO 拉低，此时 TIM1 不再接收外部脉冲。 */
  HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);

  overflow_snapshot = tim1_overflow_count;
  counter_snapshot = __HAL_TIM_GET_COUNTER(&htim1);

  /* 硬件已经溢出、但 Update ISR 尚未来得及执行时补记一次。 */
  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET)
  {
    overflow_snapshot++;
  }

  *frequency_hz = overflow_snapshot * 65536UL + counter_snapshot;

  /* 重置 TIM1，为下一轮 1 秒 Gate 做准备。 */
  tim1_overflow_count = 0;
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  HAL_NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
  gate_ready = 0;
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

  /* TIM4 是 One Pulse；重新置 CNT 并使能即可开始下一轮。 */
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
  HAL_NVIC_ClearPendingIRQ(TIM4_IRQn);
  __HAL_TIM_ENABLE(&htim4);

  return 1U;
}

/**
 * @brief 把 TIM2 配置为普通自由运行时间轴 + 双路输入捕获。
 * @param enable_ch2_interrupt 0=只使能 CH1 捕获中断；非 0=额外使能 CH2 捕获中断。
 * @note CH1=Direct TI1，CH2=Direct TI2；该参数只控制 CH2 中断，不关闭 CH2 捕获硬件。
 */
void MeasurementHw_TIM2ConfigureTimestampCapture(uint8_t enable_ch2_interrupt)
{
  __HAL_TIM_DISABLE(&htim2);

  // DMA / 中断使能寄存器(TIMx_DIER)，UIE：允许更新中断（Update interrupt enable）0：禁止更新中断；1：允许更新中断。
  htim2.Instance->DIER = 0U;
  // 状态寄存器(TIMx_SR)，UIF：更新中断标志（Update interrupt flag）0：没有发生更新事件；1：发生了更新事件。
  htim2.Instance->SR = 0U;

  /* 退出 DUTY 使用的 Reset Mode。把 TIM2 的 SMCR 寄存器中的 SMS 和 TS 字段清零，其他位保持不变， */
  htim2.Instance->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_TS);

  /* CH1=Direct TI1；CH2=Direct TI2；DIV1；Filter=0。 */
  // CCMR1：配置为输入捕获，并选择输入信号
  htim2.Instance->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  // CCER：使能捕获，并配置捕获边沿
  htim2.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;

  htim2.Init.Prescaler = 0U;
  htim2.Instance->PSC = 0U;
  htim2.Instance->ARR = 65535U;
  htim2.Instance->CNT = 0U;
  htim2.Instance->EGR = TIM_EGR_UG;
  htim2.Instance->SR = 0U;

  tim2_overflow_count = 0;

  htim2.Instance->DIER = TIM_IT_UPDATE | TIM_IT_CC1;
  if (enable_ch2_interrupt)
  {
    htim2.Instance->DIER |= TIM_IT_CC2;  // 置一，使能 CH2 捕获中断
  }

  HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
  __HAL_TIM_ENABLE(&htim2);
}

/**
 * @brief 把 TIM2 配置为 PWM Input，供 DUTY 模式测量周期和高电平时间。
 * @param prescaler 写入 TIM2 PSC 的预分频值。
 * @note CH1 Direct TI1 捕获上升沿得到周期；CH2 Indirect TI1 捕获下降沿得到高电平时间。
 * @note TI1FP1 上升沿作为 Reset Mode 触发源，每个新周期自动把 CNT 归零。
 */
void MeasurementHw_TIM2ConfigureDutyCapture(uint16_t prescaler)
{
  __HAL_TIM_DISABLE(&htim2);

  htim2.Instance->DIER = 0U;
  htim2.Instance->SR = 0U;

  /* 同一根 TI1：CH1 Direct 捕获上升沿，CH2 Indirect 捕获下降沿。 */
  htim2.Instance->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1;  
  htim2.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;

  /* TI1FP1 上升沿触发 Reset Mode，每周期自动把 CNT 归零。 */
  htim2.Instance->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_TS);
  htim2.Instance->SMCR |= TIM_SLAVEMODE_RESET | TIM_TS_TI1FP1;

  htim2.Init.Prescaler = prescaler;
  htim2.Instance->PSC = prescaler;
  htim2.Instance->ARR = 65535U;
  htim2.Instance->CNT = 0U;
  htim2.Instance->EGR = TIM_EGR_UG;
  htim2.Instance->SR = 0U;

  tim2_overflow_count = 0;
  HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
  __HAL_TIM_ENABLE(&htim2);
}

/**
 * @brief DUTY 自动量程时应用新的 TIM2 PSC，并重新同步计数器。
 * @param prescaler 新的 TIM2 预分频寄存器 PSC 值。
 * @note 会把 CNT 清零并产生一次更新事件，使新的 PSC 生效。
 */
void MeasurementHw_TIM2ApplyPrescaler(uint16_t prescaler)
{
  __HAL_TIM_DISABLE(&htim2);

  htim2.Init.Prescaler = prescaler;
  htim2.Instance->PSC = prescaler;
  htim2.Instance->CNT = 0U;
  htim2.Instance->EGR = TIM_EGR_UG;
  htim2.Instance->SR = 0U;

  __HAL_TIM_ENABLE(&htim2);
}

/**
 * @brief 读取 TIM2 某捕获通道的 CCR，并结合软件溢出计数构造 64 位扩展时间戳。
 * @param channel TIM_CHANNEL_1 或 TIM_CHANNEL_2。
 * @retval 64 位扩展时间戳，单位为 TIM2 tick。
 * @note UIF 已置位但 Update ISR 尚未执行时，会根据捕获值判断本次 Capture 是否位于溢出之后。
 */
uint64_t MeasurementHw_TIM2ReadCapturedTimestamp(uint32_t channel)
{
  uint32_t capture = HAL_TIM_ReadCapturedValue(&htim2, channel);
  uint32_t overflow_snapshot = tim2_overflow_count;

  /* UIF 已置位但 Update ISR 尚未执行时，判断本次 Capture 是否发生在溢出之后。 */
  if ((__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) &&
      (capture < 32768U))
  {
    overflow_snapshot++;
  }

  return (uint64_t)overflow_snapshot * 65536ULL + capture;
}

/**
 * @brief 轮询 PWM Input 的 CCR1/CCR2，并在成功读取后清除捕获与 overcapture 标志。
 * @param period_ticks 输出完整周期 tick 数，对应 CCR1。
 * @param high_ticks 输出高电平持续 tick 数，对应 CCR2。
 * @retval 1=同时获得新的周期和高电平捕获；0=输出指针为空或数据尚不完整。
 */
uint8_t MeasurementHw_TIM2ReadPwmCapture(uint32_t *period_ticks,
                                         uint32_t *high_ticks)
{
  if ((period_ticks == 0) || (high_ticks == 0))
  {
    return 0U;
  }

  if ((__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1) == RESET) ||
      (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC2) == RESET))
  {
    return 0U;
  }

  *period_ticks = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
  *high_ticks = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);

  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1 | TIM_FLAG_CC2 |
                               TIM_FLAG_CC1OF | TIM_FLAG_CC2OF);
  return 1U;
}

/**
 * @brief 打开或关闭 TIM2 CH1 捕获中断。
 * @param enable 0=关闭 CC1 中断；非 0=打开 CC1 中断。
 * @note 切换前会先关闭 CC1 中断并清除旧 CC1 标志，避免处理陈旧捕获事件。
 */
void MeasurementHw_TIM2SetCh1Interrupt(uint8_t enable)
{
  __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);

  if (enable)
  {
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
  }
}

/**
 * @brief 在 NVIC 层暂时屏蔽 TIM2_IRQn。
 * @note 用于保护与 TIM2 ISR 共享的软件状态，避免主循环与中断竞争。
 */
void MeasurementHw_TIM2IrqDisable(void)
{
  HAL_NVIC_DisableIRQ(TIM2_IRQn);
}

/**
 * @brief 清除 TIM2_IRQn Pending 状态后重新使能 TIM2 中断。
 */
void MeasurementHw_TIM2IrqEnable(void)
{
  HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief HAL Timer Update 回调，只维护底层硬件时间轴和 Gate 状态。
 * @param htim 产生 Update 中断的 Timer 句柄。
 * @note TIM1 Update=外部脉冲计数溢出；TIM2 Update=时间轴溢出；TIM4 Update=本轮 Gate 完成。
 * @note 业务算法不放在该回调中，仅维护底层计数状态。
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
    gate_ready = 1U;
  }
}
