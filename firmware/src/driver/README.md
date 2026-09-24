# driver：STM32 片内外设驱动层（Driver）

放置 TIM、输入捕获、硬件 Gate、扩展时间戳等片内外设驱动。

当前 `measurement_hw.c/.h` 负责 TIM1~TIM4 的底层启动、模式重配置、Gate 快照和时间戳构造。

边界：Driver 负责“硬件怎么测”，不负责“当前用户选择哪一种仪器模式”。
