# app：业务编排层（App）

2015F 的仪器模式、状态机与测量算法放在这里。

当前包含 instrument、frequency_meter、duty_meter、interval_meter。

边界：App 可以调用 driver、bsp 和 common；不要在这里继续堆 TIM 寄存器重配置和 HAL 外设启动细节。
