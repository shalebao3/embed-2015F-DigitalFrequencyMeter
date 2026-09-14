# embed-2015F-DigitalFrequencyMeter

基于 **STM32F103C8T6 + STM32CubeMX + HAL** 的数字频率计学习项目。

项目以 2015 年全国大学生电子设计竞赛 F 题《数字频率计》为背景，目标不是先把所有竞赛指标一次性堆完，而是通过一个真实测量项目把 STM32 Timer 的关键能力串起来：**PWM、输入捕获、外部脉冲计数、PWM Input、One Pulse、Master/Slave、TRGO、Update Event、中断、溢出扩展、测量有效性与多定时器协同**。

当前核心测量代码已经覆盖：

- 频率测量
- 周期测量
- A→B 时间间隔测量
- 占空比测量
- 低频周期法 / 高频闸门法自动切换
- 测量超时与结果有效性
- GitHub Actions ARM 交叉编译验证

> 当前阶段的软件测量骨架已经基本成型，但 **真实精度、最高输入频率、模拟前端和完整竞赛指标仍需要实板验证**。

---

## 当前仪器模式

软件目前划分为 4 种仪器功能模式：

```c
typedef enum
{
  INSTRUMENT_MODE_FREQUENCY = 0,
  INSTRUMENT_MODE_PERIOD,
  INSTRUMENT_MODE_DUTY,
  INSTRUMENT_MODE_INTERVAL
} InstrumentMode;
```

对应：

| 模式 | 功能 | 主要硬件路径 |
| --- | --- | --- |
| `FREQUENCY` | 测频率 | TIM2 周期法 + TIM1/TIM4 闸门法自动切换 |
| `PERIOD` | 测周期 | 共用频率测量引擎，再换算周期 |
| `DUTY` | 测占空比 | TIM2 PWM Input + TIM1/TIM4 粗频率辅助自动选择 PSC |
| `INTERVAL` | 测 A→B 时间间隔 | TIM2_CH1 + TIM2_CH2 双路输入捕获 |

模式切换统一通过：

```c
Instrument_SetMode(...)
```

后续接按键、串口菜单或 TFT/LVGL 时，不需要让显示层直接操作底层 Timer。

---

## 定时器资源分配

| 定时器 | 当前职责 | 主要模式 | 关键引脚 | 作用 |
| --- | --- | --- | --- | --- |
| **TIM1** | 外部脉冲计数器 | ETR External Clock Mode 2 + Gated Slave Mode | PA12 / TIM1_ETR | 在 TIM4 打开的 1 秒 Gate 内统计外部脉冲 |
| **TIM2** | 高分辨率时间测量核心 | Input Capture / PWM Input | PA0 / CH1，PA1 / CH2 | 周期、A→B 时间间隔、占空比 |
| **TIM3** | 内部测试信号源 | PWM Generation CH1 | PA6 / TIM3_CH1 | 输出约 1 kHz、50% 占空比测试 PWM |
| **TIM4** | 1 秒硬件闸门 | One Pulse + TRGO Enable | 无外部输入 | 产生 1 秒 Gate，通过 ITR3 控制 TIM1 |

当前核心关系：

```text
                 被测数字信号
                /           \
               /             \
        PA0 / TIM2          PA12 / TIM1_ETR
            │                     │
   周期 / PWM Input          外部脉冲计数
            │                     │
            │                 TIM1 Gated
            │                     ▲
            │                     │ ITR3
            │                 TIM4 TRGO
            │                 1 s One Pulse
            │
            └────── 软件策略层 ──────┘
```

测试时可以使用 TIM3 的 PA6 作为已知信号源，再把同一信号分配到对应测量输入。

---

## TIM3：PWM 测试信号源

TIM3_CH1 当前用于输出约 **1 kHz、50% 占空比** 的测试信号。

```text
Timer Clock = 72 MHz
PSC         = 71
ARR         = 999
CCR1        = 500
```

因此：

```text
72 MHz / (71 + 1) = 1 MHz
1 MHz / (999 + 1) = 1 kHz
Duty ≈ 500 / 1000 = 50%
```

TIM3 启动后完全由硬件持续产生 PWM，不需要 CPU 在主循环里翻转 GPIO。

---

## TIM2：72 MHz 高分辨率时间轴

普通周期 / 时间间隔模式下 TIM2 使用：

```text
Timer Clock = 72 MHz
PSC         = 0
ARR         = 65535
```

因此：

```text
1 tick = 1 / 72 MHz ≈ 13.8889 ns
```

TIM2 是 16 位定时器，所以使用：

```text
tim2_overflow_count
```

把硬件 CNT 扩展成软件 64 位时间轴：

```text
extended_timestamp = overflow_count × 65536 + CCRx
```

代码同时处理了 **Capture 与 Update 几乎同时发生** 的边界情况，避免硬件已经溢出但软件溢出计数尚未来得及更新造成时间戳错误。

---

## 频率测量：周期法 + 1 秒闸门法自动切换

### 1. 低频 / 中频：TIM2 周期法

TIM2_CH1 / PA0 捕获相邻两个上升沿：

```text
上升沿 1 → timestamp1
上升沿 2 → timestamp2

period_ticks = timestamp2 - timestamp1
```

然后计算：

```text
frequency_hz = 72,000,000 / period_ticks
period_ns    = period_ticks × 1e9 / 72,000,000
```

周期越长，TIM2 能获得的 tick 越多，低频时分辨率越好。

### 2. 高频：TIM1 + TIM4 硬件闸门法

TIM1 配置为：

```text
Clock Source = ETR Mode 2
Input        = PA12 / TIM1_ETR
Slave Mode   = Gated
Trigger      = ITR3
PSC          = 0
ARR          = 65535
```

TIM4 配置为：

```text
Timer Clock = 72 MHz
PSC         = 7199
ARR         = 9999
One Pulse   = Single
TRGO        = Enable
```

所以：

```text
72 MHz / (7199 + 1) = 10 kHz
10000 tick = 1 s
```

工作过程：

```text
TIM1 CEN = 1
TIM4 尚未启动
↓
TIM4 TRGO = LOW
TIM1 Gate 关闭

TIM4 启动 One Pulse
↓
TRGO = HIGH
↓
TIM1 Gate 打开
↓
PA12 每个有效脉冲推动 TIM1_CNT + 1

1 秒后 TIM4 自动停止
↓
TRGO = LOW
↓
TIM1 Gate 关闭
↓
主循环读取 TIM1 CNT + overflow
```

因为窗口正好为 1 秒：

```text
gate_frequency_hz ≈ 1 秒内统计到的脉冲数
```

### 自动切换与迟滞

理论上周期法和 1 秒闸门法的量化误差在约 8.5 kHz 附近交叉。

当前实现采用迟滞区：

```text
周期法 → 闸门法：约 >= 10 kHz
闸门法 → 周期法：约 <= 7 kHz
```

即：

```text
< 7 kHz          7~10 kHz             > 10 kHz
周期法       保持当前测量方法            闸门法
```

避免临界点附近反复切换。

另外，当 TIM2 周期法判断频率已经进入高频区域后，会立即关闭 CH1 捕获中断，避免 MHz 级输入产生海量 ISR 把 CPU 淹没。

最终给上层使用的统一频率结果为：

```c
measured_frequency_hz
```

周期结果为：

```c
measured_period_ns
```

---

## A→B 时间间隔测量

`INTERVAL` 模式下：

```text
PA0 / TIM2_CH1 = A
PA1 / TIM2_CH2 = B
```

流程：

```text
A 上升沿
↓
记录 interval_start_timestamp
↓
等待 B
↓
B 上升沿
↓
记录 interval_end_timestamp
↓
interval_ticks = B - A
↓
换算 interval_ns
```

当前代码同时维护：

```c
interval_ns
interval_waiting_ch2
interval_valid
```

分别表示：

```text
interval_ns          → 测量值
interval_waiting_ch2 → 当前是否已经收到 A、正在等待 B
interval_valid       → 当前结果是否可信
```

题目 A→B 时间间隔上限为 100 ms，当前软件等待 B 的超时设置为：

```text
200 ms
```

如果 B 超时未到：

```text
放弃旧 A
↓
清除旧结果
↓
重新等待新的 A
```

CH2 ISR 内也会再次检查超时，避免“迟到的 B”错误匹配旧 A。

---

## 占空比测量：TIM2 PWM Input

`DUTY` 模式下，TIM2 会在运行时临时切换成 **PWM Input**。

这里仍然只需要一根输入：

```text
PA0 / TIM2_TI1
```

但同一根 TI1 会同时映射到两个 Capture Channel：

```text
PA0 / TI1
   │
   ├── CH1 Direct TI1   → 上升沿 → CCR1 = 周期
   │
   └── CH2 Indirect TI1 → 下降沿 → CCR2 = 高电平时间
```

同时 TIM2 使用 Reset Mode：

```text
每个 TI1 上升沿
↓
锁存 CCR1
↓
CNT 自动归零
↓
重新开始下一周期
```

因此：

```text
CCR1 = 完整周期 tick
CCR2 = 高电平 tick
```

占空比：

```text
Duty = CCR2 / CCR1 × 100%
```

软件保存为千分数：

```c
measured_duty_permille
```

例如：

```text
500 → 50.0%
333 → 33.3%
725 → 72.5%
```

### 为什么 DUTY 不使用每边沿中断

高频 PWM 如果每个上升沿、下降沿都触发 ISR，会迅速超过 F103 的 CPU 处理能力。

因此 DUTY 模式中：

```text
CC1 interrupt = OFF
CC2 interrupt = OFF
```

CCR1 / CCR2 由硬件持续更新，主循环只轮询最新 Capture 结果。

### DUTY 的自动 PSC

TIM2 只有 16 位，低频信号在 `PSC=0` 时周期会超过 65535 tick。

当前 DUTY 模式会继续利用 TIM1 + TIM4 的 1 秒闸门得到一个粗频率，然后动态计算 TIM2 PSC，目标让一个周期尽量落在：

```text
<= 60000 tick
```

这样可以兼顾：

- 低频不溢出
- 高频尽量保持更高时间分辨率

占空比测量也带有：

```c
duty_valid
```

和约 1500 ms 的无有效 Capture 超时处理。

> 注意：F103 的 TIM2 时钟只有 72 MHz。到 MHz 级时，每周期可用 tick 数会明显下降，因此极高频端占空比精度最终仍受硬件时间分辨率限制，需要实测验证。

---

## 测量结果有效性与超时

当前代码不再把“变量里有旧数值”等同于“现在仍然测量有效”。

主要有效性状态包括：

```c
frequency_valid
interval_valid
duty_valid
gate_frequency_valid
```

典型思想：

```text
测量值
+
测量状态
+
超时
+
重新同步
```

例如频率周期法超过约 1500 ms 没有新的 CH1 上升沿，会清除旧结果并重新等待两个有效边沿。

---

## 当前关键结果变量

| 变量 | 作用 |
| --- | --- |
| `measured_frequency_hz` | 自动选择周期法 / 闸门法后的最终频率 |
| `measured_period_ns` | 最终周期结果 |
| `measured_duty_permille` | 占空比，0~1000 对应 0.0%~100.0% |
| `interval_ns` | A→B 时间间隔 |
| `frequency_valid` | TIM2 周期法结果是否有效 |
| `duty_valid` | DUTY PWM Input 结果是否有效 |
| `interval_valid` | A→B 结果是否有效 |
| `gate_frequency_hz` | TIM1 + TIM4 1 秒闸门测频结果 |
| `gate_ready` | TIM4 本轮 1 秒 One Pulse 是否结束 |
| `tim1_overflow_count` | TIM1 16 位外部脉冲计数扩展 |
| `tim2_overflow_count` | TIM2 16 位时间轴扩展 |

---

## GitHub Actions：STM32 ARM 自动编译

仓库已经加入：

```text
.github/workflows/stm32-build.yml
```

CI 使用仓库现有的：

```text
CMake
CMakePresets.json
Ninja
arm-none-eabi-gcc
```

自动执行：

```text
push / PR
↓
安装 ARM GCC 工具链
↓
cmake --preset Debug
↓
cmake --build --preset Debug
↓
ARM 编译 + 链接
↓
arm-none-eabi-size
↓
生成并上传
firmware.elf
firmware.hex
firmware.bin
firmware.map
```

当前加入 DUTY 模式后的 CI 已成功通过。

最近一次成功构建资源占用：

```text
RAM   : 2032 B / 20 KB ≈ 9.92%
FLASH : 18164 B / 64 KB ≈ 27.72%
```

这意味着后续每次修改 `firmware/**` 后，都可以让 GitHub Actions 自动充当 ARM 编译守门员。

---

## 当前完成度

### 已实现

- TIM3 1 kHz / 50% PWM 测试信号源
- TIM2 72 MHz 高分辨率时间轴
- TIM2 周期法测频
- TIM2 16 位溢出扩展
- Capture / Update 边界处理
- TIM1 ETR 外部脉冲计数
- TIM1 软件溢出扩展
- TIM4 1 秒 One Pulse 硬件闸门
- TIM4 TRGO → TIM1 ITR3 → Gated Mode
- 周期法 / 闸门法自动切换
- 7~10 kHz 迟滞区
- 高频自动关闭 TIM2_CH1 捕获中断
- 频率无信号超时
- 周期测量
- A→B 双路时间间隔测量
- A→B 有效性、超时和重新同步
- DUTY 占空比模式
- TIM2 PWM Input
- DUTY 动态 PSC
- DUTY 有效性与超时
- 仪器模式框架
- GitHub Actions ARM Debug 真编译
- ELF / HEX / BIN / MAP 自动生成

### 尚未完成 / 需要实测

- 开发板实测
- 1 Hz ~ 10 MHz 全范围误差测试
- 7~10 kHz 自动方法切换实测
- TIM1 ETR 高频极限验证
- 1 秒 Gate 实际误差验证
- A→B 0.1 us ~ 100 ms 全范围验证
- 占空比 1 Hz ~ 5 MHz、10%~90% 全范围验证
- 极高频下 TIM2 时间量化误差评估
- OLED / TFT 显示
- 按键或菜单模式切换
- Hz / kHz / MHz、ns / us / ms / s 自动单位显示
- 正弦波输入的放大、比较、施密特整形和输入保护
- 完整竞赛性能指标验证

---

## 推荐硬件自测接线

使用 TIM3 自带测试 PWM 时，可先验证 1 kHz / 50%：

```text
TIM3 PA6
   │
   ├────> PA0  / TIM2_CH1
   │
   └────> PA12 / TIM1_ETR
```

预期：

```text
Frequency ≈ 1000 Hz
Period    ≈ 1 ms
Duty      ≈ 50.0%
```

A→B 时间间隔模式则需要两路独立数字输入：

```text
A → PA0 / TIM2_CH1
B → PA1 / TIM2_CH2
```

---

## 当前项目定位

当前仓库更准确的定位是：

> **围绕 2015 F 数字频率计展开的 STM32 定时器综合测量原型。**

项目已经不再只是单个 Timer 的练习，而是在同一个工程里同时处理：

```text
PWM
Input Capture
PWM Input
External Clock
One Pulse
Master / Slave
TRGO / ITR
Gated Mode
NVIC / ISR
16 位溢出扩展
超时与有效性
自动测量策略
GitHub Actions ARM CI
```

下一阶段重点将从“继续堆定时器功能”转向：**实板验证、显示层、硬件输入前端和完整性能测试**。
