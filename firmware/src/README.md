# src：项目自定义代码

本目录只放项目自行维护的代码，目录语义与 `embed-stm32c8t6-template` 对齐。

- `app/`：赛题业务、状态机和测量算法编排。
- `driver/`：STM32 片内外设的项目驱动。
- `bsp/`：与当前板级硬件直接相关的器件适配。
- `common/`：可跨题目复用的纯通用组件。

本仓库仍使用 CubeMX + HAL，因此 `main.c`、中断文件和 HAL 初始化代码继续保留在 CubeMX 的 `Core/` 中。
