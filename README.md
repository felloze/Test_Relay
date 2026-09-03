# Test_Relay — 继电器触点测试仪固件

基于 **合宙 AIR001**（Cortex-M0+, AirMCU 0.6.4 Arduino BSP）的继电器触点类型识别与压力测试工具。

## 功能

- **触点类型识别**：长按功能键，自动完成继电器触点类型检测，判定 8 种触点组合（1A/1B/1C/2A/2B/2C/AB/BA），TM1650 数码管显示识别结果（1s 后恢复电压显示）
- **压力测试**：识别完成后自动开启。吸合/释放两种状态下持续监测触点状态，任一判据不满足即驱动蜂鸣器报警，用于机械参数测试
- **测试键**：点动模式（跟随按下/释放
- **电压切换**：功能键单击循环切换 5V / 12V / 24V（控制升压使能与切换引脚），数码管实时显示

## 可靠性设计

针对继电器动作带来的强 EMI 与机械延时，做了以下处理：

| 措施 | 说明 |
|---|---|
| 非阻塞状态机 | 触点识别、蜂鸣器时序、按键扫描全部非阻塞，主循环不再有 `delay(300)` 长阻塞窗口 |
| 继电器动作保护窗 | 测试键按下/释放（或识别流程）使继电器动作后，`RELAY_GUARD_MS`（50ms，覆盖 ~10ms 机械延时 + 弹跳）内 ForceTest 静音待判，避免「新继电器状态 + 旧触点值」判据自相矛盾而误鸣 |
| 灵敏报警判定 | 保护窗外 ForceTest 直接读触点引脚原始电平、逐主循环判定，无滤波拖累——压力测试中真实故障的报警延迟即主循环周期 |
| 统一继电器入口 | 所有继电器动作走 `setRelay()`，`RELAY_STATE` 与实际驱动电平严格同步 |
| 引脚抗干扰 | 功能键 `INPUT_PULLDOWN`、触点采样 `INPUT_PULLDOWN`、TM1650 ACK 引脚上拉输入 + 短超时 |
| IWDG 独立看门狗 | 2s 超时（LSI 32kHz / 64 分频），跑飞自动复位；启动时通过串口报告上次复位原因 |

关键可调参数集中在 `Test_Relay.ino` 头部：

```cpp
#define USE_IWDG 1         // 0 = 编译期完全剥离看门狗
#define IWDG_TIMEOUT_MS 2000
#define RELAY_GUARD_MS 50  // 继电器动作保护窗，需 > 机械延时 + 弹跳
```

## 硬件连接

| 引脚 | 功能 |
|---|---|
| PF1 / PF0 | TM1650 时钟 / 数据 |
| PA4 | 测试键（内部上拉，按下为低） |
| PB6 | 功能键（外部下拉，按下为高） |
| PB2 / PB1 | 升压使能 / 升压切换 |
| PA5 | 继电器控制 |
| PB3 | 蜂鸣器 |
| PA0 / PA7 | 常开触点 NO1 / NO2 采样 |
| PA1 / PB0 | 常闭触点 NC1 / NC2 采样 |

## 编译环境

- Arduino IDE + 合宙 AirMCU 板级包（`AirMCU 0.6.4`）
- 库：`OneButton 2.6.1`、TM1650（随项目提供）、内置 EEPROM
- AIR001 时钟：HSI 24MHz + PLL → 48MHz（未使用 HSE，PF0/PF1 可作普通 GPIO）

## 已知坑（AirMCU BSP）

- **HAL 的 IWDG 不可用**：`cores/AirMCU/air/airyyxx_hal_conf.h` 把 `HAL_IWDG_MODULE_ENABLED` 放在 `#if 0` 的 "Unused HAL modules" 块内，`HAL_IWDG_Init / HAL_IWDG_Refresh` 链接报 undefined reference。本项目改用 **LL 驱动**（`air001xx_ll_iwdg.h`，纯头文件 static inline）。
- 同在该 `#if 0` 块内的还有 CRC / DAC / EXTI / CAN / COMP / WWDG / USART 等，使用这些外设的 HAL 版本需自行开启模块开关。

## 作者

Latiaomaibu
