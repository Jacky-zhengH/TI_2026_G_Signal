# STM32F407 + AD9220 + AD603 配置与引脚建议（更新版）

适用工程：`Jacky-zhengH/TI_2026_G_Signal`

目标：在保留现有 `app_process.c` 中 USART1/USART3 与串口屏按钮逻辑的前提下，增加 AD9220 并行采集、AD603 程控增益、8 阶低通补偿和 CMSIS-DSP 频域分析。

> 电赛原则：先跑通固定增益和固定采样，再增加自动增益；所有采集、计算和显示均采用简单顺序流程，不引入 RTOS、消息队列和复杂多层状态机。

---

## 1. 本次更新的关键结论

1. AD9220 模块默认 5 倍衰减已经焊死，输入量程约 10 Vpp。直接测量 50~250 mVpp 时 ADC 有效码数过少，因此在 AD9220 前加入 AD603-VCA 是合理的补救方案。
2. AD603 模块外部控制电压约 0.4~1.446 V，对应标称 -20~60 dB；模块之间存在差异，必须建立本模块自己的“DAC 码值-实际增益”表。
3. AD603 输出阻抗约 51 Ω，AD9220 模块输入阻抗约 50 Ω，二者直接连接会形成近似二分压。软件标定必须包含这约 -6 dB 的负载效应，不能只使用手册增益公式。
4. 当前 8 阶低通实测在 500 kHz 已衰减约 15.22 dB，在 1 MHz 衰减约 43.10 dB。它对第三问抑制 `uJ` 有利，但并不是 10~500 kHz 平坦通带滤波器；必须做幅值和相位补偿。
5. 当前仓库的 HSE 配置对板载 8 MHz 晶体是正确的：HSE 晶体模式、PLLM=8、PLLN=336、PLLP=2，系统时钟 168 MHz。
6. 当前 GPIOC 方案不推荐继续作为 12 位 ADC 总线：PC8~PC12 与板载 TF 卡相连，且仓库中 PC0~PC3 当前被配置为输出。推荐改为 GPIOE 低 12 位连续总线。

---

## 2. 最终信号链

```text
信号源/BNC
  -> 49.9 Ω端接（建议）
  -> 8阶低通滤波器
  -> AD603-VCA程控增益
  -> AD9220模块默认5倍衰减 + AD8132 + AD9220
  -> STM32F407并行DMA采集
  -> FFT定位 + 联合正弦拟合 + 复合频响补偿
  -> TJC8048X270显示
```

注意：滤波器应放在 AD603 前面。这样 1 MHz 以上干扰不会先被 AD603 大幅放大，减少 AD603 和 AD9220 过载风险。

---

## 3. HSE 与系统时钟检查

### 3.1 核心板硬件

核心板原理图采用：

- X1：8 MHz 晶体；
- MCU OSC_IN/OSC_OUT，即 STM32F407 的 PH0/PH1；
- 两侧负载电容均为 10 pF；
- 不是外部有源时钟模块。

因此 CubeMX 必须选择：

```text
RCC -> High Speed Clock (HSE)
Crystal/Ceramic Resonator
```

不能选择 `BYPASS Clock Source`。

### 3.2 当前仓库配置判断

当前配置：

```text
HSE = 8 MHz
PLLM = 8
PLLN = 336
PLLP = 2
PLLQ = 4
SYSCLK = 168 MHz
AHB = 168 MHz
APB1 = 42 MHz，APB1 Timer = 84 MHz
APB2 = 84 MHz，APB2 Timer = 168 MHz
```

结论：

- CPU、TIM1、USART 和 DSP 使用均正确；
- TIM1 位于 APB2，实际定时器时钟为 168 MHz；
- 当前 PLLQ=4 得到 84 MHz，不满足 USB/SDIO/RNG 的 48 MHz 时钟要求；本项目当前不使用这些功能，因此不影响；若以后启用 USB/SDIO，应改为 PLLQ=7。

### 3.3 建议的时钟验证

在第一阶段直接用示波器测量 PA8 的 ADC_CLK：

```text
目标：2.000 MHz，50%占空比
```

软件中保留：

```c
#define ADC_SAMPLE_RATE_HZ 2000000.0f
```

若频率计测得不完全等于 2 MHz，可把实际值写入标定参数，FFT 频率换算使用 `fs_actual`。

---

## 4. 推荐主控引脚对应关系

### 4.1 首选映射：GPIOE 连续 12 位总线

| STM32F407 引脚 | 外设/模块引脚           | 方向      | 配置                | 说明                          |
| -------------- | ----------------------- | --------- | ------------------- | ----------------------------- |
| PA8            | AD9220 CLK              | 输出      | TIM1_CH1 PWM        | 2 MHz、50%占空比              |
| PE0            | AD9220 D0               | 输入      | GPIO Input, No Pull | ADC最低位                     |
| PE1            | AD9220 D1               | 输入      | GPIO Input, No Pull |                               |
| PE2            | AD9220 D2               | 输入      | GPIO Input, No Pull |                               |
| PE3            | AD9220 D3               | 输入      | GPIO Input, No Pull |                               |
| PE4            | AD9220 D4               | 输入      | GPIO Input, No Pull |                               |
| PE5            | AD9220 D5               | 输入      | GPIO Input, No Pull |                               |
| PE6            | AD9220 D6               | 输入      | GPIO Input, No Pull |                               |
| PE7            | AD9220 D7               | 输入      | GPIO Input, No Pull |                               |
| PE8            | AD9220 D8               | 输入      | GPIO Input, No Pull |                               |
| PE9            | AD9220 D9               | 输入      | GPIO Input, No Pull |                               |
| PE10           | AD9220 D10              | 输入      | GPIO Input, No Pull |                               |
| PE11           | AD9220 D11              | 输入      | GPIO Input, No Pull | ADC最高位                     |
| PE12           | AD9220 OTR              | 输入      | GPIO Input, No Pull | 过量程标志                    |
| PE13           | TIM1_CH3                | 输出/未接 | PWM或OC触发         | 仅产生DMA请求，不连接外部模块 |
| PA4            | AD603 P2-2 外部增益控制 | 模拟输出  | DAC_OUT1            | P2跳线帽拔下                  |
| GND            | AD603 P2-3              | 地        | 共地                | DAC控制地                     |
| PA9 / PA10     | TJC屏 TX/RX链路         | UART      | USART1              | 按仓库现状保留                |
| PB10 / PB11    | PC调试串口              | UART      | USART3              | 按仓库现状保留                |
| PC13           | 板载LED                 | 输出      | Active Low          | 可作为采集/错误指示           |
| PH0 / PH1      | 8 MHz晶体               | RCC       | HSE Crystal         | 不作普通GPIO                  |
| PA13 / PA14    | SWDIO/SWCLK             | 调试      | Serial Wire         | 保留下载调试                  |

连续连接后，DMA原始值转换极其简单：

```c
uint16_t code = (uint16_t)(raw_gpioe_idr & 0x0FFFU);
```

### 4.2 不推荐继续使用当前 GPIOC 总线的原因

当前仓库中：

- PC0~PC3 被配置为推挽输出，不符合 AD9220 数据输入要求；
- PC8、PC9、PC10、PC11、PC12 与核心板 TF 卡座及上拉电阻相连；
- 当前映射跳过 PC9，数据整理需要额外位拼接；
- 板载走线和上拉支路会增加高速并行采样风险。

若硬件已经焊接到 GPIOC，最低限度需要：

1. 把所有 12 根数据线改为输入、无上下拉；
2. TF 卡槽中不得插卡；
3. 对跳过 PC9 的位进行明确重排；
4. 使用逻辑分析仪验证 2 MHz 下的数据稳定性。

---

## 5. CubeMX / SysConfig 详细配置

## 5.1 RCC

```text
HSE: Crystal/Ceramic Resonator
HSE_VALUE: 8 MHz
PLL Source: HSE
PLLM: 8
PLLN: 336
PLLP: /2
PLLQ: /4（当前不用USB/SDIO时保留）
SYSCLK: PLLCLK 168 MHz
AHB: /1
APB1: /4
APB2: /2
Flash Latency: 5 WS
Voltage Scale: Scale 1
```

## 5.2 SYS

```text
Debug: Serial Wire
Timebase Source: SysTick
```

## 5.3 TIM1：AD9220采样时钟与DMA触发

TIM1 时钟为 168 MHz。

### 基础参数

```text
Prescaler = 0
Counter Mode = Up
Auto Reload / Period = 84 - 1
Repetition Counter = 0
Clock Division = DIV1
Auto Reload Preload = Disable
```

得到：

```text
Fs = 168 MHz / 84 = 2.000 MHz
```

### Channel 1：输出给 AD9220 CLK

```text
Mode = PWM Generation CH1
Pin = PA8
Pulse = 42
Polarity = High
Fast Mode = Disable
GPIO Speed = Very High
```

得到 50% 占空比。

### Channel 3：触发一次GPIO读取DMA

当前仓库使用：

```text
TIM1_CH3
Pulse = 63
DMA Request = TIM1_CH3
```

CH3 比较事件发生在周期的 75% 位置，即采样时钟上升沿后约 375 ns，数据稳定裕量充足。

电赛简化方案：

- 保留 PE13 为 TIM1_CH3，但 PE13 不接外部模块；
- CH3 的目的只是每周期产生一次 DMA 请求；
- 不用 CH3 波形参与任何硬件功能。

若 CubeMX 支持 `Output Compare No Output`，可使用无输出比较模式，释放 PE13；不是必须修改项。

## 5.4 DMA2 Stream6 / Channel6

请求源：`TIM1_CH3`

```text
Instance: DMA2_Stream6
Channel: DMA_CHANNEL_6
Direction: Peripheral to Memory
Peripheral increment: Disable
Memory increment: Enable
Peripheral data width: Word
Memory data width: Word
Mode: Normal
Priority: Very High
FIFO: Disable
Interrupt: Enable
```

DMA 的外设源地址必须手动指定为：

```c
(uint32_t)&GPIOE->IDR
```

内存目标：

```c
static uint32_t adc_raw[4096];
```

不要直接使用 `HAL_TIM_PWM_Start_DMA()` 作为GPIO采集接口，因为该HAL函数按“内存向CCR写数据”的PWM DMA用途设计。建议在 `bsp_ad9220.c` 中手动启动：

```c
HAL_DMA_Start_IT(&hdma_tim1_ch3,
                 (uint32_t)&GPIOE->IDR,
                 (uint32_t)adc_raw,
                 ADC_SAMPLE_COUNT);

__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC3);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
```

完成后依次：

```c
HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
__HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC3);
```

DMA中断只设置：

```c
adc_capture_done = 1U;
```

不要在 DMA 回调中执行 FFT、串口发送或浮点计算。

## 5.5 GPIO

```text
PE0~PE12: Input, No Pull
PA8: Alternate Function TIM1_CH1, Very High Speed
PE13: Alternate Function TIM1_CH3，可不外接
PC13: Output Push Pull，默认高电平（LED灭）
```

## 5.6 DAC：AD603增益控制

新增 DAC1 Channel 1：

```text
Pin: PA4 / DAC_OUT1
Trigger: None
Output Buffer: Enable
```

AD603 模块：

1. 拔掉 P2 的 1-2 跳线帽；
2. PA4 接 P2-2；
3. STM32 GND 接 P2-3；
4. DAC 输出范围仅使用约 0.4~1.446 V。

初始换算：

```c
uint16_t dac_code = (uint16_t)(v_ctrl / 3.3f * 4095.0f + 0.5f);
HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,
                 DAC_ALIGN_12B_R, dac_code);
```

手册标称关系：

```text
G_dB = 76.4812 * Vctrl - 50.592
Vctrl = (G_dB + 50.592) / 76.4812
```

该公式只能用于初始设置。实际使用必须标定：

```text
Vctrl -> 整个链路实际增益
```

整个链路包括：8阶滤波、AD603、AD603 51Ω输出、AD9220 50Ω输入、AD9220默认5倍衰减。

## 5.7 NVIC建议

```text
DMA2_Stream6_IRQn: Preemption Priority 1
USART1_IRQn: Priority 5
DMA2_Stream2_IRQn: Priority 5
SysTick: Priority 15
```

项目不使用 RTOS，优先保证采集完成中断，不需要复杂中断优先级设计。

---

## 6. CMSIS-DSP 配置

需要 DSP 库，推荐直接使用 CMSIS-DSP：

```c
#include "arm_math.h"
```

核心函数：

```c
arm_rfft_fast_init_f32(&rfft, 4096);
arm_rfft_fast_f32(&rfft, fft_input, fft_output, 0);
arm_cmplx_mag_f32(...);
```

编译设置：

```text
ARM_MATH_CM4
FPU = FPv4-SP-D16
Float ABI = Hard
```

Keil 可链接 Cortex-M4F 浮点版本 DSP 库；GCC/CMake 可加入 CMSIS-DSP 源码或对应预编译库。

当前 `.ioc` 中启用了 X-CUBE-ALGOBUILD 的 DSP Library 选项，但本项目不需要使用 AlgoBuilder 生成算法。只要 CMSIS-DSP 头文件和库正确加入即可。

若编译出现 `__FPU_PRESENT` 重复定义警告，应删除工程中手工重复定义，保留 STM32F407 设备头文件中的定义。

---

## 7. 软件目录与模块边界

```text
User/
├─ bsp/
│  ├─ bsp_ad9220.c/h       并行DMA采集、OTR检测
│  └─ bsp_ad603.c/h        DAC控制、固定档位增益
├─ alog/
│  ├─ alog_signal.c/h      FFT、谱峰、拟合、RMS、Upp
│  └─ alog_calib.c/h       频响、相位、增益标定表
└─ app/
   └─ app_process.c/h      保留现有UART/HMI，增加顺序调用
```

不新增 manager/service/event-bus 等层。

### `bsp_ad9220` 推荐接口

```c
void BSP_AD9220_Init(void);
bool BSP_AD9220_Capture(uint32_t *buffer,
                        uint32_t count,
                        uint32_t timeout_ms);
bool BSP_AD9220_IsOverrange(void);
```

### `bsp_ad603` 推荐接口

```c
void BSP_AD603_Init(void);
void BSP_AD603_SetVoltage(float voltage_v);
void BSP_AD603_SetLevel(uint8_t level);
uint8_t BSP_AD603_GetLevel(void);
```

先只做 5~6 个离散增益档，不做连续闭环 AGC。

### `alog_signal` 推荐接口

```c
bool ALOG_Signal_Analyze(const uint32_t *raw,
                         uint32_t count,
                         uint8_t gain_level,
                         SignalResult_t *result);
```

---

## 8. 保留现有 app_process 串口逻辑

当前仓库的真实分配为：

```text
USART1：HMI 串口屏，PA9/PA10，9600 baud
USART3：电脑调试，PB10/PB11，115200 baud
```

`app_process.c` 中：

- `huart1` 用于 HMI 指令发送和单字节按钮接收；
- `huart3` 用于 `Debug_printf()`；
- USART1 接收回调解析 `0xA1 / 0xA3 / 0xAF`。

本次不修改 UART 驱动和按钮协议，只在 `App_Main_Process_Poll()` 中增加：

```text
按键处理
-> 到达刷新周期
-> 预采样/选择增益
-> 正式采样
-> ALOG分析
-> 更新当前页面
```

由于 HMI 当前只有 9600 baud，波形刷新应控制：

- 每次发送约 160~240 个显示点；
- 刷新率 2~4 Hz；
- 数值变化明显时再更新文本；
- 不向屏幕发送 4096 个原始点。

---

## 9. AD603 离散增益与自动量程建议

### 9.1 初始档位

以下仅为启动值，必须实测修正：

| 档位 | Vctrl 初值 | 手册标称增益 |
| ---: | ---------: | -----------: |
|    0 |     0.45 V |     -16.2 dB |
|    1 |     0.65 V |      -0.9 dB |
|    2 |     0.85 V |      14.4 dB |
|    3 |     1.05 V |      29.7 dB |
|    4 |     1.25 V |      45.0 dB |
|    5 |     1.40 V |      56.5 dB |

### 9.2 简洁自动量程

1. 先用档位 1 或 2 预采样 1024 点；
2. 计算去直流后的最大绝对码值；
3. 选择使正式采样占 ADC 有效范围 40%~75% 的最高安全档；
4. DAC切换后等待 2~5 ms；
5. 正式采样 4096 点；
6. 若 OTR 或峰值超过 90%，退一档并仅重采一次；
7. 一帧计算期间冻结增益。

最多允许一次重采，不写多层 AGC 状态机。

---

## 10. 8阶低通滤波器纳入软件标定

根据当前 3 Vpp 实测：

|    频率 |      输出 |    增益dB | 初始幅值修正倍数 |
| ------: | --------: | --------: | ---------------: |
|  10 kHz |  3.04 Vpp |  +0.12 dB |            0.987 |
| 100 kHz |  2.83 Vpp |  -0.51 dB |            1.060 |
| 200 kHz |  2.28 Vpp |  -2.38 dB |            1.316 |
| 250 kHz |  1.94 Vpp |  -3.79 dB |            1.546 |
| 300 kHz |  1.60 Vpp |  -5.46 dB |            1.875 |
| 400 kHz | 0.957 Vpp |  -9.92 dB |            3.135 |
| 450 kHz | 0.713 Vpp | -12.48 dB |            4.208 |
| 500 kHz | 0.520 Vpp | -15.22 dB |            5.769 |
|   1 MHz | 0.021 Vpp | -43.10 dB | 不作有效信号补偿 |

判断：

- 约 222 kHz 已达到 -3 dB；
- 500 kHz 有效分量被明显削弱；
- 1 MHz 干扰抑制较好；
- 该表只能作为初始参考，最终必须在“滤波器 + AD603 + AD9220实际负载”条件下重测。

标定表至少包含：

```c
frequency_hz
magnitude_correction[gain_level]
phase_correction_rad[gain_level]
```

只校正幅值不够。多谐波波形的峰峰值与各分量相位有关，必须测量滤波器/链路相移，才能重构输入端原始波形并准确计算 Upp。

---

## 11. TJC8048X270 页面建议

保持一个主页面，避免多页面状态同步：

```text
顶部：Upp、Urms、基频、当前增益档、采样状态
中部：大曲线区域
底部：1周期、3周期、频谱 三个按钮
右侧：最多3组分量频率和幅值
状态栏：OK / NO SIGNAL / OTR / ADC TIMEOUT / CALIBRATION
```

按钮命令继续使用：

```text
0xA1：1周期
0xA3：3周期
0xAF：频谱
```

---

## 12. 推荐调试顺序

1. 确认 HSE 与 168 MHz 系统时钟；测 PA8 为 2 MHz。
2. 固定 AD603 手动增益，示波器确认不失真。
3. DMA 采集 GPIOE 4096 点，串口输出前 32 个码。
4. 固定单正弦完成 FFT 和频率测量。
5. 在一个固定增益档完成幅值标定。
6. 加入多个增益档和离散自动量程。
7. 建立 10~500 kHz 全链路幅频表。
8. 建立相频表，完成波形重构和 Upp。
9. 加入 1 MHz 以上干扰，验证第三问。
10. 最后接入 HMI 波形和频谱刷新。

---

## 13. 当前必须避免的做法

- 不直接把手册 AD603 增益公式当作最终测量增益；
- 不直接对含 `uJ` 的原始样本求 RMS；
- 不使用原始样本最大值减最小值作为 500 kHz 多谐波信号 Upp；
- 不在每个采样点进入中断；
- 不在 DMA 回调中运行 FFT；
- 不在一个采样帧中连续改变 AD603 增益；
- 不继续把 PC8~PC12 当作理想无负载数据总线；
- 不只做幅值补偿而忽略滤波器相位补偿。
