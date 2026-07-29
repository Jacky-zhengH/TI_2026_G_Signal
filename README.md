# 2026 电赛 G 题：STM32F407 配置与 TJC8048X270 UI 建议

> 目标：以 STM32F407 + AD9220 + TJC8048X270 完成周期信号测量分析装置。配置遵循电赛原则：少模块、少状态、先通链路、可快速定位问题。

## 1. 先确定的总体约束

- 软件分为 `bsp / alog / app` 三层。
- 不使用 FreeRTOS；不使用动态内存；不建立消息总线、设备注册表或多层状态机。
- `USART1` 和 `USART3` 已经集成到 `app_process`，保持现有驱动及屏幕按键解析逻辑，不做重构。
- AD9220 采集必须使用定时器 + DMA，不使用“每个采样点进入一次中断”的参考例程方式。
- 算法使用 CMSIS-DSP 的 4096 点实数 FFT；数据结果使用 `float32_t`。
- 首先跑通 `2 MSPS + 4096 点` 基础方案；第三项抗 1 MHz 干扰不足时，再启用 `4 MSPS 原始采集 + FIR 二抽一 + 4096 点 FFT`。

## 2. 与当前仓库保持一致的内容

当前仓库已经配置：

- STM32F407，主频目标 168 MHz；
- `USART1`：PA9/PA10；
- `USART3`：PB10/PB11；
- GPIO、DMA、USART 初始化函数已经由 CubeMX 生成；
- CMSIS-DSP 头文件和 Cortex-M4F DSP 库已经出现在工程配置中。

保持下面的调用关系：

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();

    APP_Process_Init();

    while (1)
    {
        APP_Process_Run();
    }
}
```

`main.c` 只负责初始化和调用，不放采样、FFT、屏幕协议或参数计算代码。

---

# 3. 推荐引脚分配

## 3.1 AD9220 并行数据与时钟

为了一次读取 `GPIOC->IDR` 得到完整 12 位数据，推荐把 12 根数据线连续放在 PC0~PC11。

| 功能       | STM32F407 引脚 | CubeMX 模式  | 说明                    |
| ---------- | -------------- | ------------ | ----------------------- |
| AD9220 D0  | PC0            | GPIO_Input   | ADC 最低位              |
| AD9220 D1  | PC1            | GPIO_Input   |                         |
| AD9220 D2  | PC2            | GPIO_Input   |                         |
| AD9220 D3  | PC3            | GPIO_Input   |                         |
| AD9220 D4  | PC4            | GPIO_Input   |                         |
| AD9220 D5  | PC5            | GPIO_Input   |                         |
| AD9220 D6  | PC6            | GPIO_Input   |                         |
| AD9220 D7  | PC7            | GPIO_Input   |                         |
| AD9220 D8  | PC8            | GPIO_Input   |                         |
| AD9220 D9  | PC9            | GPIO_Input   |                         |
| AD9220 D10 | PC10           | GPIO_Input   |                         |
| AD9220 D11 | PC11           | GPIO_Input   | ADC 最高位              |
| AD9220 OTR | PC12           | GPIO_Input   | 高电平表示过量程        |
| AD9220 CLK | PA8            | TIM1_CH1 PWM | 2 MHz 或 4 MHz 采样时钟 |

GPIO 建议：

- PC0~PC12：`No pull`；
- GPIO 速度对输入无意义，保持默认即可；
- `ADC_CLK` PA8：复用推挽、Very High Speed、No Pull；
- 数据线尽量等长、短、同一排连接；时钟线旁边配地线，避免长杜邦线平行串扰。

读取方式：

```c
uint16_t code = (uint16_t)(GPIOC->IDR & 0x0FFFU);
```

参考程序的电压换算中存在极性反向关系，因此第一次接线后必须用直流正负电压或低频正弦确认：输入上升时 ADC 码是上升还是下降。最终在 `bsp_ad9220` 中统一做一次极性处理，不要把 `4095-code` 分散到算法层。

## 3.2 保留的串口引脚

| 串口      | 引脚 | 现有用途      | 建议                  |
| --------- | ---- | ------------- | --------------------- |
| USART1_TX | PA9  | 电脑调试      | 保持现有配置          |
| USART1_RX | PA10 | 电脑调试/命令 | 保持现有配置及 RX DMA |
| USART3_TX | PB10 | TJC 屏幕      | 保持现有配置          |
| USART3_RX | PB11 | TJC 按键事件  | 保持现有配置          |

不要把 AD9220 数据线改到 GPIOA 或 GPIOB，否则会与现有 UART 引脚产生冲突，也失去“一次读端口”的优势。

---

# 4. 时钟配置

## 4.1 必须优先使用 HSE

当前工程使用 HSI 经过 PLL 得到 168 MHz。开发初期可运行，但最终测频建议使用外部晶振 HSE：

- 题目基频误差要求不超过 1 kHz；
- 在 500 kHz 处，相对误差上限只有约 0.2%；
- FFT 插值只能降低“频点量化误差”，不能修复采样时钟整体偏差；
- HSE 通常比内部 RC 的频率稳定性更适合测量仪器。

以 8 MHz HSE 为例：

| 参数       |                       设置 |
| ---------- | -------------------------: |
| PLL Source |                        HSE |
| PLLM       |                          8 |
| PLLN       |                        336 |
| PLLP       |                          2 |
| PLLQ       |                          7 |
| SYSCLK     |                    168 MHz |
| AHB        |                    168 MHz |
| APB1       |  42 MHz，定时器时钟 84 MHz |
| APB2       | 84 MHz，定时器时钟 168 MHz |

如果开发板晶振不是 8 MHz，按实际晶振重新计算，不要照抄 PLLM。

## 4.2 TIM1 基础采样参数

### 基础模式：2 MSPS

```text
TIM1 clock = 168 MHz
PSC = 0
ARR = 83
PWM frequency = 168 MHz / (83 + 1) = 2 MHz
CH1 CCR1 = 42       // 约 50% 占空比，输出到 PA8
CH3 CCR3 = 63       // 内部比较事件，延后读取稳定后的数据
```

### 增强模式：4 MSPS 原始采样

```text
TIM1 clock = 168 MHz
PSC = 0
ARR = 41
PWM frequency = 168 MHz / (41 + 1) = 4 MHz
CH1 CCR1 = 21
CH3 CCR3 = 31
```

CH1 输出采样时钟；CH3 不需要配置外部引脚，仅用比较事件产生 DMA 请求，使读取动作避开 AD9220 的采样边沿。

---

# 5. DMA 配置：TIM1_CH3 触发读取 GPIOC->IDR

## 5.1 推荐映射

根据 STM32F407 的 DMA2 请求映射，选择：

```text
TIM1_CH3 -> DMA2 Stream6 Channel6
```

理由：

- 现有 USART1_RX 使用 DMA2 Stream2 Channel4；
- 选择 Stream6 可以避免与 USART1_RX 冲突；
- TIM1_CH3 可在时钟边沿之后触发一次 GPIO 端口读取。

## 5.2 CubeMX 中的设置

CubeMX 通常不能直接把 `GPIOC->IDR` 作为“外设地址”图形化配置完整，因此分两步：

1. 在 TIM1 的 DMA Settings 中添加 `TIM1_CH3` DMA 请求；
2. CubeMX 生成后，在 `bsp_ad9220.c` 中启动前设置 DMA 源地址为 `&GPIOC->IDR`。

推荐 DMA 参数：

| 参数                  | 设置                               |
| --------------------- | ---------------------------------- |
| DMA                   | DMA2 Stream6 Channel6              |
| Direction             | Peripheral to Memory               |
| Peripheral increment  | Disable                            |
| Memory increment      | Enable                             |
| Peripheral data width | Word                               |
| Memory data width     | Word                               |
| Mode                  | Normal                             |
| Priority              | Very High                          |
| FIFO                  | Disable，先求简单                  |
| Interrupt             | Transfer Complete + Transfer Error |

原始缓存：

```c
#define ADC_RAW_MAX_COUNT 8192U

static uint32_t g_adc_raw[ADC_RAW_MAX_COUNT];
```

使用 32 位缓存是为了让 DMA 直接搬运整个 `GPIOC->IDR`。算法开始时再掩码低 12 位。

## 5.3 重要内存限制

DMA 缓冲区必须放在 SRAM1/SRAM2，不能放在 CCM RAM。

- `g_adc_raw[]`：普通 SRAM，DMA 可访问；
- FFT 工作区、窗函数、矩阵等仅由 CPU 使用的数据，可以放 CCM；
- 不确定链接脚本时，比赛期间先全部放普通 SRAM，确认稳定后再优化。

## 5.4 采集接口保持阻塞、简单

```c
bool BSP_AD9220_Capture(uint32_t *dst,
                        uint32_t count,
                        uint32_t timeout_ms);
```

内部步骤：

```text
停止 TIM1
清 DMA 标志
设置 DMA 源 = GPIOC->IDR
设置 DMA 目标与长度
使能 TIM1_CH3 DMA 请求
启动 DMA
启动 TIM1 PWM
等待完成标志或超时
停止 TIM1 和 DMA
返回成功/失败
```

DMA 中断中只做：

```c
g_adc_dma_done = true;
```

禁止在中断中进行 FFT、浮点运算、printf 或 TJC 发送。

---

# 6. CMSIS-DSP 配置

## 6.1 是否需要 DSP 库

需要。STM32F407 是 Cortex-M4F，使用 CMSIS-DSP 可以直接得到经过优化的实数 FFT、向量运算和统计函数。

建议只使用以下少量接口：

```c
#include "arm_math.h"

arm_rfft_fast_instance_f32 fft_inst;
arm_rfft_fast_init_4096_f32(&fft_inst);
arm_rfft_fast_f32(&fft_inst, fft_in, fft_out, 0);
```

可选：

```c
arm_mean_f32();
arm_rms_f32();
arm_max_f32();
arm_min_f32();
```

项目宏建议：

```text
ARM_MATH_CM4
__FPU_PRESENT=1
```

编译器必须启用硬件浮点：

```text
FPU: FPv4-SP-D16
Float ABI: Hard
```

## 6.2 FFT 缓冲建议

```c
#define FFT_SIZE 4096U

static float32_t fft_in[FFT_SIZE];
static float32_t fft_out[FFT_SIZE];
static float32_t spectrum[FFT_SIZE / 2U];
```

不要运行时 `malloc`。可以复用数组减少 SRAM：完成峰值搜索后，`spectrum` 与波形显示缓存可复用。

---

# 7. 推荐软件目录

```text
Core/
├─ Inc/
│  ├─ bsp_ad9220.h
│  ├─ alog_signal.h
│  └─ app_process.h
└─ Src/
   ├─ bsp_ad9220.c
   ├─ alog_signal.c
   └─ app_process.c
```

已有 UART1、UART3 处理继续放在 `app_process.c`，不再新增 `bsp_hmi`，避免比赛中为了分层而重复改动已验证代码。

## 7.1 BSP 层

`bsp_ad9220.c/h` 仅负责：

- TIM1 采样时钟启动/停止；
- DMA 一帧采集；
- 读取 OTR；
- 读取实际采样率配置；
- 丢弃流水线前若干点或提供有效数据起始位置。

推荐最少接口：

```c
void BSP_AD9220_Init(void);
bool BSP_AD9220_Capture(uint32_t *buffer,
                        uint32_t count,
                        uint32_t timeout_ms);
bool BSP_AD9220_IsOverrange(void);
uint32_t BSP_AD9220_GetRawSampleRate(void);
```

## 7.2 ALOG 层

`alog_signal.c/h` 只保留一个主入口：

```c
bool ALOG_Signal_Analyze(const uint32_t *raw,
                         uint32_t raw_count,
                         SignalResult_t *result);
```

内部按顺序执行：

1. 掩码、极性、零偏和电压标定；
2. 需要时 FIR 二抽一；
3. Hann 窗；
4. 4096 点 RFFT；
5. 谱峰、插值、基波/谐波关系；
6. 正弦最小二乘拟合幅值和相位；
7. 频响校正；
8. 真有效值、峰峰值和显示波形重构。

不要把每个数学步骤拆成独立文件。

## 7.3 APP 层

APP 只保留一个显示模式变量：

```c
typedef enum {
    VIEW_WAVE_1T = 0,
    VIEW_WAVE_3T,
    VIEW_SPECTRUM
} AppView_t;
```

主循环：

```c
void APP_Process_Run(void)
{
    APP_ParseUartAndTjcButtons();  // 保留现有逻辑

    if (!APP_TimeToRefresh()) {
        return;
    }

    if (BSP_AD9220_Capture(g_adc_raw, ADC_RAW_COUNT, 20U)) {
        ALOG_Signal_Analyze(g_adc_raw, ADC_RAW_COUNT, &g_result);
        APP_HMI_Update(&g_result, g_view);
    } else {
        APP_HMI_ShowError("ADC TIMEOUT");
    }
}
```

不使用多级状态机；只有显示模式、一次采集完成标志和错误标志。

---

# 8. 两阶段采样方案

## 8.1 第一阶段：必须先完成

```text
Fs = 2 MHz
Raw count = 4096
FFT count = 4096
频率间隔 = 488.28125 Hz
```

优点：

- 满足 500 Hz 频率分辨率；
- 单帧仅 2.048 ms；
- 4096 点 FFT 可直接使用 CMSIS-DSP；
- 软件和内存最简单。

## 8.2 第二阶段：用于抗干扰增强

```text
Raw Fs = 4 MHz
Raw count = 8192
63 阶左右线性相位 FIR 低通
二抽一
Effective Fs = 2 MHz
FFT count = 4096
```

目的：先在 4 MHz 原始数据中把 1 MHz 附近干扰表示出来，再在抽取前数字低通抑制，避免它在 2 MHz 直接采样时落在奈奎斯特边界附近。

注意：数字滤波不能替代模拟抗混叠滤波器。高于 2 MHz 的输入分量仍可能在 4 MSPS 下混叠，因此硬件前端必须有低通。

建议使用编译期开关，而不是运行中动态切换复杂状态：

```c
#define ADC_USE_4M_DECIMATE2  0
```

---

# 9. AD9220 模块必须先做的硬件确认

上传模块默认参数为：

- 5 V 供电；
- 12 位并行输出、3.3 V 控制电平；
- 50 Ω 输入；
- 模块默认量程 10 Vpp；
- 前端 π 型网络默认衰减 5 倍；
- AD8132 完成单端转差分和共模偏置。

本题只有 50~250 mVpp，直接使用 10 Vpp 量程会严重浪费 ADC 码数：

```text
50 mVpp 约占 20.5 codes
250 mVpp 约占 102.4 codes
```

不建议直接使用默认量程。

推荐最终硬件路线：

1. 把模块 π 型衰减改为 1 倍参考值：`R6=R7≈151 Ω，R8+R9≈37 Ω`；
2. 自制 BNC 前端完成 49.9 Ω 端接、保护、约 6 倍固定增益和 600~700 kHz 低通；
3. 前端输出送入已修改的 AD9220 模块。

这样输入端等效满量程约为：

```text
2 Vpp / 6 ≈ 333 mVpp
```

大致码数：

```text
50 mVpp  -> 约 614 codes
250 mVpp -> 约 3072 codes
```

ADC 输入折算到被测端的理论量化步距约 0.081 mV，明显更有利于达到 5 mV 幅值误差指标。

改阻值后先用 10 kHz 小信号验证：

- 0 V 输入时中点稳定；
- 正弦输入不触发 OTR；
- 50 mVpp、250 mVpp 都能正常显示；
- 输入频率升到 500 kHz 后幅值衰减可重复，后续用标定表补偿。

---

# 10. TJC8048X270 UI 设计

## 10.1 页面原则

使用一个主页面，不要为了 1 周期、3 周期和频谱创建三个完全独立页面。

推荐 800×480 横屏布局：

```text
┌──────────────────────────────────────────────────────────────┐
│ G题 周期信号分析     RUN     Fs:2.000M     OTR:OK            │
├──────────────────────────────────────┬───────────────────────┤
│                                      │ Upp      182.6 mV      │
│                                      │ Urms      66.2 mV      │
│       波形 / 正频率轴频谱图区         │ f0       50.01 kHz      │
│            约 560×300                │                       │
│                                      │ 1: 50.01k  72.0mV     │
│                                      │ 3:150.02k  18.1mV     │
│                                      │ 4:200.03k   9.0mV     │
├──────────────────────────────────────┴───────────────────────┤
│ [1周期]      [3周期]      [频谱]       状态: MEASURE OK      │
└──────────────────────────────────────────────────────────────┘
```

## 10.2 推荐控件

| 类型        | 建议名称    | 用途                                |
| ----------- | ----------- | ----------------------------------- |
| Text        | `t_title`   | 标题                                |
| Text        | `t_status`  | OK、OTR、TIMEOUT、NO SIGNAL         |
| Text/Number | `n_upp`     | 峰峰值，建议 MCU 先转成 0.1 mV 整数 |
| Text/Number | `n_urms`    | 真有效值                            |
| Text/Number | `n_f0`      | 基频                                |
| Text        | `t_comp1~3` | 三个频率分量的阶次、频率、幅值      |
| Waveform    | `s_wave`    | 波形或频谱显示                      |
| Button      | `b_wave1`   | 发送 1 周期事件                     |
| Button      | `b_wave3`   | 发送 3 周期事件                     |
| Button      | `b_spec`    | 发送频谱事件                        |

屏幕按钮事件使用固定单字节或短帧，例如：

```text
0xA1 = 1 周期
0xA3 = 3 周期
0xAF = 频谱
```

继续由现有 `app_process` 的 USART3 接收逻辑解析，不再新建按键驱动层。

## 10.3 刷新频率

- 数值：5~10 Hz；
- 曲线：4~5 Hz；
- 每帧只发送 400~600 个显示点；
- 不向屏幕发送全部 4096 个采样点；
- 只有结果或模式改变时更新对应控件。

建议先保持 USART3 115200 波特率。若 600 点曲线刷新不足，再在屏幕与 MCU 同时改为 230400；不要在算法未通前先改串口链路。

## 10.4 显示数据生成

波形不要直接抽取原始 ADC 点。应由 ALOG 根据检测到的幅值、频率和相位重构：

```text
1 周期：横轴覆盖 T0
3 周期：横轴覆盖 3T0
```

这样在 500 kHz、2 MSPS 仅有 4 点/周期时，屏幕仍可得到平滑且相位正确的波形。

频谱图只需要正频率轴上的离散谱线。题目最多 3 个有效分量，因此可以：

- 用波形控件画 0~500 kHz 的简化谱线；或
- 直接画 3 根竖线并在右侧列表显示精确值。

第二种更容易控制视觉效果，且符合“定性显示频谱”的评分目标。

---

# 11. 建议的调试日志

USART1 日志保持简短：

```text
[ADC] fs=2000000 n=4096 min=1812 max=2276 otr=0
[FFT] f0=50012.3Hz comp=3 time=7.4ms
[RES] upp=182.6mV rms=66.2mV
[HMI] view=SPEC tx=486B
```

禁止每帧打印 4096 个 ADC 码。需要看原始数据时，只在调试命令触发后输出前 32 或 128 点。

---

# 12. 最短验证顺序

1. PA8 输出 2 MHz 方波，用示波器确认频率和占空比；
2. 关闭 DMA，只手动读取 GPIOC，确认 D0~D11 接线；
3. DMA 采集 4096 点，USART1 输出 min/max/mean 和前 32 点；
4. 输入 10 kHz 单正弦，确认码值极性和波形；
5. 输入 100 kHz，完成 4096 点 FFT 和频率显示；
6. 输入 500 kHz，验证频率和最小信号幅值；
7. 加入 Hann 窗、插值和幅值拟合；
8. 完成 TJC 三个按钮及单页 UI；
9. 最后加入模拟低通、4 MSPS + 二抽一和抗 1 MHz 干扰。

不要并行开发复杂 UI、FFT、滤波和 DMA。每完成一段链路，先留下可重复的串口验证日志。

---

# 13. 配置检查表

- [ ] HSE + PLL，SYSCLK 168 MHz；
- [ ] PC0~PC11 连续接 D0~D11；
- [ ] PC12 读取 OTR；
- [ ] PA8 TIM1_CH1 输出 ADC CLK；
- [ ] TIM1_CH3 产生 DMA 请求；
- [ ] DMA2 Stream6 Channel6，源地址 GPIOC->IDR；
- [ ] DMA 缓冲不放 CCM；
- [ ] USART1/3 和 app_process 保持原样；
- [ ] CMSIS-DSP 4096 点 RFFT 可链接；
- [ ] 编译器启用 Cortex-M4F 硬件浮点；
- [ ] 基础模式 2 MSPS + 4096 点先通过；
- [ ] TJC 单页、三个按钮、每帧 400~600 显示点；
- [ ] AD9220 模块默认 5 倍衰减已处理，不能直接按 10 Vpp 量程测 50 mVpp；
- [ ] 5 V 电源分屏幕、数字、模拟三支路并共地；
- [ ] 完成 10~500 kHz 多频点幅值校准。

## 参考资料

- 2026 电赛 G 题《周期信号测量分析装置》；
- 凌智电子《AD9220 模数转换器模块用户手册 V1.1》；
- 凌智电子《AD9220 原理图 V1.1》；
- 上传的 STM32F103/H750 标准库测试程序，仅用于接口、极性和时序参考；
- STM32F407 参考手册 RM0090；
- CMSIS-DSP Real FFT 文档；
- 仓库：`Jacky-zhengH/TI_2026_G_Signal`。


