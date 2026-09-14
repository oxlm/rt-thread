# MCXN947 DAC0 到三路 ADC 回环测试说明

## 1. 目标

工程同时采样以下三路模拟输入，并判断每一路是否处于 DAC0 输出附近：

```text
ADC0_A2 + ADC1_A0 + ADC1_B0
```

判定是逐路独立进行的，因此三路可以同时被判定为接到了 DAC0。程序不会在三路中选择“最接近的一路”，而是打印当前所有满足条件的路。只有这个集合发生变化时才打印一次。

## 2. 硬件接线

### 2.1 信号位置

| 信号 | MCU 引脚 | FRDM-MCXN947 位置 |
|---|---|---|
| DAC0_OUT | P4_2 | J1[4] |
| ADC0_A2 | P4_23 | J8[28] |
| ADC1_A0 | ANA_4 / T2 | J6[1] |
| ADC1_B0 | ANA_5 / T3 | J2[1] |

`ADC0_A2` 和 `ADC0_B2` 都位于 `P4_23/J8[28]`，不能作为两个独立外部输入。本工程采用 `ADC1_A0` 和 `ADC1_B0` 作为另外两路独立输入。

### 2.2 回环接线

将 `J1[4]/P4_2/DAC0_OUT` 通过跳线接到需要验证的 ADC 输入：

```text
DAC0_OUT -> ADC0_A2   : J1[4] -> J8[28]
DAC0_OUT -> ADC1_A0   : J1[4] -> J6[1]
DAC0_OUT -> ADC1_B0   : J1[4] -> J2[1]
```

为了识别具体接通的是哪一路，建议一次只接一条跳线。若同时将 DAC0_OUT 扇出接到多路 ADC 输入，程序会同时打印所有被判定为接通的路，例如：

```text
DAC0 connected via: ADC0_A2, ADC1_A0
```

`J2[1]` 在板上标注为 `MC_BEMF_A`，接入外部信号前应确认该节点没有其他板载输出源驱动。不要把两个推挽输出直接短接。

外部模拟源、示波器和开发板必须共地。ADC 输入不得为负电压，也不得超过当前芯片模拟输入和参考电压允许范围。DAC0_OUT 不能同时连接到外部电压源。

## 3. 外设和 DMA 分工

### 3.1 统一采样触发

```text
FRO12M -> CTIMER0 -> match 3
                    ├── INPUTMUX -> ADC0 trigger 0
                    └── INPUTMUX -> ADC1 trigger 0
```

采样率为 16 kHz。ADC0 和 ADC1 使用同一个 CTIMER0 match 3 事件。

### 3.2 ADC0

```text
ADC0 command 1
  single-ended side A
  channel 2 = ADC0_A2
  FIFO A/0 -> DMA0 CH0 -> adc0A2Samples
```

`ADC0_A2` 的物理位置为 `P4_23/J8[28]`。

### 3.3 ADC1

ADC1 command 1 使用双 side 单端模式：

```text
ADC1 command 1
  channel A = 0 -> ADC1_A0 -> FIFO A/0 -> DMA0 CH3 -> adc1A0Samples
  channel B = 0 -> ADC1_B0 -> FIFO B/1 -> DMA0 CH4 -> adc1B0Samples
```

每次 ADC1 硬件触发都会产生 A、B 两个 FIFO 结果。ADC1 的 A/B FIFO 分别由独立 DMA 通道搬运。

串口控制台已经使用 `DMA0 CH1` 和 `DMA0 CH2`，因此 ADC1 两路使用 `DMA0 CH3` 和 `DMA0 CH4`，避免覆盖串口 DMA。

### 3.4 同步 block

三路 DMA 都使用双缓冲，每路各有两个 160-sample block：

```text
adc0A2Samples[2][160]
adc1A0Samples[2][160]
adc1B0Samples[2][160]
```

某一半块或整块只有在 DMA0 CH0、CH3、CH4 全部完成后，才会发布为一个同步 block。主循环取得 block 后：

1. 分别统计三路 ADC 的 16-bit 结果。
2. 判断三路是否接近 DAC0 输出码。
3. 将 `ADC0_A2` 转换为 PCM16，继续送入原有音频后端。

## 4. DAC0 近似判断

DAC0 初始化后固定输出：

```c
#define DAC_ADC_TEST_OUTPUT_CODE (2048U)
```

DAC0/LPDAC 的硬件输出数据字段仍为 12-bit，因此 `2048` 仍是 DAC 的中点输出码。ADC 命令切换为单端高分辨率模式后，ADC FIFO 原始结果转换为无符号 16-bit 数值的方式为：

```c
unsigned16 = (rawSample & ADC_RESFIFO_D_MASK);
```

DAC 的 12-bit 中点换算到 ADC 的 16-bit 量程后，判定目标值为：

```text
2048 * 65536 / 4096 = 32768
```

每个 160-sample block 分别计算三路 16-bit ADC 结果的平均值。前两个同步 block 用于等待 DAC 和 ADC 稳定，不参与连接判定。

判定阈值：

```text
首次进入“接近”状态：|average - 32768| <= 8192
保持已接通状态：      |average - 32768| <= 10240
```

释放阈值略宽于进入阈值，用于避免输入噪声在边界附近导致反复打印。三路使用独立状态和独立阈值判断。

连接状态 bit 定义：

| bit | 宏 | 路径 |
|---:|---|---|
| 0 | `DAC_ADC_TEST_CONNECTED_ADC0_A2_MASK` | `ADC0_A2` |
| 1 | `DAC_ADC_TEST_CONNECTED_ADC1_A0_MASK` | `ADC1_A0` |
| 2 | `DAC_ADC_TEST_CONNECTED_ADC1_B0_MASK` | `ADC1_B0` |

串口输出示例：

```text
DAC0 connected via: ADC0_A2
DAC0 connected via: ADC1_A0, ADC1_B0
DAC0 connected via: none (floating/no match)
```

如果某一路悬空但其平均值偶然落入阈值，软件只能根据采样电平将其判为接近 DAC0。要提高物理识别可靠性，应让未连接输入有确定偏置或使用合适的下拉/前端电路，而不是依赖悬空节点。

## 5. 主要观测变量

| 变量 | 含义 |
|---|---|
| `g_outputCode` | 当前 DAC 输出码，应为 `2048` |
| `g_blockCount` | 已处理的同步 block 数 |
| `g_lastSequence` | 最近同步 block 序号 |
| `g_adc0A2Stats.average` | 最近 block 的 ADC0_A2 平均值 |
| `g_adc1A0Stats.average` | 最近 block 的 ADC1_A0 平均值 |
| `g_adc1B0Stats.average` | 最近 block 的 ADC1_B0 平均值 |
| `g_connectedMask` | 当前近似接通路的 bit mask |
| `g_stateChangeCount` | 已成功打印的状态变化次数 |
| `g_matchedBlockCount` | 至少一路接近 DAC0 的 block 数 |
| `g_unmatchedBlockCount` | 没有一路接近 DAC0 的 block 数 |
| `g_backendBlockCount` | PCM 后端已处理的 block 数 |
| `g_backendOverrunCount` | 采样 block 未及时取走的次数 |

三路分别通过 `g_adc0A2Stats`、`g_adc1A0Stats` 和 `g_adc1B0Stats` 保留最近 block 的 first/min/max/average，便于观察噪声和悬空状态。

## 6. 构建和验证

在项目根目录执行：

```powershell
cmake --build --preset debug
```

构建产物：

```text
debug/frdm_mcxn947_blank_cm33_core0.elf
```

在线调试至少检查：

- 能停在 `main()`。
- 能经过 DAC0、CTIMER0、ADC0、ADC1 和 DMA 初始化。
- `EDMA_0_CH0_IRQHandler()`、`EDMA_0_CH3_IRQHandler()`、`EDMA_0_CH4_IRQHandler()` 均会进入。
- `g_blockCount`、`g_backendBlockCount` 持续增加。
- `g_backendOverrunCount` 和 ADC DMA error 计数保持为 0。
- 不命中 `HardFault_Handler` 或 `DefaultISR`。
- ADC 三路统计平均值接近 16-bit 中点 `32768`。
- 单独接某一路时，串口输出的集合只包含该路；同时接多路时，输出包含全部命中的路。

## 7. 本次代码变更

| 文件 | 变更 |
|---|---|
| `drivers/adc_audio_capture.h` | 增加三路同步 block 接口 |
| `drivers/adc_audio_capture.c` | 配置 ADC0/ADC1、双 FIFO、三路 DMA 和同步 block 汇合 |
| `application/dac_adc_test.h` | 增加三路统计变量和连接状态 bit 定义 |
| `application/dac_adc_test.c` | 实现三路独立 DAC0 近似判断及变化时打印 |
| `application/led_blinky.c` | 消费三路同步 block，保持 ADC0_A2 的 PCM 后端路径 |
| `Docs/DAC_ADC_LOOPBACK.md` | 更新三路接线、DMA、判定和调试说明 |

`drivers/dac.c`、串口 DMA 通道、生成的 pin mux 和 MCUXpresso SDK junction 未修改。
