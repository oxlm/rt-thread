# FRDM-MCXN947 USB ADC Microphone

## 功能目标

本应用目标是在 RT-Thread 的 `frdm-mcxn947` BSP 上实现一个 USB 麦克风设备：

- USB 设备类型：USB Audio Class 2.0 microphone。
- 音频格式：3 通道、16 kHz 采样率、16 bit 位深、PCM。
- 采样来源：3 路模拟麦克风信号，经 MCXN947 内部 OPAMP/PGA 后进入 LPADC。
- 采样时钟：使用硬件定时器触发 LPADC，避免软件定时带来的采样抖动。
- 数据搬运：LPADC FIFO 通过 EDMA 搬运到内存，应用层将 3 路 ADC 数据交织成 USB mic PCM 帧。
- 启停策略：主机打开 mic 的 AudioStreaming alternate setting 时开始 ADC/DMA/CTIMER 采样；主机关闭、USB reset、disconnect、suspend 时停止采样。

当前设计只暴露固定 16 kHz 采样率，不做多采样率切换。

## 硬件结构

### 模拟输入

当前按 3 路单端麦克风输入设计：

| 逻辑通道 | 麦克风信号输入 | OPAMP 负输入 | OPAMP 输出 | 内部模拟链路 |
| --- | --- | --- | --- | --- |
| MIC0 | `P4_12` / `OPAMP0_INP0` | `P4_14` / `OPAMP0_INN` 接地 | `P4_15` / `OPAMP0_OUT` 不外接 | OPAMP0 PGA -> LPADC |
| MIC1 | `P4_16` / `OPAMP1_INP0` | `P4_18` / `OPAMP1_INN` 接地 | `P4_19` / `OPAMP1_OUT` 不外接 | OPAMP1 PGA -> LPADC |
| MIC2 | `P4_20` / `OPAMP2_INP0` | `P4_22` / `OPAMP2_INN` 接地 | `P4_23` / `OPAMP2_OUT` 不外接 | OPAMP2 PGA -> LPADC |

推荐接法：

- 麦克风单端信号经隔直电容接到对应 `INP0`。
- 麦克风偏置由外部电路提供。
- OPAMP 的 `INN` 接模拟地或麦克风信号参考地。
- OPAMP 的 `OUT` 焊盘不外接 ADC，也不接外部反馈电阻，按内部 PGA 模式使用。
- OPAMP 输出通过芯片内部模拟通路送入 LPADC，不依赖外部 `OUT` 引脚回接 ADC。

### ADC 与 DMA

当前采样分配如下：

| 逻辑通道 | ADC 资源 | FIFO | DMA request |
| --- | --- | --- | --- |
| MIC0 | ADC0 side A | ADC0 FIFO0 | `kDma0RequestMuxAdc0FifoARequest` |
| MIC1 | ADC0 side B | ADC0 FIFO1 | `kDma0RequestMuxAdc0FifoBRequest` |
| MIC2 | ADC1 side A | ADC1 FIFO0 | `kDma0RequestMuxAdc1FifoARequest` |

ADC0 使用 dual single-ended A/B 模式，一次硬件触发采两路；ADC1 使用 side A 单端模式。CTIMER 同时触发 ADC0 和 ADC1，因此目标效果是 ADC0 的两路和 ADC1 的一路在同一采样节拍下连续采样。

DMA 使用 DMA0 的 3 个通道：

- DMA channel 0：ADC0 FIFO A。
- DMA channel 1：ADC0 FIFO B。
- DMA channel 2：ADC1 FIFO A。

每路 DMA 使用 8 个 block 的 loop transfer。每个 block 为 16 个采样点，对应 16 kHz 下 1 ms 的采样数据。

### 采样时钟

当前使用：

- `CTIMER2 MAT3` 作为硬件采样节拍源。
- `INPUTMUX` 将 `CTIMER2 MAT3` 同时连接到 ADC0 trigger 和 ADC1 trigger。
- CTIMER2 时钟源配置为 `FRO_HF`，分频为 1。
- `CTIMER2 MAT3` 每次 match 翻转输出；LPADC 硬件触发按边沿触发处理，所以代码中将 match rate 设置为 `16 kHz * 2`，使有效上升沿触发约为 16 kHz。

需要在实机上用示波器或调试计数确认最终触发频率。

## USB 结构

USB 使用 CherryUSB device stack，当前为 UAC2 mic-only 设备：

- AudioControl interface：interface 0。
- AudioStreaming interface：interface 1。
- IN endpoint：`0x81`。
- 标称 USB 帧数据：`16 samples/ms * 3 channels * 2 bytes = 96 bytes/ms`。
- 等时端点 `wMaxPacketSize`：`102 bytes`，即标称 96 bytes 加 1 个 audio slot 的余量。

UAC2 拓扑为：

```text
Input Terminal (Microphone, id 0x02)
    -> Feature Unit (id 0x03)
    -> Output Terminal (USB Streaming, id 0x04)
    -> AudioStreaming interface bTerminalLink = 0x04
    -> IN endpoint 0x81
```

这里的 `Output Terminal (USB Streaming)` 是 UAC 拓扑中音频功能的输出端，表示音频数据从设备流向 USB Host；它不是扬声器播放节点。

Clock Source 当前按固定 16 kHz 处理：

- Clock Source 属性：internal fixed clock。
- Sampling Frequency Control：read-only。
- `GET_RANGE` 只返回一个 discrete range：min = max = 16000 Hz，resolution = 0。

## 软件入口

相关文件：

- `applications/usb_adc_mic.c`：USB mic 描述符、ADC/OPAMP/CTIMER/DMA 配置、PCM 缓冲和 USB 发送逻辑。
- `applications/SConscript`：通过 `Glob('*.c')` 自动编译 `usb_adc_mic.c`。
- `board/cherryusb_port.c`：在 `RT_CHERRYUSB_DEVICE_TEMPLATE_NONE && RT_CHERRYUSB_DEVICE_AUDIO` 条件下调用 `usb_adc_mic_init(0, USBHS1__USBC_BASE)`。
- `.config` / `rtconfig.h`：当前配置为 CherryUSB device、audio class、template none。
- `board/MCUX_Config/board/pin_mux.c`：已将 `P4_12`、`P4_16`、`P4_20` 配成模拟输入用途。

运行时主要流程：

1. BSP 初始化时调用 `usb_adc_mic_init()`。
2. 初始化 OPAMP、LPADC、INPUTMUX、CTIMER 和 EDMA。
3. 注册 UAC2 描述符和 CherryUSB audio interface。
4. 主机选择 AudioStreaming interface 1 的 alt setting 1 时，`usbd_audio_open()` 启动采样。
5. EDMA 每完成 1 ms block 后回调，等待 3 路数据都到齐。
6. 应用将 3 路原始 ADC 数据转成 signed 16-bit PCM，并按通道交织写入 PCM ring。
7. USB IN endpoint 空闲时，从 PCM ring 取 96 bytes 发送给主机。
8. ring 空时发送静音帧；ring 满时丢弃最旧帧并记录 overrun。

## 当前完成程度

已经完成：

- 新增 `applications/usb_adc_mic.c`，实现 UAC2 3 通道 USB mic 的基本描述符和数据路径。
- 实现主机打开/关闭 mic 时启停 ADC/DMA/CTIMER。
- 实现 CTIMER2 触发 ADC0/ADC1 的硬件采样框架。
- 实现 ADC0 A/B 与 ADC1 A 三路 FIFO DMA 采样框架。
- 实现 3 路 ADC 数据到 3 通道 signed 16-bit PCM 的交织。
- 实现 USB IN endpoint 发送、USB underrun 静音帧、PCM ring overrun 丢旧帧。
- 将 CherryUSB device template 切到 `TEMPLATE_NONE`，避免继续使用原始 mic + speaker 模板。
- 已按 Windows UAC2 兼容性调整描述符中的固定 Clock Source 和 IN endpoint `wMaxPacketSize`。
- 已将 `P4_12`、`P4_16`、`P4_20` 配置为 OPAMP 输入相关的模拟功能。
- 已做 `arm-none-eabi-gcc -fsyntax-only` 语法检查。

尚未完成或尚需确认：

- 尚未完成实机 Windows 枚举复测；前一版曾出现 Code 10，当前描述符已调整但需要重新插拔验证。
- 尚未运行完整 BSP `scons` 构建；当前环境中 `scons` 不在 PATH。
- `OPAMPx_INN` 接地对应的 pin mux 仍需确认；若使用 `P4_14`、`P4_18`、`P4_22`，建议同步配置为模拟输入或禁用数字功能。
- `USB_ADC_MIC_ADC0_A_CHANNEL`、`USB_ADC_MIC_ADC0_B_CHANNEL`、`USB_ADC_MIC_ADC1_A_CHANNEL` 当前仍是默认值，需要结合 MCXN947 Reference Manual 或 MCUXpresso Config Tools 确认 OPAMP 输出到 LPADC 的内部通道号。
- OPAMP 增益当前默认为 `kOPAMP_PosGainNonInvert1X`，实际麦克风幅度可能需要重新选择 PGA 增益。
- 需要实测 CTIMER2 触发频率是否稳定为 16 kHz。
- 需要实测 ADC 原始值范围、直流偏置、削波情况和噪声底。
- USB Feature Unit 目前声明了 mute/volume 控制，但尚未接入实际模拟增益或数字音量处理。

## 后续验证建议

1. 重新生成配置并完整构建：

```powershell
scons --pyconfig-silent
scons -j
```

2. Windows 端删除旧设备缓存或修改 USB serial 后重新插拔，确认设备管理器不再出现 Code 10。

3. 用 USBView、USBPcap 或 Wireshark 检查：

- UAC2 descriptor 是否完整。
- Clock Source 的 `CUR/RANGE` 请求是否返回 16000 Hz。
- AS interface 是否能切到 alt setting 1。
- IN endpoint `wMaxPacketSize` 是否为 102 bytes。

4. 上板确认硬件采样链路：

- CTIMER2 MAT3 触发频率。
- ADC0 FIFO0/FIFO1 和 ADC1 FIFO0 DMA 是否持续完成。
- 3 路 PCM 数据是否随输入信号变化。
- 长时间打开 mic 是否出现 overrun/underrun。
