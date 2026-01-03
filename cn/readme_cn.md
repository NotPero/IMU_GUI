# IMU SYSTEM PRO (v3.3) 开发者指引

本系统是一个基于 Julia 语言开发的高性能 IMU 实时监控与数据采集终端。它专为高频串口数据设计，兼顾了 UI 的响应速度与硬件原始数据的完整性。

---

## 1. 总体架构说明

本系统采用 **“异步生产者-消费者”** 架构，利用 Julia 的原生协程（Task）实现界面与数据流的完全解耦。



### 核心分层设计：
1.  **静态配置层 (`CONFIG`)**：统管全局参数。UI 尺寸、保存路径、采样频率均在此定义，修改此处即可实现布局重排。
2.  **状态管理层 (`AppState`)**：存储实时运行状态（如校准偏置、录制开关、运行时间）。它是 UI 与后台线程通信的唯一窗口。
3.  **异步解析层 (`@async` 线程)**：这是程序的“心脏”。它持续监听串口，负责原始字节流的校验、协议解析、数据保存以及向 UI 推送刷新信号。
4.  **渲染展示层 (`GLMakie` 渲染器)**：基于 GPU 加速的绘图引擎，负责将处理后的数据实时绘制成动态波形。

---

## 2. 开发者对照表 (代码 vs 功能)

| 架构模块 | 源码位置 (核心变量/代码块) | 功能描述 |
| :--- | :--- | :--- |
| **布局参数** | `const CONFIG` | 调整 `block.ratios` 数组即可改变各绘图窗口的高度占比。 |
| **状态追踪** | `mutable struct AppState` | 增加全局状态（如“丢包率”）需在此结构体定义。 |
| **绘图单元** | `create_sensor_block!` | 封装了带数值显示的波形窗口。若需增加新传感器，直接调用此工厂函数。 |
| **协议解析** | `while length(raw_buffer) >= 52` | 硬件协议解包逻辑。如修改帧长或数据偏移量，在此处调整。 |
| **扩展沙盒** | `g_extension` | 预留的 UI 槽位。开发者可在此处通过一行代码放置按钮或控制面板。 |

---

## 3. 性能优化策略：确保绘图流畅与高速保存

在高频采样（如 1000Hz）场景下，系统通过以下技术保障运行稳健：

### 3.1 绘图流畅性 (GPU Acceleration)
* **循环缓冲区 (Circular Buffer)**：
    ```julia
    obs_acc[i].val = circshift(obs_acc[i].val, -1)
    obs_acc[i].val[end] = new_data
    ```
    通过 `circshift` 操作，代码避免了频繁的内存申请与释放，极大降低了垃圾回收（GC）引起的卡顿。
* **UI 刷新节流 (Throttling)**：
    系统通过 `CONFIG.data.refresh` 控制刷新频率。即使硬件每秒发送上千帧，UI 渲染仍维持在稳定的 20Hz，确保视觉平滑且不榨干显卡性能。



### 3.2 高速数据保存 (Zero-Lag Persistence)
* **无损二进制转换**：
    ```julia
    fs = reinterpret(Float32, frame[13:48])
    ```
    使用 `reinterpret` 直接将字节映射为浮点数，相比传统的字符串解析，其速度快了数个数量级。
* **非阻塞写入**：
    数据解析与 `printf` 写入同步在异步线程中进行，绝不干扰主界面的点击与交互。
* **自动路径管理**：
    系统通过 `mkpath` 自动确保保存文件夹的存在，避免了因路径缺失导致的程序崩溃。

### 3.3 硬件通讯优化 (Linux Serial Tuning)
针对 921600 bps 的高频数据流，系统依赖以下系统级优化：
* **Raw 模式配置**：通过 `stty` 将串口设为 `raw` 模式，禁用回显和特殊字符处理，确保二进制数据包（含 `0x0A` 等）不被内核拦截。
    ```
    sudo stty -F /dev/ttyUSB0 921600 raw -echo -echoe -echok
    ```
* **低延迟模式 (Low Latency)**：通过 `setserial` 开启 `low_latency` 标志。这会强制驱动程序在收到字节后立即上报给 Julia，而非等待内核缓冲区填满，彻底解决数据“成堆出现”的卡顿问题。
    ```
    sudo setserial /dev/ttyUSB0 low_latency
    ```
---

## 4. 扩展指引：如何增加自定义功能

1.  **增加一个分析按钮**：
    在 `g_extension` 块下添加：
    ```julia
    btn_ana = Button(g_extension[1, 1], label = "Run FFT")
    on(btn_ana.clicks) do _
        # 你的分析代码
    end
    ```
2.  **修改保存逻辑**：
    定位到 `on(btn_record.clicks)`，修改 `fname` 的生成规则或 `println` 的列标题。
3.  **调整轴显示范围**：
    在 `create_sensor_block!` 中通过 `ylims!(ax, min, max)` 设定固定量程。

---

## 5. 维护者提示
* **安全关闭**：请务必点击 `Stop` 按钮结束录制，以确保磁盘缓冲区的数据完全刷入 CSV 文件。
* **校准要求**：点击 `Calibrate` 时请确保 IMU 处于水平静止状态。