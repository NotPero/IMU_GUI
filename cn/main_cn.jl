using LibSerialPort, Printf, Dates, GLMakie, LinearAlgebra

# ==========================================================
# 1. 深度参数化配置中心 (在此处修改保存路径和 UI 尺寸)
# ==========================================================
const CONFIG = (
    window = (
        size       = (1650, 1000),
        title      = "IMU Monitor",  # Window title text
        bg_color   = :white
    ),
    toolbar = (
        height     = 70,
        timer_w    = 180,    # 运行时间模块宽
        btn_w      = 120,    # 按钮宽
        btn_h      = 38,     # 按钮高
        status_w   = 280,    # 状态提示宽 (略宽以显示路径)
        font_size  = 20      
    ),
    block = (
        header_h   = 45,     
        title_w    = 180,    
        label_w    = 35,     
        value_w    = 95,     
        unit_w     = 60,     
        plot_gap   = 15,     
        ratios     = [0.26, 0.26, 0.26, 0.22] # 纵向比例 (Acc, Gyr, Eul, Ext)
    ),
    axis = (
        left = 8, right = 20, bottom = 12, top = 8
    ),
    data = (
        history  = 600,       
        refresh  = 0.05,
        # 【重要】指定保存文件夹，支持相对路径或绝对路径
        save_dir = "./imu_logs" 
    )
)

# ==========================================================
# 2. 状态管理
# ==========================================================
mutable struct AppState
    is_calibrating::Observable{Bool}
    is_recording::Observable{Bool}
    run_time_str::Observable{String}
    gyro_offset::Vec3f
    calib_buffer::Vector{Vec3f}
    file_io::Union{IOStream, Nothing}
    
    AppState() = new(Observable(false), Observable(false), Observable("0.0 s"), Vec3f(0), Vec3f[], nothing)
end

# ==========================================================
# 3. UI 构建器
# ==========================================================
function create_sensor_block!(pos, title, unit, colors)
    block = pos[1, 1] = GridLayout()
    header = block[1, 1] = GridLayout(halign = :left, height = CONFIG.block.header_h)
    
    Label(header[1, 1], title, font = :bold, fontsize = 19, width = CONFIG.block.title_w, halign = :left)
    
    v_obs = [Observable("0.00") for _ in 1:3]
    labels = (title == "Euler Angles") ? ["R:", "P:", "Y:"] : ["X:", "Y:", "Z:"]
    
    for i in 1:3
        Label(header[1, i*2], labels[i], color = :grey40, width = CONFIG.block.label_w, halign = :right)
        Label(header[1, i*2+1], v_obs[i], color = colors[i], width = CONFIG.block.value_w, halign = :left)
    end
    Label(header[1, 8], unit, color = :grey60, width = CONFIG.block.unit_w, halign = :left)

    ax = Axis(block[2, 1], xgridvisible = true, ygridvisible = true)
    ax.alignmode = Mixed(left = CONFIG.axis.left, right = CONFIG.axis.right, 
                         bottom = CONFIG.axis.bottom, top = CONFIG.axis.top)
    return ax, v_obs
end

# ==========================================================
# 4. 主程序逻辑
# ==========================================================
function start_app()
    state = AppState()
    fig = Figure(size = CONFIG.window.size, backgroundcolor = CONFIG.window.bg_color)
    
    g_top   = fig[1, 1] = GridLayout(height = CONFIG.toolbar.height)
    g_main  = fig[2, 1] = GridLayout()
    rowsize!(fig.layout, 1, Fixed(CONFIG.toolbar.height))

    # --- 工具栏 ---
    t_box = g_top[1, 1] = GridLayout(width = CONFIG.toolbar.timer_w)
    Label(t_box[1, 1], "Uptime:", color = :grey30, fontsize = 15)
    Label(t_box[1, 2], state.run_time_str, font = :bold, fontsize = 20, width = 85)

    btn_calib  = Button(g_top[1, 2], label = "Calibrate", width = CONFIG.toolbar.btn_w, height = CONFIG.toolbar.btn_h)
    btn_record = Button(g_top[1, 3], label = "Record All", width = CONFIG.toolbar.btn_w, height = CONFIG.toolbar.btn_h)
    lbl_status = Label(g_top[1, 4], "Ready", width = CONFIG.toolbar.status_w, halign = :left, color = :blue)
    
    Label(g_top[1, 5], CONFIG.window.title, font = :bold, fontsize = CONFIG.toolbar.font_size, halign = :right)
    colsize!(g_top, 5, Relative(1))

    # --- 传感器 Block 实例化 ---
    colors = [:red, :forestgreen, :dodgerblue]
    ax_acc, v_acc = create_sensor_block!(g_main[1, 1], "Accelerometer", "g", colors)
    ax_gyr, v_gyr = create_sensor_block!(g_main[2, 1], "Gyroscope", "d/s", colors)
    ax_eul, v_eul = create_sensor_block!(g_main[3, 1], "Euler Angles", "deg", colors)

    # --- 扩展功能引导区 (Slot) ---
    g_extension = g_main[4, 1] = GridLayout()
    # 示例扩展：在这里你可以添加自定义按键或图表
    btn_task = Button(g_extension[1, 1], label = "Custom Task", width = 140, height = 40)
    Label(g_extension[1, 2], "Files saved to: $(abspath(CONFIG.data.save_dir))", color = :grey60, halign = :left)
    
    on(btn_task.clicks) do _
        println("Extension button clicked at $(state.run_time_str[])")
    end

    # 应用网格比例
    for i in 1:4; rowsize!(g_main, i, Relative(CONFIG.block.ratios[i])); end
    rowgap!(g_main, CONFIG.block.plot_gap)

    # --- 数据观察者与线条 ---
    obs_acc = [Observable(fill(0.0f0, CONFIG.data.history)) for _ in 1:3]
    obs_gyr = [Observable(fill(0.0f0, CONFIG.data.history)) for _ in 1:3]
    obs_eul = [Observable(fill(0.0f0, CONFIG.data.history)) for _ in 1:3]
    for i in 1:3
        lines!(ax_acc, obs_acc[i], color = colors[i], linewidth = 1.5)
        lines!(ax_gyr, obs_gyr[i], color = colors[i], linewidth = 1.5)
        lines!(ax_eul, obs_eul[i], color = colors[i], linewidth = 1.5)
    end

    # --- 交互事件逻辑 ---
    on(btn_calib.clicks) do _
        state.is_calibrating[] = true; empty!(state.calib_buffer)
        lbl_status.text = "Calibrating..."
    end

    on(btn_record.clicks) do _
        if !state.is_recording[]
            # 自动创建保存目录
            mkpath(CONFIG.data.save_dir)
            
            timestamp = Dates.format(now(), "yyyy-mm-dd_HHMMSS")
            fname = "imu_$(timestamp).csv"
            full_path = joinpath(CONFIG.data.save_dir, fname)
            
            state.file_io = open(full_path, "w")
            # 协议：硬件计数器, dt, 以及 36 字节负载
            println(state.file_io, "Counter,dt_us,AccX,AccY,AccZ,GyrX,GyrY,GyrZ,Roll,Pitch,Yaw")
            
            state.is_recording[] = true
            btn_record.label = "Stop"; btn_record.buttoncolor = :tomato
            lbl_status.text = "REC: $fname"
        else
            state.is_recording[] = false
            (state.file_io !== nothing) && close(state.file_io)
            state.file_io = nothing
            btn_record.label = "Record All"; btn_record.buttoncolor = :white
            lbl_status.text = "Saved to $(CONFIG.data.save_dir)"
        end
    end

    display(fig)

    # --- 串口处理后台线程 ---
    port_name = "/dev/ttyUSB0"; baudrate = 921600
    @async begin
        try
            sp = LibSerialPort.open(port_name, baudrate)
            raw_buffer = UInt8[]; last_ui_update = time(); boot_time = time()
            
            while events(fig).window_open[]
                n = bytesavailable(sp)
                if n > 0
                    append!(raw_buffer, read(sp, n))
                    (length(raw_buffer) > 10000) && deleteat!(raw_buffer, 1:(length(raw_buffer)-1040))
                    
                    while length(raw_buffer) >= 52
                        # 查找帧头 AA 55
                        idx = findfirst(i -> raw_buffer[i] == 0xAA && raw_buffer[i+1] == 0x55, 1:length(raw_buffer)-1)
                        if idx === nothing; deleteat!(raw_buffer, 1:length(raw_buffer)); break; end
                        
                        if length(raw_buffer) >= idx + 51
                            frame = raw_buffer[idx : idx+51]
                            
                            # 解析字段
                            cnt = reinterpret(UInt32, frame[5:8])[1]
                            dt  = reinterpret(UInt32, frame[9:12])[1]
                            fs  = reinterpret(Float32, frame[13:48])
                            raw_acc, raw_gyr, raw_eul = Vec3f(fs[1:3]), Vec3f(fs[4:6]), Vec3f(fs[7:9])

                            # 【保存】纯硬件数据
                            if state.is_recording[] && state.file_io !== nothing
                                @printf(state.file_io, "%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", cnt, dt, fs...)
                            end
                            
                            # 【校准】
                            if state.is_calibrating[]
                                push!(state.calib_buffer, raw_gyr)
                                if length(state.calib_buffer) >= 100
                                    state.gyro_offset = sum(state.calib_buffer)/100; state.is_calibrating[] = false
                                    lbl_status.text = "Calib Done"
                                end
                            end

                            # 【UI 刷新】
                            if (time() - last_ui_update) > CONFIG.data.refresh
                                state.run_time_str[] = @sprintf("%.1f s", time() - boot_time)
                                cur_gyr = raw_gyr - state.gyro_offset
                                for i in 1:3
                                    obs_acc[i].val = circshift(obs_acc[i].val, -1); obs_acc[i].val[end] = raw_acc[i]
                                    obs_gyr[i].val = circshift(obs_gyr[i].val, -1); obs_gyr[i].val[end] = cur_gyr[i]
                                    obs_eul[i].val = circshift(obs_eul[i].val, -1); obs_eul[i].val[end] = raw_eul[i]
                                    notify(obs_acc[i]); notify(obs_gyr[i]); notify(obs_eul[i])
                                    v_acc[i][] = @sprintf("%.3f", raw_acc[i]); v_gyr[i][] = @sprintf("%.2f", cur_gyr[i]); v_eul[i][] = @sprintf("%.2f", raw_eul[i])
                                end
                                autolimits!(ax_acc); autolimits!(ax_gyr); autolimits!(ax_eul)
                                last_ui_update = time()
                            end
                            deleteat!(raw_buffer, 1 : idx+51)
                        else break end
                    end
                end
                yield()
            end
            close(sp)
        catch e; println("Serial Error: $e"); end
    end

    while events(fig).window_open[]; yield(); sleep(0.01); end
end

start_app()