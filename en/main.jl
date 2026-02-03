using LibSerialPort, Printf, Dates, GLMakie, LinearAlgebra, GeometryBasics

# ==========================================================
# 1. Deep configuration center (modify save path and UI sizes here)
# ==========================================================
# The CONFIG tuple centralizes UI sizing, colors, timing, and data parameters.
# Modify these values to change the application's appearance and behavior.
const CONFIG = (
    window = (
        size       = (1650, 1000),    # Window pixel size (width, height)
        title      = "IMU Monitor",   # Window title text
        bg_color   = :white           # Background color for figure
    ),
    toolbar = (
        height     = 70,              # Toolbar height in pixels
        timer_w    = 180,             # Width reserved for uptime timer
        btn_w      = 120,             # Button width
        btn_h      = 38,              # Button height
        status_w   = 280,             # Status label width
        font_size  = 20               # Toolbar font size
    ),
    block = (
        header_h   = 45,              # Header height
        title_w    = 220,             # Title label width
        label_w    = 35,              # Axis label width (X, Y, Z)
        value_w    = 95,              # Value label width
        unit_w     = 60,              # Unit text width
        plot_gap   = 15,              # Vertical gap between plots
        ratios     = [0.26, 0.26, 0.26, 0.22] # Row layout ratios
    ),
    axis = (
        left = 8, right = 20, bottom = 12, top = 8  # Axis padding
    ),
    data = (
        history  = 600,               # Plot history samples
        refresh  = 0.05,              # Refresh interval (s)
        save_dir = "./imu_logs"       # CSV output directory
    )
)

# ==========================================================
# 2. Application State
# ==========================================================
# Manages runtime state, observables, and thread-safe buffers.
mutable struct AppState
    is_calibrating::Observable{Bool}   # Gyro calibration flag
    is_recording::Observable{Bool}     # Data recording flag
    run_time_str::Observable{String}   # Uptime string
    gyro_offset::Vec3f                 # Gyro bias vector
    calib_buffer::Vector{Vec3f}        # Calibration sample buffer
    calib_lock::ReentrantLock          # Lock for calibration buffer
    file_io::Union{IOStream, Nothing}  # CSV file handle
    file_lock::ReentrantLock           # Lock for file I/O
    
    # Initialize state with default values
    AppState() = new(Observable(false), Observable(false), Observable("0.0 s"), Vec3f(0), Vec3f[], ReentrantLock(), nothing, ReentrantLock())
end

# ==========================================================
# 3. UI Construction
# ==========================================================
# Creates a sensor block with a header and plotting axis.
# Returns the Axis and observables for value labels.
function create_sensor_block!(pos, title, unit, colors)
    # Block layout: Header + Axis
    block = pos[1, 1] = GridLayout(alignmode = Outside(20, 0, 0, 0))
    # Header layout with offset
    header = block[1, 1] = GridLayout(halign = :left, height = CONFIG.block.header_h, alignmode = Outside(30, 0, 0, 0))
    
    # Title
    Label(header[1, 1], title, font = :bold, fontsize = 19, width = CONFIG.block.title_w, halign = :left)
    
    # Value observables
    v_obs = [Observable("0.00") for _ in 1:3]
    labels = (title == "Euler Angles") ? ["R:", "P:", "Y:"] : ["X:", "Y:", "Z:"]
    
    # Value labels
    for i in 1:3
        Label(header[1, i*2], labels[i], color = :grey40, width = CONFIG.block.label_w, halign = :right)
        Label(header[1, i*2+1], v_obs[i], color = colors[i], width = CONFIG.block.value_w, halign = :left)
    end
    # Unit label
    Label(header[1, 8], unit, color = :grey60, width = CONFIG.block.unit_w, halign = :left)

    # Plotting Axis
    ax = Axis(block[2, 1], xgridvisible = true, ygridvisible = true)
    ax.alignmode = Mixed(left = CONFIG.axis.left, right = CONFIG.axis.right, 
                         bottom = CONFIG.axis.bottom, top = CONFIG.axis.top)
    return ax, v_obs
end

# ==========================================================
# 4. Main Application Logic
# ==========================================================
# Initializes UI, background tasks, and event handling.
function start_app()
    state = AppState()
    
    # Main window
    fig = Figure(size = CONFIG.window.size, backgroundcolor = CONFIG.window.bg_color)
    
    # Layout configuration
    colsize!(fig.layout, 1, Fixed(420))

    # Toolbar (Top)
    g_top   = fig[1, 2] = GridLayout(height = CONFIG.toolbar.height)
    # Main Content Area
    g_main  = fig[2, 2] = GridLayout()
    colgap!(g_main, 100) 
    rowsize!(fig.layout, 1, Fixed(CONFIG.toolbar.height))

    # --- Toolbar Widgets ---
    t_box = g_top[1, 1] = GridLayout(width = CONFIG.toolbar.timer_w)
    Label(t_box[1, 1], "Uptime:", color = :grey30, fontsize = 15)
    Label(t_box[1, 2], state.run_time_str, font = :bold, fontsize = 20, width = 85)

    # Buttons and Status
    btn_calib  = Button(g_top[1, 2], label = "Calibrate", width = CONFIG.toolbar.btn_w, height = CONFIG.toolbar.btn_h)
    btn_record = Button(g_top[1, 3], label = "Record All", width = CONFIG.toolbar.btn_w, height = CONFIG.toolbar.btn_h)
    lbl_status = Label(g_top[1, 4], "Ready", width = CONFIG.toolbar.status_w, halign = :left, color = :blue)
    
    Label(g_top[1, 5], CONFIG.window.title, font = :bold, fontsize = CONFIG.toolbar.font_size, halign = :right)
    colsize!(g_top, 5, Relative(1))

    # --- Layout: Charts (Left) vs 3D (Right) ---
    g_left   = g_main[1, 1] = GridLayout()  
    g_right  = g_main[1, 2] = GridLayout()
    
    # Column sizing
    colsize!(g_main, 1, Fixed(380))
    colsize!(g_main, 2, Relative(1))

    # --- Sensor Blocks ---
    colors = [:red, :forestgreen, :dodgerblue]
    ax_acc, v_acc = create_sensor_block!(g_left[1, 1], "Accelerometer", "g", colors)
    ax_gyr, v_gyr = create_sensor_block!(g_left[2, 1], "Gyroscope", "d/s", colors)
    ax_eul, v_eul = create_sensor_block!(g_left[3, 1], "Euler Angles", "deg", colors)

    # Chart distribution
    rowsize!(g_left, 1, Relative(0.33))
    rowsize!(g_left, 2, Relative(0.33))
    rowsize!(g_left, 3, Relative(0.33))
    rowgap!(g_left, CONFIG.block.plot_gap)
    
    # --- 3D Visualization ---
    # 3D Axis setup
    ax3d = Axis3(g_right[1, 1], 
                 title = "IMU 3D View",
                 xlabel = "X", ylabel = "Y", zlabel = "Z",
                 xlabelfont = :bold, ylabelfont = :bold, zlabelfont = :bold,
                 xlabelsize = 25, ylabelsize = 25, zlabelsize = 25, 
                 xlabelrotation = 0, ylabelrotation = 0, zlabelrotation = 0,
                 xgridvisible = true, ygridvisible = true, zgridvisible = true,
                 xticklabelsvisible = false, yticklabelsvisible = false, zticklabelsvisible = false, # Hide labels
                 xticksvisible = false, yticksvisible = false, zticksvisible = false, # Hide ticks
                 aspect = :equal,
                 limits = (-1.2, 1.2, -1.2, 1.2, -1.2, 1.2),
                 viewmode = :fit)
    
    # PCB Geometry Setup
    # Dimensions
    pcb_width = 0.6f0
    pcb_height = 1.0f0
    pcb_thickness = 0.05f0
    
    # Define PCB vertices (centered)
    function create_pcb_vertices()
        w2 = pcb_width / 2
        h2 = pcb_height / 2
        t2 = pcb_thickness / 2
        return [
            Point3f(-w2, -h2, -t2),
            Point3f( w2, -h2, -t2),
            Point3f( w2,  h2, -t2),
            Point3f(-w2,  h2, -t2),
            Point3f(-w2, -h2,  t2),
            Point3f( w2, -h2,  t2),
            Point3f( w2,  h2,  t2),
            Point3f(-w2,  h2,  t2),
        ]
    end
    
    # Define PCB faces
    function create_pcb_faces()
        return [
            [4, 3, 2, 1],  # Bottom
            [5, 6, 7, 8],  # Top
            [1, 2, 6, 5],  # Front
            [3, 4, 8, 7],  # Back
            [5, 8, 4, 1],  # Left
            [2, 3, 7, 6],  # Right
        ]
    end

    # --- Geometry & Coloring ---
    # Duplicate vertices for flat shading with distinct face colors
    base_8_verts = create_pcb_vertices()
    face_indices_list = create_pcb_faces()
    
    # Face colors: Bottom, Top, Front, Back, Left, Right
    face_palette = [:grey, :dodgerblue, :forestgreen, :gold, :darkorange, :purple]
    
    flat_base_verts = Point3f[]
    flat_colors = Symbol[]
    flat_triangles = UInt32[]
    
    let current_idx = 0
        for (i, face_idxs) in enumerate(face_indices_list)
            # Vertices for current face
            for idx in face_idxs
                push!(flat_base_verts, base_8_verts[idx])
                push!(flat_colors, face_palette[i])
            end
            # Triangles for current face
            push!(flat_triangles, current_idx+1, current_idx+2, current_idx+3)
            push!(flat_triangles, current_idx+1, current_idx+3, current_idx+4)
            current_idx += 4
        end
    end
    
    # Euler angles observable
    euler_obs = Observable(Vec3f(0, 0, 0))
    
    # Convert Euler angles to rotation matrix
    function euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg)
        roll = deg2rad(roll_deg)
        pitch = deg2rad(pitch_deg)
        yaw = deg2rad(yaw_deg)
        
        Rx = [1.0  0.0         0.0        ;
              0.0  cos(roll)  -sin(roll) ;
              0.0  sin(roll)   cos(roll)]
        
        Ry = [ cos(pitch)  0.0  sin(pitch) ;
               0.0         1.0  0.0        ;
              -sin(pitch)  0.0  cos(pitch)]
        
        Rz = [cos(yaw)  -sin(yaw)  0.0 ;
              sin(yaw)   cos(yaw)  0.0 ;
              0.0        0.0       1.0]
        
        # ZYX order
        return Rz * Ry * Rx
    end
    
    # Update PCB mesh based on rotation
    function create_pcb_mesh(roll_deg, pitch_deg, yaw_deg)
        R = euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg)
        rotated_vertices = Point3f[]
        for v in flat_base_verts
            v_vec = [v[1], v[2], v[3]]
            rotated_vec = R * v_vec
            push!(rotated_vertices, Point3f(rotated_vec[1], rotated_vec[2], rotated_vec[3]))
        end
        return GeometryBasics.Mesh(rotated_vertices, flat_triangles)
    end
    
    pcb_mesh_obs = Observable(create_pcb_mesh(0, 0, 0))
    
    # Render mesh
    pcb_mesh = mesh!(ax3d, 
                     pcb_mesh_obs,
                     color = flat_colors,
                     transparency = false,
                     shading = NoShading)
    
    # Update mesh when Euler angles change
    on(euler_obs) do euler
        pcb_mesh_obs[] = create_pcb_mesh(euler[1], euler[2], euler[3])
    end
    
    # --- Custom Extensions ---
    g_extension = g_main[2, 1:3] = GridLayout()
    btn_task = Button(g_extension[1, 1], label = "Custom Task", width = 140, height = 40)
    Label(g_extension[1, 2], "Files saved to: $(abspath(CONFIG.data.save_dir))", color = :grey60, halign = :left)
    
    on(btn_task.clicks) do _
        println("Extension button clicked at $(state.run_time_str[])")
    end
    
    rowsize!(g_main, 2, Fixed(60))

    # --- Plot Data Buffers ---
    obs_acc = [Observable(fill(0.0f0, CONFIG.data.history)) for _ in 1:3]
    obs_gyr = [Observable(fill(0.0f0, CONFIG.data.history)) for _ in 1:3]
    obs_eul = [Observable(fill(0.0f0, CONFIG.data.history)) for _ in 1:3]
    for i in 1:3
        lines!(ax_acc, obs_acc[i], color = colors[i], linewidth = 1.5)
        lines!(ax_gyr, obs_gyr[i], color = colors[i], linewidth = 1.5)
        lines!(ax_eul, obs_eul[i], color = colors[i], linewidth = 1.5)
    end

    # --- Event Handling ---
    on(btn_calib.clicks) do _
        # Start calibration
        lock(state.calib_lock) do
            empty!(state.calib_buffer)
        end
        state.is_calibrating[] = true
        lbl_status.text = "Calibrating..."
    end

    on(btn_record.clicks) do _
        if !state.is_recording[]
            # Start recording
            mkpath(CONFIG.data.save_dir)
            
            timestamp = Dates.format(now(), "yyyy-mm-dd_HHMMSS")
            fname = "imu_$(timestamp).csv"
            full_path = joinpath(CONFIG.data.save_dir, fname)
            
            lock(state.file_lock) do
                state.file_io = open(full_path, "w")
                println(state.file_io, "Counter,dt_us,AccX,AccY,AccZ,GyrX,GyrY,GyrZ,Roll,Pitch,Yaw")
            end
            
            state.is_recording[] = true
            btn_record.label = "Stop"; btn_record.buttoncolor = :tomato
            lbl_status.text = "REC: $fname"
        else
            # Stop recording
            state.is_recording[] = false
            btn_record.label = "Record All"; btn_record.buttoncolor = :white
            lbl_status.text = "Saved to $(CONFIG.data.save_dir)"
        end
    end

    display(fig)

    # --- Background Task: Serial Read ---
    # Handles data reading, parsing, and buffer updates.

    function find_serial_port()
        ports = LibSerialPort.get_port_list()
        if isempty(ports)
            println("Error: No serial ports found. Connect a device.")
            return nothing
        end
        println("Available serial ports: ", ports)
        
        local port_to_use
        if Sys.iswindows()
            com_ports = sort(filter(p -> startswith(p, "COM"), ports))
            port_to_use = isempty(com_ports) ? ports[1] : com_ports[end]
        else
            port_to_use = ports[1]
        end
        println("Attempting to use port: ", port_to_use)
        return port_to_use
    end

    port_name = find_serial_port()
    baudrate = 921600
    
    @async begin
        if port_name === nothing
            lbl_status.text = "No Serial Port Found!"
            lbl_status.color = :red
            return
        end

        try
            sp = LibSerialPort.open(port_name, baudrate)
            raw_buffer = UInt8[]; last_ui_update = time(); boot_time = time()
            
            while events(fig).window_open[]
                # Cleanup file IO if stopped
                if !state.is_recording[] && state.file_io !== nothing
                    lock(state.file_lock) do
                        if state.file_io !== nothing
                            close(state.file_io)
                            state.file_io = nothing
                        end
                    end
                end

                n = bytesavailable(sp)
                if n > 0
                    append!(raw_buffer, read(sp, n))
                    (length(raw_buffer) > 10000) && deleteat!(raw_buffer, 1:(length(raw_buffer)-1040))
                    
                    while length(raw_buffer) >= 52
                        # Sync sequence: 0xAA 0x55
                        idx = findfirst(i -> raw_buffer[i] == 0xAA && raw_buffer[i+1] == 0x55, 1:length(raw_buffer)-1)
                        if idx === nothing
                            empty!(raw_buffer); break
                        end
                        
                        if length(raw_buffer) >= idx + 51
                            frame = raw_buffer[idx : idx+51]
                            
                            cnt = reinterpret(UInt32, frame[5:8])[1]
                            dt  = reinterpret(UInt32, frame[9:12])[1]
                            fs  = reinterpret(Float32, frame[13:48])
                            raw_acc, raw_gyr, raw_eul = Vec3f(fs[1:3]), Vec3f(fs[4:6]), Vec3f(fs[7:9])

                            # Save to file
                            lock(state.file_lock) do
                                if state.is_recording[] && state.file_io !== nothing
                                    @printf(state.file_io, "%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", cnt, dt, fs...)
                                end
                            end
                            
                            # Calibration accumulation
                            if state.is_calibrating[]
                                calib_finished = false
                                lock(state.calib_lock) do
                                    if state.is_calibrating[]
                                        push!(state.calib_buffer, raw_gyr)
                                        if length(state.calib_buffer) >= 100
                                            state.gyro_offset = sum(state.calib_buffer) / length(state.calib_buffer)
                                            calib_finished = true
                                        end
                                    end
                                end
                                if calib_finished
                                    state.is_calibrating[] = false
                                    lbl_status.text = "Calib Done"
                                end
                            end

                            # Update UI
                            if (time() - last_ui_update) > CONFIG.data.refresh
                                state.run_time_str[] = @sprintf("%.1f s", time() - boot_time)
                                cur_gyr = raw_gyr - state.gyro_offset
                                for i in 1:3
                                    obs_acc[i].val = circshift(obs_acc[i].val, -1); obs_acc[i].val[end] = raw_acc[i]
                                    obs_gyr[i].val = circshift(obs_gyr[i].val, -1); obs_gyr[i].val[end] = cur_gyr[i]
                                    obs_eul[i].val = circshift(obs_eul[i].val, -1); obs_eul[i].val[end] = raw_eul[i]
                                    notify(obs_acc[i]); notify(obs_gyr[i]); notify(obs_eul[i])
                                    v_acc[i][] = @sprintf("%.3f", raw_acc[i]); v_gyr[i][] = @sprintf("%.3f", cur_gyr[i]); v_eul[i][] = @sprintf("%.3f", raw_eul[i])
                                end
                                euler_obs[] = Vec3f(raw_eul[1], raw_eul[2], raw_eul[3])
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
        catch e
            println("Serial Error: $e")
            lbl_status.text = "Serial Error!"
            lbl_status.color = :red
        end
    end

    while events(fig).window_open[]; yield(); sleep(0.01); end
end

start_app()
