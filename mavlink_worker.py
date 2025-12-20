import time
import csv
from PyQt6.QtCore import QThread, pyqtSignal, QTimer
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

class MavlinkWorker(QThread):
    new_param_value = pyqtSignal(str, float)
    connection_status = pyqtSignal(bool)
    log_message = pyqtSignal(str)
    new_telemetry_data = pyqtSignal(dict)
    
    start_saving_signal = pyqtSignal(str, list)
    stop_saving_signal = pyqtSignal()

    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.master = None
        self.running = True
        
        self.is_saving = False
        self.data_file = None
        self.csv_writer = None
        self.saved_variables = []
        self.start_timestamp = 0
        
        self.start_saving_signal.connect(self.start_saving_to_csv)
        self.stop_saving_signal.connect(self.stop_saving_to_csv)
        
        self.current_data = {}
        self.telemetry_buffer = {}
        
        # UI更新频率为20Hz
        self.last_emit_time = time.time()
        self.emit_interval = 0.05 # 50ms (20 Hz)
        
        # 新增：用于稳定数据保存频率的定时器
        self.save_timer = QTimer(self)
        self.save_timer.timeout.connect(self._save_data_to_csv)

    def run(self):
        try:
            self.log_message.emit(f"Attempting to connect to {self.port}...")
            # 修复：增加连接超时时间
            self.master = mavutil.mavlink_connection(self.port, baud=self.baudrate, timeout=5) 
            self.master.wait_heartbeat()
            self.log_message.emit("Heartbeat received, connection successful!")
            self.connection_status.emit(True)
            
            while self.running:
                msg = self.master.recv_match(
                    type=['HEARTBEAT', 'SYS_STATUS', 'VFR_HUD', 'ATTITUDE', 'GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'HOME_POSITION', 'PARAM_VALUE', 'STATUSTEXT', 'RAW_IMU', 'SCALED_PRESSURE'], 
                    blocking=False
                )
                
                if not msg:
                    # 避免CPU空转，略微休眠
                    time.sleep(0.001) 
                    continue
                
                msg_type = msg.get_type()
                
                if msg_type == 'STATUSTEXT':
                    self.log_message.emit(f"FCU message: {msg.text}")
                    continue
                elif msg_type == 'PARAM_VALUE':
                    #self.log_message.emit("get params")
                    param_id = msg.param_id.strip()
                    param_value = msg.param_value
                    self.new_param_value.emit(param_id, param_value)
                    continue
                else:
                    data = {}
                    if msg_type == 'ATTITUDE':
                        data = {'roll': msg.roll, 'pitch': msg.pitch, 'yaw': msg.yaw}
                    elif msg_type == 'SYS_STATUS':
                        data = {'voltage_battery': msg.voltage_battery / 1000.0, 'current_battery': msg.current_battery / 100.0, 'load': msg.load / 10.0}
                    elif msg_type == 'VFR_HUD':
                        data = {'airspeed': msg.airspeed, 'groundspeed': msg.groundspeed, 'heading': msg.heading, 'alt': msg.alt, 'climb': msg.climb}
                    elif msg_type == 'GLOBAL_POSITION_INT':
                        data = {'lat': msg.lat / 1e7, 'lon': msg.lon / 1e7, 'alt': msg.alt / 1000}
                    elif msg_type == 'GPS_RAW_INT':
                        data = {'fix_type': msg.fix_type, 'satellites_visible': msg.satellites_visible, 'eph': msg.eph}
                    elif msg_type == 'HOME_POSITION':
                        data = {'home_lat': msg.latitude / 1e7, 'home_lon': msg.longitude / 1e7}
                    elif msg_type == 'RAW_IMU':
                        data = {'acc_x': msg.xacc, 'acc_y': msg.yacc, 'acc_z': msg.zacc,
                                'gyro_x': msg.xgyro, 'gyro_y': msg.ygyro, 'gyro_z': msg.zgyro,
                                'mag_x': msg.xmag, 'mag_y': msg.ymag, 'mag_z': msg.zmag}
                    elif msg_type == 'SCALED_PRESSURE':
                        data = {'abs_pressure': msg.press_abs, 'diff_pressure': msg.press_diff}
                    elif msg_type == 'HEARTBEAT':
                        data = {'type': msg.type,'autopilot': msg.autopilot,'base_mode': msg.base_mode,'custom_mode': msg.custom_mode,'system_status': msg.system_status, 'mavlink_version': msg.mavlink_version}
                    
                    # 更新全局数据字典
                    for var_name, value in data.items():
                        full_key = f"{msg_type}.{var_name}"
                        self.current_data[full_key] = value

                    # 缓冲数据用于批量发送到主线程（20Hz）
                    if msg_type not in self.telemetry_buffer:
                        self.telemetry_buffer[msg_type] = []
                    self.telemetry_buffer[msg_type].append(data)
                    
                    if (time.time() - self.last_emit_time) > self.emit_interval:
                        if self.telemetry_buffer:
                            self.new_telemetry_data.emit(self.telemetry_buffer)
                            self.telemetry_buffer = {}
                            self.last_emit_time = time.time()
        except Exception as e:
            self.log_message.emit(f"Connection failed or an error occurred: {e}")
            self.connection_status.emit(False)
        finally:
            self.stop_saving_to_csv()
            if self.master:
                self.master.close()
            self.log_message.emit("Mavlink connection closed.")
    
    def _save_data_to_csv(self):
        """
        每 2ms 被定时器调用，用于统一频率保存数据。
        """
        if self.is_saving and self.csv_writer:
            current_timestamp = time.time() - self.start_timestamp
            row = [current_timestamp]
            for var_key in self.saved_variables:
                value = self.current_data.get(var_key, 'NaN')
                row.append(value)
            
            self.csv_writer.writerow(row)

    def start_saving_to_csv(self, filename, variables):
        if self.is_saving:
            self.log_message.emit("Warning: Already saving data. Cannot start new session.")
            return

        try:
            self.data_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.data_file)
            
            header = ['timestamp'] + variables
            self.csv_writer.writerow(header)
            
            self.is_saving = True
            self.saved_variables = variables
            self.start_timestamp = time.time()
            self.log_message.emit(f"Worker started saving data to: {filename} at 500Hz.")
            self.save_timer.start(2) # 启动定时器，每 2ms (500Hz) 触发一次
        except IOError as e:
            self.log_message.emit(f"Error saving file: {e}")
            self.is_saving = False

    def stop_saving_to_csv(self):
        self.save_timer.stop()
        if self.data_file:
            self.data_file.close()
        self.data_file = None
        self.csv_writer = None
        self.is_saving = False
        self.log_message.emit("Worker stopped saving data.")

    def request_param_list(self):
        if self.master:
            # 修复：传入 target_system 和 target_component 参数
            self.master.mav.param_request_list_send(self.master.target_system, self.master.target_component) 
            self.log_message.emit("Parameter list request sent.")
    
    def set_param(self, param_id, value, param_type):
        if self.master:
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                param_id.encode('utf-8'),
                value,
                param_type
            )
            self.log_message.emit(f"Parameter '{param_id}' set to {value}")

    def stop(self):
        self.running = False
        self.wait()
