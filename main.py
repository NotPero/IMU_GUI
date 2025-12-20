import sys
import csv
import time
from PyQt6.QtWidgets import QApplication, QMainWindow, QAbstractItemView, QFileDialog, QTreeWidgetItem
from PyQt6.QtCore import QTimer, Qt, pyqtSignal
from ui_mainwindow import Ui_MainWindow
from mavlink_worker import MavlinkWorker
from pymavlink.dialects.v20 import common as mavlink2
from pymavlink import mavutil 
from plot_window import PlotWindow
import serial.tools.list_ports

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        
        self.serial_port_timer = QTimer(self)
        self.serial_port_timer.timeout.connect(self.populate_serial_ports)
        self.serial_port_timer.start(2000)

        # QTimer for refreshing plot every 50ms (20 Hz)
        self.plot_update_timer = QTimer(self)
        self.plot_update_timer.timeout.connect(self.update_plots)
        self.plot_update_timer.start(50) 

        # 新增: 参数请求重试定时器
        self.param_request_timer = QTimer(self)
        self.param_request_timer.timeout.connect(self._check_and_request_params)
        
        self.connect_button.clicked.connect(self.on_connect_clicked)
        self.request_param_button.clicked.connect(self.on_request_param_clicked)
        self.set_param_button.clicked.connect(self.on_set_param_clicked)
        self.param_tree_widget.itemClicked.connect(self.on_param_tree_item_clicked)
        self.plot_selected_button.clicked.connect(self.on_plot_selected_clicked)
        self.save_data_button.clicked.connect(self.on_save_data_clicked)
        self.save_plot_button.clicked.connect(self.on_save_plot_clicked)

        self.mavlink_worker = None
        self.params = {}
        self.param_parent_items = {}
        self.telemetry_data = {}
        self.plotted_variables = set()

        self.plot_window = PlotWindow("Real-time Data Plot")
        
        self.is_saving_data = False
        self.saved_variables = []
        self.start_timestamp = 0
        
        self.monitored_messages = {
            'ATTITUDE': ['roll', 'pitch', 'yaw'],
            'GLOBAL_POSITION_INT': ['lat', 'lon', 'alt'],
            'VFR_HUD': ['airspeed', 'groundspeed', 'heading', 'alt', 'climb'],
            'SYS_STATUS': ['voltage_battery', 'current_battery', 'current_battery', 'load'],
            'GPS_RAW_INT': ['fix_type', 'satellites_visible', 'eph'],
            'HOME_POSITION': ['home_lat', 'home_lon'],
            'RAW_IMU': ['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'mag_x', 'mag_y', 'mag_z'],
            'SCALED_PRESSURE': ['abs_pressure', 'diff_pressure']
        }
        
        self.populate_serial_ports()

    def populate_serial_ports(self):
        current_port = self.port_combo_box.currentText()
        ports = serial.tools.list_ports.comports()
        
        EXCLUDED_PORTS = ['COM1', 'COM2', 'COM3']
        
        filtered_ports = [
            port.device for port in ports 
            if port.device not in EXCLUDED_PORTS and not port.device.startswith('/dev/ttyS')
        ]
        
        if current_port and current_port in filtered_ports:
            pass
        else:
            self.port_combo_box.clear()
            if not filtered_ports:
                self.port_combo_box.addItem("No available ports found")
                self.connect_button.setEnabled(False)
            else:
                self.port_combo_box.addItems(filtered_ports)
                self.connect_button.setEnabled(True)

    def populate_variable_list_dynamically(self, new_data):
        for msg_type, variable_list in new_data.items():
            parent_item = self.variable_tree_widget.findItems(msg_type, Qt.MatchFlag.MatchExactly)
            if not parent_item:
                parent_item = QTreeWidgetItem(self.variable_tree_widget, [msg_type])
                parent_item.setFlags(parent_item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
                parent_item.setCheckState(0, Qt.CheckState.Unchecked)
            else:
                parent_item = parent_item[0]
            
            variables = variable_list[0].keys()
            for var in variables:
                item_name = f"{msg_type}.{var}"
                if item_name not in self.plotted_variables:
                    child_item = QTreeWidgetItem(parent_item, [var])
                    child_item.setFlags(child_item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
                    child_item.setCheckState(0, Qt.CheckState.Unchecked)
                    self.plotted_variables.add(item_name)
    
    def on_connect_clicked(self):
        port = self.port_combo_box.currentText()
        baudrate_str = self.baudrate_input.text().strip()
        if port == "No available ports found" or not baudrate_str:
            self.update_log("Error: Please select a valid port and enter a baudrate.")
            return
        baudrate = int(baudrate_str)
        
        if self.connect_button.text() == "Connect":
            self.mavlink_worker = MavlinkWorker(port, baudrate)
            self.mavlink_worker.connection_status.connect(self.update_connection_status)
            self.mavlink_worker.new_param_value.connect(self.add_or_update_param)
            self.mavlink_worker.log_message.connect(self.update_log)
            
            self.mavlink_worker.new_telemetry_data.connect(self.buffer_telemetry_data)
            
            self.mavlink_worker.new_telemetry_data.connect(self.populate_variable_list_dynamically)
            
            self.mavlink_worker.start()
            self.serial_port_timer.stop()
        else:
            if self.mavlink_worker:
                self.mavlink_worker.stop()
                self.mavlink_worker = None
            self.connect_button.setText("Connect")
            self.update_connection_status(False)
            self.serial_port_timer.start(2000)
    
    def on_plot_selected_clicked(self):
        selected_variables = []
        root = self.variable_tree_widget.invisibleRootItem()
        for i in range(root.childCount()):
            parent_item = root.child(i)
            if parent_item.checkState(0) == Qt.CheckState.Checked:
                for j in range(parent_item.childCount()):
                    child_item = parent_item.child(j)
                    selected_variables.append(f"{parent_item.text(0)}.{child_item.text(0)}")
            else:
                for j in range(parent_item.childCount()):
                    child_item = parent_item.child(j)
                    if child_item.checkState(0) == Qt.CheckState.Checked:
                        selected_variables.append(f"{parent_item.text(0)}.{child_item.text(0)}")
        
        if not selected_variables:
            self.update_log("Warning: Please select at least one variable to plot.")
            return
        
        self.plot_window.show()

    def on_save_data_clicked(self):
        if not self.is_saving_data:
            self.start_saving_data()
        else:
            self.stop_saving_data()
            
    def on_save_plot_clicked(self):
        self.plot_window.save_snapshot()

    def start_saving_data(self):
        selected_variables = []
        root = self.variable_tree_widget.invisibleRootItem()
        for i in range(root.childCount()):
            parent_item = root.child(i)
            if parent_item.checkState(0) == Qt.CheckState.Checked:
                for j in range(parent_item.childCount()):
                    child_item = parent_item.child(j)
                    selected_variables.append(f"{parent_item.text(0)}.{child_item.text(0)}")
            else:
                for j in range(parent_item.childCount()):
                    child_item = parent_item.child(j)
                    if child_item.checkState(0) == Qt.CheckState.Checked:
                        selected_variables.append(f"{parent_item.text(0)}.{child_item.text(0)}")
        
        if not selected_variables:
            self.update_log("Error: Please select variables to save first.")
            return
        
        filename = self.filename_input.text().strip()
        if not filename:
            filename = "flight_data.csv"
        
        if self.mavlink_worker and self.mavlink_worker.isRunning():
            self.saved_variables = selected_variables
            self.is_saving_data = True
            self.save_data_button.setText("Stop Saving")
            self.update_log(f"Requesting worker thread to start saving to file: {filename}")
            
            self.mavlink_worker.start_saving_signal.emit(filename, selected_variables)
        else:
            self.update_log("Error: Please connect to a device first.")
            
    def stop_saving_data(self):
        if self.mavlink_worker and self.mavlink_worker.isRunning():
            self.is_saving_data = False
            self.save_data_button.setText("Start Saving")
            self.update_log("Requesting worker thread to stop saving.")
            
            self.mavlink_worker.stop_saving_signal.emit()
        else:
            self.update_log("Warning: Not connected to a device.")

    def buffer_telemetry_data(self, data):
        for msg_type, msg_list in data.items():
            for data_dict in msg_list:
                if msg_type not in self.telemetry_data:
                    self.telemetry_data[msg_type] = {}
                for var, value in data_dict.items():
                    self.telemetry_data[msg_type][var] = value
        
        self.populate_variable_list_dynamically(data)
    
    def update_plots(self):
        selected_data_for_plot = {}
        root = self.variable_tree_widget.invisibleRootItem()
        for i in range(root.childCount()):
            parent_item = root.child(i)
            for j in range(parent_item.childCount()):
                child_item = parent_item.child(j)
                if child_item.checkState(0) == Qt.CheckState.Checked:
                    key = f"{parent_item.text(0)}.{child_item.text(0)}"
                    msg_type, var_name = key.split('.')
                    if msg_type in self.telemetry_data and var_name in self.telemetry_data[msg_type]:
                        selected_data_for_plot[key] = self.telemetry_data[msg_type][var_name]

        self.plot_window.update_plot(selected_data_for_plot)

    def on_request_param_clicked(self):
        if self.mavlink_worker and self.mavlink_worker.isRunning():
            self.param_tree_widget.clear()
            self.params.clear()
            self.param_parent_items.clear()
            self.update_log("Starting parameter list request...")
            self.mavlink_worker.request_param_list()
        else:
            self.update_log("Error: Please connect to a device first.")
    
    def _check_and_request_params(self):
        if self.mavlink_worker and self.mavlink_worker.isRunning() and not self.params:
            self.update_log("No parameters received yet. Re-sending request...")
            self.mavlink_worker.request_param_list()

    def on_set_param_clicked(self):
        if self.mavlink_worker and self.mavlink_worker.isRunning():
            param_id = self.param_id_input.text().strip()
            param_value_str = self.param_value_input.text().strip()
            if param_id and param_value_str:
                try:
                    param_value = float(param_value_str)
                    param_type = mavlink2.MAV_PARAM_TYPE_REAL32
                    self.mavlink_worker.set_param(param_id, param_value, param_type)
                except ValueError:
                    self.update_log("Invalid input value. Please check the parameter type.")
            else:
                self.update_log("Error: Parameter name and new value cannot be empty.")
        else:
            self.update_log("Error: Please connect to a device first.")

    def on_param_tree_item_clicked(self, item, column):
        if item.parent(): 
            param_id = item.text(0)
            param_value = item.text(1)
            self.param_id_input.setText(param_id)
            self.param_value_input.setText(param_value)

    def update_connection_status(self, is_connected):
        if is_connected:
            self.status_label.setText("Status: Connected")
            self.connect_button.setText("Disconnect")
            self.on_request_param_clicked()
            # 新增: 启动重试定时器，每5秒检查一次
            self.param_request_timer.start(5000)
        else:
            self.status_label.setText("Status: Disconnected")
            self.connect_button.setText("Connect")
            # 停止定时器
            self.param_request_timer.stop()
            
    def add_or_update_param(self, param_id, param_value):
        # 如果是收到的第一个参数，停止重试定时器
        if not self.params:
            self.param_request_timer.stop()
            self.update_log("First parameter received. Stopping retry timer.")

        self.params[param_id] = param_value
        #self.update_log(f"Received PARAM_VALUE for: {param_id}")
        
        prefix = param_id.split('_')[0]
        if prefix not in self.param_parent_items:
            parent_item = QTreeWidgetItem(self.param_tree_widget, [prefix])
            self.param_parent_items[prefix] = parent_item
        else:
            parent_item = self.param_parent_items[prefix]
        
        found_item = None
        for i in range(parent_item.childCount()):
            if parent_item.child(i).text(0) == param_id:
                found_item = parent_item.child(i)
                break
        
        if found_item:
            found_item.setText(1, f"{param_value:.4f}")
        else:
            child_item = QTreeWidgetItem(parent_item, [param_id, f"{param_value:.4f}"])
            
    def update_log(self, message):
        self.log_list_widget.addItem(message)
        self.log_list_widget.scrollToBottom()

    def closeEvent(self, event):
        if self.is_saving_data:
            self.stop_saving_data()
        
        if self.mavlink_worker and self.mavlink_worker.isRunning():
            self.mavlink_worker.stop()
        
        # 停止所有定时器
        self.param_request_timer.stop()
        self.plot_update_timer.stop()
        self.serial_port_timer.stop()
        
        self.plot_window.close()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    try:
        with open("qgroundcontrol.qss", "r") as f:
            app.setStyleSheet(f.read())
    except FileNotFoundError:
        print("Warning: qgroundcontrol.qss file not found, using default style.")

    window = MainWindow()
    window.show()
    sys.exit(app.exec())
