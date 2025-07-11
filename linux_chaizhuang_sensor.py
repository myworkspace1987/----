#!/usr/bin/python
import socket
import threading
import sys
import time
import requests
import tkinter as tk
from tkinter import messagebox, simpledialog

# Sensor command definitions
cmdLuxSensor = b'\x02\x04\x00\x07\x00\x01\x80\x38'
cmdTempSensor = b'\x03\x04\x00\x01\x00\x01\x61\xE8'
cmdHumiditySensor = b'\x03\x03\x00\x00\x00\x01\x85\xE8'
cmdLidarSensor = b'\x04\x03\x00\x0C\x00\x01\x44\x5C'
cmdSmokeSensor = b'\x05\x04\x00\x00\x00\x01\x30\x4E'
cmdPM25Sensor = b'\x08\x04\x00\x02\x00\x01\x90\x93'
cmdNoiseSensor = b'\x08\x04\x00\x0A\x00\x01\x11\x51'
cmdFanSensor = b'\x06\x03\x00\x01\x00\x01\xD4\x7D'
cmdPowerSensor = b'\x01\x04\x00\x00\x00\x01\x31\xCA'

sensor_data = {}
# Initial server IP (will be updated by GUI)
server_ip = "192.168.69.172"
server_ip = "192.168.69.253"
server_port = 1030

# Global variables for GUI updates
current_ip_var = None
data_success_count_var = None
sensor_display_vars = {}
data_acquisition_success_count = 0
socket_connection_status = False
sock = None
receive_thread = None
stop_threads = False

def get_local_ip():
    """Attempts to get the local machine's IP address."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))  # Connect to a public server to get local IP
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception as e:
        print(f"Error getting local IP: {e}")
        return "N/A"

def update_gui_ip_display():
    """Updates the displayed local IP address in the GUI."""
    if current_ip_var:
        current_ip_var.set(f"当前本机IP: {get_local_ip()}")

def update_data_success_count():
    """Updates the successful data acquisition count in the GUI."""
    if data_success_count_var:
        data_success_count_var.set(f"成功获取数据次数: {data_acquisition_success_count}")

def update_sensor_display(sensor_name, value):
    """Updates the display for a specific sensor in the GUI."""
    if sensor_name in sensor_display_vars:
        sensor_display_vars[sensor_name].set(f"{sensor_name.capitalize()}: {value}")

def receive_data(socket_obj, signal):
    """Socket data reception thread."""
    global data_acquisition_success_count
    while signal():
        try:
            rcv_data = socket_obj.recv(256)
            # print(rcv_data.hex()) # Uncomment for debugging received hex data

            if rcv_data:
                data_acquisition_success_count += 1
                update_data_success_count()

                if rcv_data[0] == 0x01:
                    if len(rcv_data) == 7:
                        power_val = (rcv_data[3] * 256 + rcv_data[4]) / 10
                        update_sensor_display('电压', f"{power_val:.1f}V")
                        sensor_data.update({'power': str(power_val)})
                elif rcv_data[0] == 0x02:
                    lux_val = (rcv_data[3] * 256 + rcv_data[4]) / 10
                    update_sensor_display('照度', f"{lux_val:.1f} Lux")
                    sensor_data.update({'lux': str(lux_val)})
                elif rcv_data[0] == 0x03:
                    temp_val = (rcv_data[3] * 256 + rcv_data[4]) / 10
                    update_sensor_display('温度', f"{temp_val:.1f}°C")
                    sensor_data.update({'temp': str(temp_val)})
                    # Assuming humidity is also received with temperature or needs a separate command
                    # For simplicity, if humidity comes with the same command, parse it here
                elif rcv_data[0] == 0x04:
                    lidar_val = rcv_data[3] * 256 + rcv_data[4]
                    update_sensor_display('雷达', f"{lidar_val} cm")
                    sensor_data.update({'lidar': str(lidar_val)})
                elif rcv_data[0] == 0x05:
                    smoke_val = rcv_data[3] * 256 + rcv_data[4]
                    update_sensor_display('烟雾', f"{smoke_val}")
                    sensor_data.update({'smoke': str(smoke_val)})
                elif rcv_data[0] == 0x06:
                    fan_data = rcv_data[-3:-2][-1]
                    fan_status = "开启" if fan_data == 8 else "关闭"
                    update_sensor_display('风扇', fan_status)
                    sensor_data.update({'fan': fan_status}) # Store 'true' or 'false'
                elif rcv_data[0] == 0x08:
                    pm25_val = rcv_data[3] * 256 + rcv_data[4]
                    update_sensor_display('PM2.5', f"{pm25_val} μg/m³")
                    sensor_data.update({'pm25': str(pm25_val)})
                    # If noise comes with the same command, parse it here
            else:
                # If no data received, it might indicate a closed connection
                print("服务器断开或无数据接收。")
                set_connection_status(False)
                break
        except Exception as e:
            print(f"数据接收错误: {e}")
            set_connection_status(False)
            break
    print("接收线程退出。")

def send_commands():
    """Sends sensor query commands in a loop."""
    global stop_threads, sock
    taskIdx = 0
    while not stop_threads and socket_connection_status:
        time.sleep(1)
        if not sock:
            continue

        taskIdx += 1
        if taskIdx > 8: # Adjust based on the number of tasks you have
            taskIdx = 1

        try:
            if taskIdx == 1:
                sock.sendall(cmdPowerSensor)
            elif taskIdx == 2:
                sock.sendall(cmdLuxSensor)
            elif taskIdx == 3:
                sock.sendall(cmdTempSensor)
            elif taskIdx == 4:
                sock.sendall(cmdLidarSensor)
            elif taskIdx == 5:
                sock.sendall(cmdSmokeSensor)
            elif taskIdx == 6:
                sock.sendall(cmdFanSensor)
            elif taskIdx == 8:
                sock.sendall(cmdPM25Sensor)
        except Exception as e:
            print(f"发送命令错误: {e}")
            set_connection_status(False)
            break
    print("发送线程退出。")

def set_connection_status(status):
    """Updates the connection status and GUI button states."""
    global socket_connection_status
    socket_connection_status = status
    if status:
        status_label.config(text="连接状态: 已连接", fg="green")
        connect_button.config(state=tk.DISABLED)
        disconnect_button.config(state=tk.NORMAL)
    else:
        status_label.config(text="连接状态: 未连接", fg="red")
        connect_button.config(state=tk.NORMAL)
        disconnect_button.config(state=tk.DISABLED)
        # Attempt to close the socket if it's open
        global sock, receive_thread, stop_threads
        stop_threads = True
        if sock:
            try:
                sock.shutdown(socket.SHUT_RDWR)
                sock.close()
            except OSError as e:
                print(f"关闭socket错误: {e}")
            sock = None
        if receive_thread and receive_thread.is_alive():
            receive_thread.join(timeout=1) # Give thread a moment to finish
        print("所有线程已停止，Socket已关闭。")


def connect_to_server():
    """Establishes connection to the server."""
    global sock, receive_thread, stop_threads, server_ip, server_port, data_acquisition_success_count

    if socket_connection_status:
        messagebox.showinfo("提示", "已连接到服务器。")
        return

    server_ip = ip_entry.get()
    try:
        # Validate IP address format (basic check)
        socket.inet_aton(server_ip)
    except socket.error:
        messagebox.showerror("错误", "无效的IP地址格式。")
        return

    data_acquisition_success_count = 0 # Reset count on new connection attempt
    update_data_success_count()

    print(f"尝试连接到服务器: {server_ip}:{server_port}")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((server_ip, server_port))
        set_connection_status(True)
        messagebox.showinfo("连接成功", f"成功连接到 {server_ip}:{server_port}")

        stop_threads = False
        # Start receive thread
        receive_thread = threading.Thread(target=receive_data, args=(sock, lambda: not stop_threads))
        receive_thread.daemon = True # Allow program to exit even if thread is running
        receive_thread.start()

        # Start send thread (only if connected)
        threading.Thread(target=send_commands, daemon=True).start()

    except Exception as e:
        messagebox.showerror("连接失败", f"无法连接到服务器 {server_ip}:{server_port}。错误: {e}")
        set_connection_status(False)

def disconnect_from_server():
    """Disconnects from the server."""
    global stop_threads
    if not socket_connection_status:
        messagebox.showinfo("提示", "未连接到服务器。")
        return

    response = messagebox.askyesno("断开连接", "确定要断开与服务器的连接吗？")
    if response:
        stop_threads = True # Signal threads to stop
        set_connection_status(False)
        messagebox.showinfo("断开连接", "已尝试断开与服务器的连接。")


def create_gui():
    """Creates the main Tkinter GUI window."""
    global current_ip_var, data_success_count_var, ip_entry, status_label, connect_button, disconnect_button

    window = tk.Tk()
    window.title("传感器数据监控")
    window.geometry("500x600")

    # --- IP Address Configuration ---
    ip_frame = tk.LabelFrame(window, text="服务器IP设置", padx=10, pady=10)
    ip_frame.pack(pady=10, padx=10, fill="x")

    tk.Label(ip_frame, text="目标IP地址:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
    ip_entry = tk.Entry(ip_frame, width=20)
    ip_entry.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
    ip_entry.insert(0, server_ip) # Set initial IP

    connect_button = tk.Button(ip_frame, text="连接", command=connect_to_server)
    connect_button.grid(row=0, column=2, padx=5, pady=5)

    disconnect_button = tk.Button(ip_frame, text="断开连接", command=disconnect_from_server, state=tk.DISABLED)
    disconnect_button.grid(row=0, column=3, padx=5, pady=5)

    current_ip_var = tk.StringVar()
    current_ip_label = tk.Label(ip_frame, textvariable=current_ip_var)
    current_ip_label.grid(row=1, column=0, columnspan=4, padx=5, pady=5, sticky="w")
    update_gui_ip_display() # Display local IP on startup

    status_label = tk.Label(ip_frame, text="连接状态: 未连接", fg="red")
    status_label.grid(row=2, column=0, columnspan=4, padx=5, pady=5, sticky="w")

    # --- Data Acquisition Status ---
    status_frame = tk.LabelFrame(window, text="数据获取状态", padx=10, pady=10)
    status_frame.pack(pady=10, padx=10, fill="x")

    data_success_count_var = tk.StringVar()
    data_success_count_label = tk.Label(status_frame, textvariable=data_success_count_var)
    data_success_count_label.pack(padx=5, pady=5, anchor="w")
    update_data_success_count() # Initialize count display

    # --- Sensor Data Display ---
    sensor_display_frame = tk.LabelFrame(window, text="传感器数据", padx=10, pady=10)
    sensor_display_frame.pack(pady=10, padx=10, fill="both", expand=True)

    sensors = ["电压", "照度", "温度", "雷达", "烟雾", "风扇", "PM2.5"]
    for i, sensor in enumerate(sensors):
        sensor_display_vars[sensor] = tk.StringVar()
        sensor_display_vars[sensor].set(f"{sensor}: N/A")
        tk.Label(sensor_display_frame, textvariable=sensor_display_vars[sensor]).grid(row=i, column=0, padx=5, pady=2, sticky="w")

    window.protocol("WM_DELETE_WINDOW", on_closing)
    window.mainloop()

def on_closing():
    """Handles window closing event."""
    global stop_threads
    if messagebox.askokcancel("退出", "确定要退出程序吗？"):
        stop_threads = True # Signal all threads to stop
        # Give threads a moment to clean up
        time.sleep(0.5)
        sys.exit(0) # Force exit after threads have a chance to stop

if __name__ == '__main__':
    create_gui()