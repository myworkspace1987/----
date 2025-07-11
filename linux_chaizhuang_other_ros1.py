import tkinter as tk
from tkinter import messagebox
import subprocess
import threading
import os
import time
import socket
import re

# Assume MCUReciver class is in the same file or imported from another file
# If it's in a separate file (e.g., mcu_receiver.py), you'd use:
# from mcu_receiver import MCUReciver

class MCUReciver:
    def __init__(self):
        self.server_ip_port = ('192.168.1.101', 8087) # Fixed IP for MCU
        self.mcu_socket = None  # Server socket
        self.client_connection = None # Client connection object
        self.mcu_connect_flag = False # Flag to indicate if a client MCU is connected
        self.rx_counter = 0

        self.pattern = re.compile(r"(\w+):(-?\d+\.?\d*)")

        # Variables to store received data, initialized to indicate no data yet
        self.CSpeed = 0
        self.previous_CSpeed = 0
        self.left_ultrasound = 0.0
        self.right_ultrasound = 0.0
        self.voltage = 0.0
        self.input_status = -1
        
        # Add a flag to indicate if the server socket has been successfully started
        self._server_started = False 

        print(f'>> 启动socket服务器在 {self.server_ip_port[0]}:{self.server_ip_port[1]}，等待MCU连接...')
        self.start_server_socket()

    def start_server_socket(self):
        """Initializes and starts the server socket in a separate thread."""
        try:
            self.mcu_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.mcu_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.mcu_socket.bind(self.server_ip_port)
            self.mcu_socket.listen(1)
            self._server_started = True # Server socket started successfully
            
            accept_thread = threading.Thread(target=self._accept_connections, daemon=True)
            accept_thread.start()

        except Exception as e:
            print(f">> 启动socket服务器失败: {e}")
            if self.mcu_socket:
                self.mcu_socket.close()
            self.mcu_socket = None
            self._server_started = False # Server socket failed to start

    def _accept_connections(self):
        """Continuously accepts incoming client connections."""
        while self._server_started: # Continue as long as the server is supposed to be running
            try:
                if self.mcu_connect_flag:
                    time.sleep(1)
                    continue

                print(">> 等待新的MCU连接...")
                client_conn, client_addr = self.mcu_socket.accept()
                self.client_connection = client_conn
                self.mcu_connect_flag = True
                print(f">> MCU 已连接: {client_addr[0]}:{client_addr[1]}")

                client_handler_thread = threading.Thread(target=self.link_handler, args=(self.client_connection, client_addr), daemon=True)
                client_handler_thread.start()

            except Exception as e:
                print(f">> 接受连接时发生错误: {e}")
                self.mcu_connect_flag = False
                if self.client_connection:
                    self.client_connection.close()
                    self.client_connection = None
                time.sleep(1)

    def link_handler(self, link, client_address):
        """Handles data reception and sending for an individual MCU client connection."""
        print(f">> 接收MCU[{client_address[0]}:{client_address[1]}]信息线程启动...")
        
        while self.mcu_connect_flag and self.client_connection == link:
            try:
                link.settimeout(1.0) 
                client_data_bytes = link.recv(512)
                
                if not client_data_bytes:
                    print(f">> MCU {client_address[0]}:{client_address[1]} 已断开连接。")
                    break

                client_data_str = client_data_bytes.decode()
                self.rx_counter += 1

                matches = self.pattern.findall(client_data_str)
                received_data = {key: float(value) for key, value in matches}
                
                if "CSpeed" in received_data:
                    self.previous_CSpeed = self.CSpeed
                    self.CSpeed = received_data["CSpeed"]
                
                self.left_ultrasound = received_data.get('LeftSonic', self.left_ultrasound)
                self.right_ultrasound = received_data.get('RightSonic', self.right_ultrasound)
                self.voltage = received_data.get('VOLT', self.voltage)
                self.input_status = int(received_data.get('INPUT', self.input_status))

                if self.rx_counter % 10 == 0:
                    print(f"[{time.strftime('%H:%M:%S')}] 从MCU接收: {client_data_str}")
                    print(f"   解析后数据: CSpeed={self.CSpeed}, L_US={self.left_ultrasound}, R_US={self.right_ultrasound}, VOLT={self.voltage}, INPUT={self.input_status}")
                    self.rx_counter = 0

            except socket.timeout:
                pass 
            except ConnectionResetError:
                print(f">> MCU {client_address[0]}:{client_address[1]} 连接被远程主机强行关闭。")
                break
            except Exception as e:
                print(f">> MCU {client_address[0]}:{client_address[1]} 数据处理或通信错误: {e}")
                break

        if self.client_connection == link:
            try:
                link.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            link.close()
            self.mcu_connect_flag = False
            self.client_connection = None
            print(f">> MCU {client_address[0]}:{client_address[1]} 连接处理线程结束，套接字已关闭。")

    def shutdown(self):
        """Cleanly shuts down the socket server and connections."""
        print(">> 正在关闭 MCU 连接器...")
        self._server_started = False # Signal accept thread to stop
        self.mcu_connect_flag = False
        if self.client_connection:
            try:
                self.client_connection.shutdown(socket.SHUT_RDWR)
                self.client_connection.close()
            except OSError as e:
                print(f"Error closing client connection: {e}")
            self.client_connection = None
        if self.mcu_socket:
            try:
                self.mcu_socket.shutdown(socket.SHUT_RDWR)
                self.mcu_socket.close()
            except OSError as e:
                print(f"Error closing server socket: {e}")
            self.mcu_socket = None
        print(">> MCU 连接器已关闭。")


class DeviceVerifierApp:
    def __init__(self, master):
        self.master = master
        master.title("设备启动验证工具")
        master.geometry("600x450+500+500") # Adjust window size for new content

        self.create_widgets()

        self.running_processes = {}
        
        # Initialize MCU Receiver
        self.mcu_receiver = MCUReciver()
        # Start periodic update for MCU data
        self.update_mcu_data()

    def create_widgets(self):
        # Camera verification section
        camera_frame = tk.LabelFrame(self.master, text="摄像头验证")
        camera_frame.pack(pady=5, padx=20, fill="x") # Reduced pady for more space

        self.camera_status_label = tk.Label(camera_frame, text="状态: 未验证", fg="gray")
        self.camera_status_label.pack(side=tk.LEFT, padx=10)

        self.camera_button = tk.Button(camera_frame, text="验证摄像头", command=self.verify_camera)
        self.camera_button.pack(side=tk.RIGHT, padx=10)

        # Imu verification section
        imu_frame = tk.LabelFrame(self.master, text="Imu验证")
        imu_frame.pack(pady=5, padx=20, fill="x")

        self.imu_status_label = tk.Label(imu_frame, text="状态: 未验证", fg="gray")
        self.imu_status_label.pack(side=tk.LEFT, padx=10)

        self.imu_button = tk.Button(imu_frame, text="验证Imu", command=self.verify_imu)
        self.imu_button.pack(side=tk.RIGHT, padx=10)

        # Lidar verification section
        lidar_frame = tk.LabelFrame(self.master, text="雷达验证")
        lidar_frame.pack(pady=5, padx=20, fill="x")

        self.lidar_status_label = tk.Label(lidar_frame, text="状态: 未验证", fg="gray")
        self.lidar_status_label.pack(side=tk.LEFT, padx=10)

        self.lidar_button = tk.Button(lidar_frame, text="验证雷达", command=self.verify_lidar)
        self.lidar_button.pack(side=tk.RIGHT, padx=10)

        # Audio verification section
        # audio_frame = tk.LabelFrame(self.master, text="语音验证")
        # audio_frame.pack(pady=5, padx=20, fill="x")

        # self.audio_status_label = tk.Label(audio_frame, text="状态: 未验证", fg="gray")
        # self.audio_status_label.pack(side=tk.LEFT, padx=10)

        # self.audio_button = tk.Button(audio_frame, text="播放语音", command=self.play_audio)
        # self.audio_button.pack(side=tk.RIGHT, padx=10)
        # 语音合成验证
        audio_tts_frame = tk.LabelFrame(self.master, text="语音合成验证")
        audio_tts_frame.pack(pady=5, padx=20, fill="x")
        self.audio_tts_status_label = tk.Label(audio_tts_frame, text="状态: 未验证", fg="gray")
        self.audio_tts_status_label.pack(side=tk.LEFT, padx=10)
        # 语言合成测试
        self.audio_tts_button = tk.Button(audio_tts_frame, text="语音合成", command=self.tts_verify)
        self.audio_tts_button.pack(side=tk.RIGHT, padx=10)

        # --- New MCU Data Display Section ---
        mcu_data_frame = tk.LabelFrame(self.master, text="MCU 数据")
        mcu_data_frame.pack(pady=10, padx=20, fill="x")

        # MCU Connection Status
        self.mcu_conn_status_label = tk.Label(mcu_data_frame, text="连接状态: 待连接", fg="gray")
        self.mcu_conn_status_label.pack(anchor="w", padx=10, pady=2) # Anchor west for left alignment

        # Left Ultrasound
        self.left_us_label = tk.Label(mcu_data_frame, text="左超声: N/A", fg="black")
        self.left_us_label.pack(anchor="w", padx=10, pady=2)

        # Right Ultrasound
        self.right_us_label = tk.Label(mcu_data_frame, text="右超声: N/A", fg="black")
        self.right_us_label.pack(anchor="w", padx=10, pady=2)

        # Voltage
        self.voltage_label = tk.Label(mcu_data_frame, text="电压: N/A", fg="black")
        self.voltage_label.pack(anchor="w", padx=10, pady=2)
        # Voltage
        self.status_label = tk.Label(mcu_data_frame, text="状态: N/A", fg="black")
        self.status_label.pack(anchor="w", padx=10, pady=2)
        # Exit button
        exit_button = tk.Button(self.master, text="退出", command=self.on_closing)
        exit_button.pack(pady=20)

    def update_mcu_data(self):
        """
        Periodically fetches and updates the MCU data displayed in the UI.
        """
        # Update connection status
        if self.mcu_receiver.mcu_connect_flag:
            self.mcu_conn_status_label.config(text="连接状态: 已连接", fg="green")
            # Update values if connected and data is available
            self.left_us_label.config(text=f"左超声: {self.mcu_receiver.left_ultrasound:.2f} cm")
            self.right_us_label.config(text=f"右超声: {self.mcu_receiver.right_ultrasound:.2f} cm")
            self.voltage_label.config(text=f"电压: {self.mcu_receiver.voltage:.2f} V")
            self.status_label.config(text=f"状态: {self.mcu_receiver.input_status}")
        else:
            self.mcu_conn_status_label.config(text=f"连接状态: 等待MCU连接到 {self.mcu_receiver.server_ip_port[0]}:{self.mcu_receiver.server_ip_port[1]}", fg="red")
            self.left_us_label.config(text="左超声: N/A")
            self.right_us_label.config(text="右超声: N/A")
            self.voltage_label.config(text="电压: N/A")
            self.status_label.config(text=f"状态: N/A")
        
        # Schedule the next update
        self.master.after(500, self.update_mcu_data) # Update every 500 milliseconds (0.5 seconds)

    def run_command_in_thread(self, command, success_message, error_message, status_label, process_key=None, check_output_func=None, cwd=None):
        """
        在单独的线程中运行 shell 命令。
        :param command: 要执行的命令列表或字符串。
        :param success_message: 成功时的消息。
        :param error_message: 失败时的消息。
        :param status_label: 更新状态的 Tkinter Label。
        :param process_key: 用于存储 Popen 对象的键。
        :param check_output_func: 一个回调函数，用于检查命令的输出。
        :param cwd: 命令执行的工作目录。
        """
        status_label.config(text="状态: 正在运行...", fg="blue")
        def target():
            try:
                process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, cwd=cwd)
                if process_key:
                    self.running_processes[process_key] = process

                # For commands that run in the background, we don't necessarily wait for a short time
                # For commands that are expected to complete, wait for a bit
                if not process_key: # If it's not a long-running process, wait for it
                    stdout, stderr = process.communicate()
                    if process.returncode == 0:
                        if check_output_func and not check_output_func(stdout):
                            status_label.config(text=f"状态: {error_message} (输出不符)", fg="red")
                        else:
                            status_label.config(text=f"状态: {success_message}", fg="green")
                    else:
                        status_label.config(text=f"状态: {error_message} (错误码: {process.returncode})\n{stderr[:100]}...", fg="red")
                else: # For long-running processes, just indicate it started
                    status_label.config(text=f"状态: {success_message} (后台运行)", fg="green")

            except FileNotFoundError:
                status_label.config(text=f"状态: {error_message} (命令未找到)", fg="red")
            except Exception as e:
                status_label.config(text=f"状态: {error_message} ({e})", fg="red")

        thread = threading.Thread(target=target)
        thread.daemon = True
        thread.start()

    def verify_camera(self):
        command = "roslaunch usb_cam usb_cam-test.launch"
        self.run_command_in_thread(
            command,
            "摄像头启动成功 (请查看新窗口)",
            "摄像头启动失败",
            self.camera_status_label,
            process_key="camera_launch"
        )
        messagebox.showinfo("提示", "摄像头ROS节点已尝试启动。请观察是否有新的摄像头窗口弹出。您可能需要手动关闭它。")

    def verify_imu(self):
        command_launch = "roslaunch wit_ros_imu rviz_and_imu.launch"
        self.run_command_in_thread(
            command_launch,
            "IMU启动中...",
            "IMU启动失败",
            self.imu_status_label,
            process_key="imu_launch"
        )
        
        def check_scan_topic():
            self.imu_status_label.config(text="状态: 检查 /imu 数据...", fg="blue")
            try:
                result = subprocess.run(
                    "rostopic echo -n 1 /wit/imu",
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=8
                )
                print(f"检查 /scan 数据输出: {result.stdout.strip()}")
                if result.returncode == 0 and "header:" in result.stdout:
                    self.imu_status_label.config(text="状态: 检测到 /imu 数据，Imu正常", fg="green")
                    messagebox.showinfo("提示", "Imu驱动已启动，检测到 /imu 数据输出。")
                else:
                    self.imu_status_label.config(text="状态: 未检测到 /imu 数据", fg="orange")
                    messagebox.showwarning("警告", "Imu驱动已尝试启动，但未检测到 /imu 数据输出，请手动检查。")
            except subprocess.TimeoutExpired:
                self.imu_status_label.config(text="状态: 检查超时，未检测到数据", fg="orange")
                messagebox.showwarning("警告", "检测 /imu 数据超时，请手动检查。")
            except Exception as e:
                self.imu_status_label.config(text=f"状态: 检查失败 ({e})", fg="red")
                messagebox.showerror("错误", f"检测 /imu 数据时发生错误：{e}")

        threading.Thread(target=check_scan_topic, daemon=True).start()
        messagebox.showinfo("提示", "Imu 驱动已尝试启动。请在新终端中运行 'rostopic echo /wit/imu' 检查是否有数据输出，或运行 'roslaunch wit_ros_imu rviz_and_imu.launch' 查看RVIZ图形。")
        self.imu_status_label.config(text="状态: 请手动验证数据...", fg="orange")

    def verify_lidar(self):
        command_launch = "roslaunch ydlidar_ros_driver TG.launch"
        self.run_command_in_thread(
            command_launch,
            "雷达驱动启动中...",
            "雷达驱动启动失败",
            self.lidar_status_label,
            process_key="lidar_launch"
        )
        
        def check_scan_topic():
            self.lidar_status_label.config(text="状态: 检查 /scan 数据...", fg="blue")
            try:
                result = subprocess.run(
                    "rostopic echo -n 1 /scan",
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=8
                )
                print(f"检查 /scan 数据输出: {result.stdout.strip()}")
                if result.returncode == 0 and "header:" in result.stdout:
                    self.lidar_status_label.config(text="状态: 检测到 /scan 数据，雷达正常", fg="green")
                    messagebox.showinfo("提示", "雷达驱动已启动，检测到 /scan 数据输出。")
                else:
                    self.lidar_status_label.config(text="状态: 未检测到 /scan 数据", fg="orange")
                    messagebox.showwarning("警告", "雷达驱动已尝试启动，但未检测到 /scan 数据输出，请手动检查。")
            except subprocess.TimeoutExpired:
                self.lidar_status_label.config(text="状态: 检查超时，未检测到数据", fg="orange")
                messagebox.showwarning("警告", "检测 /scan 数据超时，请手动检查。")
            except Exception as e:
                self.lidar_status_label.config(text=f"状态: 检查失败 ({e})", fg="red")
                messagebox.showerror("错误", f"检测 /scan 数据时发生错误：{e}")

        threading.Thread(target=check_scan_topic, daemon=True).start()
        messagebox.showinfo("提示", "雷达驱动已尝试启动。请在新终端中运行 'rostopic echo /scan' 检查是否有数据输出，或运行 'roslaunch ydlidar_ros_driver lidar_view.launch' 查看RVIZ图形。")
        self.lidar_status_label.config(text="状态: 请手动验证数据...", fg="orange")
        
    def tts_verify(self):
        # Define the directory where tts_offline_sample is located
        tts_dir = os.path.expanduser("~/voice_ws/Linux_xtts") # Using expanduser for ~

        # Command to run tts_offline_sample
        tts_command = "./tts_offline_sample 1 1.pcm"

        # Command to convert pcm to wav using ffmpeg
        ffmpeg_command = "ffmpeg -y -f s16le -ar 16000 -ac 1 -i 1.pcm 1.wav" # Added -y to overwrite output file

        def run_tts_and_convert():
            self.audio_tts_status_label.config(text="状态: 正在生成语音...", fg="blue")
            try:
                # 1. Run tts_offline_sample
                print(f"执行命令: cd {tts_dir} && {tts_command}")
                tts_process = subprocess.run(
                    tts_command,
                    shell=True,
                    cwd=tts_dir, # Set current working directory
                    capture_output=True, # Capture stdout and stderr
                    text=True,
                    check=True # Raise CalledProcessError if command returns non-zero exit status
                )
                print("tts_offline_sample stdout:\n", tts_process.stdout)
                print("tts_offline_sample stderr:\n", tts_process.stderr)

                # Check if 1.pcm was created (simple check, can be more robust)
                pcm_file_path = os.path.join(tts_dir, "1.pcm")
                if not os.path.exists(pcm_file_path) or os.path.getsize(pcm_file_path) == 0:
                    self.audio_tts_status_label.config(text="状态: 语音生成失败 (1.pcm 文件未生成或为空)", fg="red")
                    messagebox.showerror("错误", "语音合成失败：1.pcm 文件未生成或为空。")
                    return

                self.audio_tts_status_label.config(text="状态: 语音生成成功，正在转换...", fg="blue")

                # 2. Run ffmpeg to convert to WAV
                print(f"执行命令: cd {tts_dir} && {ffmpeg_command}")
                ffmpeg_process = subprocess.run(
                    ffmpeg_command,
                    shell=True,
                    cwd=tts_dir, # Set current working directory
                    capture_output=True,
                    text=True,
                    check=True
                )
                print("ffmpeg stdout:\n", ffmpeg_process.stdout)
                print("ffmpeg stderr:\n", ffmpeg_process.stderr)
                
                # Check if 1.wav was created
                wav_file_path = os.path.join(tts_dir, "1.wav")
                if os.path.exists(wav_file_path) and os.path.getsize(wav_file_path) > 0:
                    self.audio_tts_status_label.config(text="状态: 语音合成与转换成功", fg="green")
                    messagebox.showinfo("成功", "语音合成 (tts_offline_sample) 和转换 (ffmpeg) 均成功完成！")
                    # Optionally play the generated WAV file
                    self.play_generated_audio(wav_file_path)
                else:
                    self.audio_tts_status_label.config(text="状态: WAV 文件未生成或为空", fg="red")
                    messagebox.showerror("错误", "ffmpeg 转换失败：1.wav 文件未生成或为空。")

            except subprocess.CalledProcessError as e:
                self.audio_tts_status_label.config(text=f"状态: 命令执行失败 (错误码: {e.returncode})", fg="red")
                error_detail = f"stdout: {e.stdout}\nstderr: {e.stderr}"
                print(f"命令执行失败: {error_detail}")
                messagebox.showerror("错误", f"语音合成或转换命令执行失败。\n请检查终端输出获取更多信息。\n错误详情:\n{error_detail[:500]}...") # Limit message length
            except FileNotFoundError as e:
                self.audio_tts_status_label.config(text=f"状态: 命令未找到 ({e})", fg="red")
                messagebox.showerror("错误", f"所需命令未找到。请确保 '{e.filename}' 已安装并添加到 PATH 中。")
            except Exception as e:
                self.audio_tts_status_label.config(text=f"状态: 发生未知错误 ({e})", fg="red")
                messagebox.showerror("错误", f"发生未知错误：{e}")

        threading.Thread(target=run_tts_and_convert, daemon=True).start()

    def play_generated_audio(self, file_path):
        """Plays the generated WAV file."""
        if not os.path.exists(file_path):
            messagebox.showerror("错误", f"生成的音频文件不存在: {file_path}")
            return
        
        if os.name == 'posix':
            command = f"ffplay -nodisp -autoexit {file_path}"
        elif os.name == 'nt':
            command = f"start {file_path}"
        else:
            messagebox.showwarning("警告", "不支持的操作系统，无法自动播放生成的音频。请手动检查文件。")
            return
        
        try:
            subprocess.Popen(command, shell=True)
            print(f"正在播放生成的音频: {file_path}")
        except Exception as e:
            print(f"播放生成的音频失败: {e}")
            messagebox.showwarning("警告", f"无法自动播放生成的音频文件: {e}\n请手动检查文件: {file_path}")

    def play_audio(self):
        audio_file = "test_audio.wav"
        if not os.path.exists(audio_file):
            messagebox.showerror("错误", f"未找到音频文件: {audio_file}\n请确保文件存在于脚本同目录下。")
            self.audio_status_label.config(text="状态: 文件缺失", fg="red")
            return

        if os.name == 'posix':
            command = f"ffplay -nodisp -autoexit {audio_file}"
        elif os.name == 'nt':
            command = f"start {audio_file}"
        else:
            messagebox.showerror("错误", "不支持的操作系统，无法播放音频。")
            self.audio_status_label.config(text="状态: 不支持", fg="red")
            return

        def audio_playback_thread():
            self.audio_status_label.config(text="状态: 正在播放...", fg="blue")
            try:
                result = subprocess.run(command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                self.audio_status_label.config(text="状态: 语音播放成功", fg="green")
            except subprocess.CalledProcessError as e:
                self.audio_status_label.config(text=f"状态: 语音播放失败 ({e.returncode})\n{e.stderr[:100]}...", fg="red")
            except FileNotFoundError:
                self.audio_status_label.config(text="状态: 语音播放失败 (播放器未找到)", fg="red")
            except Exception as e:
                self.audio_status_label.config(text=f"状态: 语音播放失败 ({e})", fg="red")

        threading.Thread(target=audio_playback_thread).start()


    def on_closing(self):
        # Terminate all subprocesses started by this program
        for key, process in self.running_processes.items():
            if process.poll() is None:
                print(f"终止进程: {key} (PID: {process.pid})")
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    print(f"进程 {key} 未在规定时间内退出，强制终止...")
                    process.kill()
        
        # Shut down MCU receiver
        if self.mcu_receiver:
            self.mcu_receiver.shutdown()

        if messagebox.askokcancel("退出", "确定要退出吗？"):
            self.master.destroy()
    
if __name__ == "__main__":
    root = tk.Tk()
    app = DeviceVerifierApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()