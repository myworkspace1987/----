import tkinter as tk
from tkinter import scrolledtext, messagebox
import subprocess
import threading
import os
import signal
import re
import time

class RosNodeController:
    """
    管理ROS2节点启动和停止以及话题回显。
    在MVC类似结构中充当模型（Model）。
    """
    def __init__(self):
        self.processes = {}  # 存储每个已启动节点的Popen对象
        self.node_commands = {
            "ipc_server": "ros2 launch ipc_server ipc_server.launch.py",
            "odom_pub": "ros2 launch odom_pub wheel_odom.launch.py",
            "wit_imu": "ros2 launch wit_imu wit_imu.launch.py",
            "ydlidar_driver": "ros2 launch ydlidar_driver ydlidar_launch.py"
        }
        self.topic_commands = {
            "robot_info": "ros2 topic echo /robot_info --once", # --once 获取一条消息
            "odom_data": "ros2 topic echo /odom",
            "imu_data": "ros2 topic echo /imu/data",
            "lidar_data": "ros2 topic echo /scan" # 常见的激光雷达话题，如果不同请调整
        }
        self.stop_flags = {} # 用于通知连续话题回显线程停止
        self.robot_info_update_id = None # 用于管理机器人信息定时更新

    def launch_node(self, node_name):
        """在单独的进程中启动一个ROS2节点。"""
        if node_name in self.processes and self.processes[node_name].poll() is None:
            return True, f"{node_name} 已经运行。"

        command = self.node_commands.get(node_name)
        if not command:
            return False, f"未知节点: {node_name}"

        try:
            # 使用 preexec_fn=os.setsid 创建一个新的进程组
            # 这允许稍后终止进程及其子进程
            process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid,
                                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) # 重定向输出以抑制控制台杂乱
            self.processes[node_name] = process
            print(f"已启动 {node_name}，PID: {process.pid}")
            # 给它一点时间启动
            time.sleep(1)
            # 检查进程是否仍在运行（即没有立即崩溃）
            if process.poll() is not None:
                return False, f"启动 {node_name} 失败。进程立即退出。"

            return True, f"成功启动 {node_name}。"
        except Exception as e:
            return False, f"启动 {node_name} 时出错: {e}"

    def stop_node(self, node_name):
        """停止一个已启动的ROS2节点。"""
        if node_name in self.processes and self.processes[node_name].poll() is None:
            try:
                # 终止进程组以确保所有子进程都被终止
                os.killpg(os.getpgid(self.processes[node_name].pid), signal.SIGTERM)
                self.processes[node_name].wait(timeout=5) # 等待终止
                del self.processes[node_name]
                print(f"已停止 {node_name}。")
                return True, f"成功停止 {node_name}。"
            except Exception as e:
                return False, f"停止 {node_name} 时出错: {e}"
        return False, f"{node_name} 未运行或已停止。"

    def get_topic_data_once(self, topic_key, timeout=5):
        """
        运行 ros2 topic echo 获取一条消息并返回数据。
        返回一个元组: (成功布尔值, 数据字符串或错误消息)
        """
        command = self.topic_commands.get(topic_key)
        if not command:
            return False, f"未知话题: {topic_key}"

        # 如果命令中没有 --once，则调整为添加 --once，用于特定检查
        if " --once" not in command:
            command_once = command + " --once"
        else:
            command_once = command

        try:
            result = subprocess.run(command_once, shell=True, capture_output=True, text=True, timeout=timeout)
            if result.returncode == 0 and result.stdout.strip():
                return True, result.stdout
            else:
                return False, f"在 {timeout} 秒内未在 {self.topic_commands.get(topic_key).split(' ')[-1]} 上接收到数据，或命令失败。标准错误: {result.stderr.strip()}"
        except subprocess.TimeoutExpired:
            return False, f"超时: 在 {timeout} 秒内未在 {self.topic_commands.get(topic_key).split(' ')[-1]} 上接收到数据。节点是否正在发布？"
        except Exception as e:
            return False, f"检查 {self.topic_commands.get(topic_key).split(' ')[-1]} 时出错: {e}"

    def start_continuous_topic_echo(self, topic_key, output_widget=None, update_callback=None):
        """
        在单独的线程中启动连续的 ros2 topic echo。
        """
        command = self.topic_commands.get(topic_key)
        if not command:
            return "未知话题: {topic_key}"

        # 首先停止该话题的任何现有回显
        self.stop_topic_echo(topic_key)

        self.stop_flags[topic_key] = True # 设置标志以允许运行
        threading.Thread(target=self._continuous_topic_echo_thread,
                         args=(command, output_widget, update_callback, topic_key),
                         daemon=True).start()
        return f"已开始连续回显 {topic_key}。"

    def _continuous_topic_echo_thread(self, command, output_widget, update_callback, topic_key):
        """连续话题回显的线程函数。"""
        process = None
        try:
            process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE,
                                         stderr=subprocess.PIPE, text=True, bufsize=1)
            for line in iter(process.stdout.readline, ''):
                if not self.stop_flags.get(topic_key, False): # 检查停止标志
                    break
                if update_callback:
                    update_callback(line) # 通过回调更新GUI
                else:
                    # 对于只接受文本的控件的备用方案
                    if output_widget:
                        output_widget.insert(tk.END, line)
                        output_widget.see(tk.END)
            # 如果进程意外退出，读取标准错误
            stderr_output = process.stderr.read()
            if stderr_output and output_widget:
                output_widget.insert(tk.END, f"来自 {topic_key} 回显的错误: {stderr_output}\n")
                output_widget.see(tk.END)

        except Exception as e:
            if output_widget:
                output_widget.insert(tk.END, f"连续回显 {topic_key} 时出错: {e}\n")
                output_widget.see(tk.END)
        finally:
            if process and process.poll() is None:
                process.terminate()
                process.wait(timeout=1)
            self.stop_flags[topic_key] = False # 确保退出时重置标志
            print(f"已停止连续回显 {topic_key}。")


    def stop_topic_echo(self, topic_key):
        """通知连续话题回显线程停止。"""
        if topic_key in self.stop_flags and self.stop_flags[topic_key]:
            self.stop_flags[topic_key] = False
            return f"已发送信号停止 {topic_key} 回显。"
        return f"{topic_key} 回显未激活。"


    def _parse_robot_info(self, output):
        """将 robot_info 字符串输出解析为字典。"""
        data = {}
        lines = output.split('\n')
        for line in lines:
            if ':' in line:
                key_val = line.split(':', 1)
                key = key_val[0].strip().replace('-', '_') # 将连字符替换为有效的Python键
                value = key_val[1].strip()
                data[key] = value
        return data

    def cleanup(self):
        """确保在应用程序退出时终止所有已启动的进程。"""
        # 首先停止所有连续话题回显
        for topic_name in list(self.stop_flags.keys()):
            self.stop_flags[topic_name] = False
        time.sleep(0.5) # 给线程一点时间响应

        for node_name, process in list(self.processes.items()):
            if process.poll() is None: # 如果仍在运行
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    process.wait(timeout=1)
                    print(f"已清理 {node_name}。")
                except Exception as e:
                    print(f"清理 {node_name} 时出错: {e}")
        self.processes.clear()


class RosMonitorApp:
    """
    用于监视ROS2节点的Tkinter GUI。
    充当视图（View）和控制器（Controller）。
    """
    def __init__(self, master):
        self.master = master
        master.title("ROS2 节点监视器")
        master.geometry("800x700")

        self.node_controller = RosNodeController()

        # 机器人信息显示
        self.robot_info_frame = tk.LabelFrame(master, text="机器人信息 (IPC 服务器)", padx=10, pady=10)
        self.robot_info_frame.pack(pady=10, padx=10, fill=tk.X)

        self.robot_info_labels = {}
        info_keys = ["left_wheel_rpm", "right_wheel_rpm", "center_rpm", "is_moving",
                     "left_sonar", "right_sonar", "battery_volt"]
        for i, key in enumerate(info_keys):
            row = i // 2
            col = (i % 2) * 2
            tk.Label(self.robot_info_frame, text=f"{key.replace('_', ' ').title()}:").grid(row=row, column=col, sticky="w", padx=5, pady=2)
            label = tk.Label(self.robot_info_frame, text="N/A", width=15, anchor="w", fg="blue")
            label.grid(row=row, column=col+1, sticky="w", padx=5, pady=2)
            self.robot_info_labels[key] = label

        # 节点控制按钮
        self.control_frame = tk.LabelFrame(master, text="节点控制", padx=10, pady=10)
        self.control_frame.pack(pady=10, padx=10, fill=tk.X)

        self.nodes = {
            "ipc_server": {"button": None, "state_label": None},
            "odom_pub": {"button": None, "state_label": None, "topic_key": "odom_data"},
            "wit_imu": {"button": None, "state_label": None, "topic_key": "imu_data"},
            "ydlidar_driver": {"button": None, "state_label": None, "topic_key": "lidar_data"}
        }

        for i, (node_name, node_info) in enumerate(self.nodes.items()):
            row = i // 2
            col = (i % 2) * 3 # 状态和话题检查按钮的调整列
            node_display_name = node_name.replace('_', ' ').title()

            tk.Label(self.control_frame, text=f"{node_display_name}:").grid(row=row, column=col, sticky="w", padx=5, pady=5)

            btn = tk.Button(self.control_frame, text=f"启动 {node_display_name}",
                            command=lambda n=node_name: self.toggle_node(n))
            btn.grid(row=row, column=col+1, sticky="ew", padx=5, pady=5)
            self.nodes[node_name]["button"] = btn

            state_label = tk.Label(self.control_frame, text="已停止", fg="red")
            state_label.grid(row=row, column=col+2, sticky="w", padx=5, pady=5)
            self.nodes[node_name]["state_label"] = state_label

        # 话题回显显示
        self.topic_echo_frame = tk.LabelFrame(master, text="话题回显输出", padx=10, pady=10)
        self.topic_echo_frame.pack(pady=10, padx=10, fill=tk.BOTH, expand=True)

        self.topic_selector_label = tk.Label(self.topic_echo_frame, text="选择要连续回显的话题:")
        self.topic_selector_label.pack(side=tk.TOP, anchor="w", padx=5, pady=2)

        self.topic_options_for_echo = {
            "odom_data": "/odom",
            "imu_data": "/imu/data",
            "lidar_data": "/scan"
        }
        self.selected_topic_for_echo = tk.StringVar(master)
        self.selected_topic_for_echo.set(list(self.topic_options_for_echo.keys())[0]) # 默认值

        self.topic_menu_for_echo = tk.OptionMenu(self.topic_echo_frame, self.selected_topic_for_echo, *self.topic_options_for_echo.keys())
        self.topic_menu_for_echo.pack(side=tk.TOP, anchor="w", padx=5, pady=2)

        self.echo_button = tk.Button(self.topic_echo_frame, text="start echo", command=self.toggle_topic_echo)
        self.echo_button.pack(side=tk.TOP, anchor="w", padx=5, pady=5)

        self.echo_output = scrolledtext.ScrolledText(self.topic_echo_frame, wrap=tk.WORD, width=70, height=15)
        self.echo_output.pack(padx=5, pady=5, fill=tk.BOTH, expand=True)

        # 初始启动IPC服务器（按要求）
        self.launch_ipc_on_start()

        # 关闭窗口的协议
        master.protocol("WM_DELETE_WINDOW", self.on_closing)

    def launch_ipc_on_start(self):
        """在应用程序启动时自动启动IPC服务器。"""
        success, message = self.node_controller.launch_node("ipc_server")
        messagebox.showinfo("IPC 服务器启动", message)
        self.update_node_status("ipc_server")
        if success:
            # IPC服务器启动后自动开始连续检查 robot_info
            self.schedule_robot_info_update()

    def schedule_robot_info_update(self):
        """安排 robot_info 每秒更新一次。"""
        self.update_robot_info()
        self.node_controller.robot_info_update_id = self.master.after(1000, self.schedule_robot_info_update) # 安排下一次更新

    def update_robot_info(self):
        """获取并显示 robot_info。"""
        def _update_gui_labels(data):
            # 清除之前的信息并最初设置为N/A
            for label in self.robot_info_labels.values():
                label.config(text="N/A", fg="blue")
            # 如果有新数据，则更新
            parsed_data = self.node_controller._parse_robot_info(data) # 使用内部解析器
            for key, value in parsed_data.items():
                if key in self.robot_info_labels:
                    self.robot_info_labels[key].config(text=str(value), fg="black")
                # else: print(f"未知 robot_info 键: {key}") # 用于调试新键

        # 在单独的线程中获取数据以防止GUI冻结
        threading.Thread(target=self._fetch_robot_info_thread, args=(_update_gui_labels,), daemon=True).start()

    def _fetch_robot_info_thread(self, update_callback):
        """获取 robot_info 数据的线程。"""
        success, output = self.node_controller.get_topic_data_once("robot_info", timeout=3) # 更短的超时时间用于快速更新
        if success:
            self.master.after(0, lambda: update_callback(output)) # 在主线程上更新GUI
        else:
            # print(f"获取 robot_info 失败: {output}") # 用于调试
            # 可选地，如果未收到数据，则将所有标签更新为“无数据”
            self.master.after(0, lambda: [label.config(text="无数据", fg="red") for label in self.robot_info_labels.values()])


    def toggle_node(self, node_name):
        """根据其当前状态启动或停止ROS2节点。"""
        current_button_text = self.nodes[node_name]["button"].cget("text")
        node_display_name = node_name.replace('_', ' ').title()

        if "启动" in current_button_text:
            success, message = self.node_controller.launch_node(node_name)
            if node_name == "wit_imu" and not success:
                messagebox.showerror("启动错误", f"启动 {node_display_name} 失败:\n{message}")
            else:
                messagebox.showinfo("节点控制", message)

            if success:
                self.nodes[node_name]["button"].config(text=f"停止 {node_display_name}")
                self.nodes[node_name]["state_label"].config(text="运行中", fg="green")
                # 对于 odom, imu, lidar，检查话题数据是否可用
                if node_name in ["odom_pub", "wit_imu", "ydlidar_driver"]:
                    topic_key = self.nodes[node_name]["topic_key"]
                    threading.Thread(target=self._check_topic_after_launch,
                                     args=(node_name, topic_key), daemon=True).start()
            else:
                # 如果启动失败，确保按钮和标签反映停止状态
                self.nodes[node_name]["button"].config(text=f"启动 {node_display_name}")
                self.nodes[node_name]["state_label"].config(text="已停止", fg="red")

        else: # 如果按钮显示“停止”
            success, message = self.node_controller.stop_node(node_name)
            messagebox.showinfo("节点控制", message)
            if success:
                self.nodes[node_name]["button"].config(text=f"启动 {node_display_name}")
                self.nodes[node_name]["state_label"].config(text="已停止", fg="red")

    def _check_topic_after_launch(self, node_name, topic_key):
        """检查节点启动后特定话题是否正在发布数据。"""
        time.sleep(2) # 给节点一些时间开始发布
        success, data_or_error = self.node_controller.get_topic_data_once(topic_key, timeout=5) # 第一次数据的5秒超时
        topic_name = self.node_controller.topic_commands.get(topic_key).split(' ')[-1] # 提取话题名称

        if not success:
            self.master.after(0, lambda: messagebox.showwarning(
                "话题数据警告",
                f"节点 '{node_name.replace('_', ' ').title()}' 已启动，但在话题 '{topic_name}' 上未收到数据！\n"
                f"原因: {data_or_error}"
            ))
        else:
            self.master.after(0, lambda: messagebox.showinfo(
                "话题数据检查",
                f"节点 '{node_name.replace('_', ' ').title()}' 已启动，并成功在话题 '{topic_name}' 上收到数据！"
            ))


    def update_node_status(self, node_name):
        """更新给定节点的状态标签。"""
        if node_name in self.node_controller.processes and \
           self.node_controller.processes[node_name].poll() is None:
            self.nodes[node_name]["state_label"].config(text="运行中", fg="green")
            self.nodes[node_name]["button"].config(text=f"停止 {node_name.replace('_', ' ').title()}")
        else:
            self.nodes[node_name]["state_label"].config(text="已停止", fg="red")
            self.nodes[node_name]["button"].config(text=f"启动 {node_name.replace('_', ' ').title()}")

    def toggle_topic_echo(self):
        """启动或停止回显选定的话题。"""
        topic_key = self.selected_topic_for_echo.get()
        current_button_text = self.echo_button.cget("text")

        if "开始回显" in current_button_text:
            self.echo_output.delete(1.0, tk.END) # 清除之前的输出
            self.echo_button.config(text="stop echo")
            def _update_echo_output(data):
                self.echo_output.insert(tk.END, data) # 数据已包含换行符
                self.echo_output.see(tk.END)
            self.node_controller.start_continuous_topic_echo(topic_key, update_callback=_update_echo_output)
        else:
            status = self.node_controller.stop_topic_echo(topic_key)
            messagebox.showinfo("topic ehco control", status)
            self.echo_button.config(text="Get echo data")


    def on_closing(self):
        """处理窗口关闭时的正常关闭。"""
        if messagebox.askokcancel("退出", "是否要退出并停止所有正在运行的ROS2节点？"):
            if self.node_controller.robot_info_update_id:
                self.master.after_cancel(self.node_controller.robot_info_update_id) # 取消定时更新
            self.node_controller.cleanup()
            self.master.destroy()


def main():
    root = tk.Tk()
    app = RosMonitorApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()