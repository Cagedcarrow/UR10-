import socket
import struct
import csv
import time
import threading


class URFullMonitor:
    def __init__(self, robot_ip='10.160.9.21', port=30003):
        self.robot_ip = robot_ip
        self.port = port
        self.socket = None

        # 定义全量数据表头 (共 55 个字段)
        self.header = [
            'Time',
            # 实际关节数据 (6个)
            'Act_q0', 'Act_q1', 'Act_q2', 'Act_q3', 'Act_q4', 'Act_q5',
            # 实际关节速度 (6个)
            'Act_qd0', 'Act_qd1', 'Act_qd2', 'Act_qd3', 'Act_qd4', 'Act_qd5',
            # 实际关节电流 (6个)
            'Act_I0', 'Act_I1', 'Act_I2', 'Act_I3', 'Act_I4', 'Act_I5',
            # 实际末端位姿 (6个)
            'Act_X', 'Act_Y', 'Act_Z', 'Act_RX', 'Act_RY', 'Act_RZ',
            # 实际末端速度 (6个)
            'Act_dX', 'Act_dY', 'Act_dZ', 'Act_dRX', 'Act_dRY', 'Act_dRZ',
            # 目标关节位置 (6个)
            'Tgt_q0', 'Tgt_q1', 'Tgt_q2', 'Tgt_q3', 'Tgt_q4', 'Tgt_q5',
            # 控制模式与状态
            'Robot_Mode', 'Joint_Mode0', 'Joint_Mode1', 'Joint_Mode2',
            'Joint_Mode3', 'Joint_Mode4', 'Joint_Mode5',
            'Safety_Mode', 'Main_Voltage', 'Robot_Voltage', 'Robot_Current'
        ]

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5)
            self.socket.connect((self.robot_ip, self.port))
            print(f"成功连接到 UR10 全量数据流: {self.robot_ip}")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def parse_packet(self, data):
        """
        解析 UR CB3 30003 端口的 1060 字节数据包
        """
        try:
            # 1. Robot Time (Offset 4)
            res = [struct.unpack('!d', data[4:12])[0]]

            # 2. Actual Joint Positions (Offset 252)
            res += list(struct.unpack('!6d', data[252:300]))

            # 3. Actual Joint Speeds (Offset 300)
            res += list(struct.unpack('!6d', data[300:348]))

            # 4. Actual Joint Currents (Offset 348)
            res += list(struct.unpack('!6d', data[348:396]))

            # 5. Actual TCP Pose (Offset 444)
            res += list(struct.unpack('!6d', data[444:492]))

            # 6. Actual TCP Speed (Offset 492)
            res += list(struct.unpack('!6d', data[492:540]))

            # 7. Target Joint Positions (Offset 12)
            res += list(struct.unpack('!6d', data[12:60]))

            # 8. Robot Mode (Offset 756)
            res.append(struct.unpack('!d', data[756:764])[0])

            # 9. Joint Modes (Offset 764)
            res += list(struct.unpack('!6d', data[764:812]))

            # 10. Safety Mode (Offset 812), Main Voltage (Offset 780), etc.
            res.append(struct.unpack('!d', data[812:820])[0])  # Safety Mode
            res.append(struct.unpack('!d', data[780:788])[0])  # Main Voltage
            res.append(struct.unpack('!d', data[788:796])[0])  # Robot Voltage
            res.append(struct.unpack('!d', data[796:804])[0])  # Robot Current

            return res
        except Exception as e:
            print(f"数据解析解析出错: {e}")
            return None

    def start_logging(self, filename, stop_event):
        print(f"开始记录全量数据到 {filename}...")
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self.header)

            while not stop_event.is_set():
                try:
                    # 确保读取完整的包头以获取长度
                    header_data = self.socket.recv(4)
                    if len(header_data) < 4: continue
                    packet_len = struct.unpack('!i', header_data)[0]

                    # 循环接收直到满一个包的长度
                    body_data = b''
                    while len(body_data) < (packet_len - 4):
                        chunk = self.socket.recv(packet_len - 4 - len(body_data))
                        if not chunk: break
                        body_data += chunk

                    full_packet = header_data + body_data

                    # CB3 标准包长通常为 1060
                    if len(full_packet) >= 1060:
                        row = self.parse_packet(full_packet)
                        if row:
                            writer.writerow(row)
                except Exception as e:
                    print(f"读取中断: {e}")
                    break
        print("录制已停止。")


if __name__ == "__main__":
    # 使用示例
    monitor = URFullMonitor(robot_ip='10.160.9.21')
    if monitor.connect():
        stop_flag = threading.Event()
        filename = f"ur10_full_data_{int(time.time())}.csv"

        t = threading.Thread(target=monitor.start_logging, args=(filename, stop_flag))
        t.start()

        input("正在录制全量数据，按回车键停止...\n")
        stop_flag.set()
        t.join()