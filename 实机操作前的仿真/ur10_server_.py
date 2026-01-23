import socket
import threading
import json
import numpy as np
import binascii
import urx
import time
import math

class UR10eServer:
    def __init__(self, host='10.160.9.100', port=8001):

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建 TCP 套接字
        self.server_socket.bind((host, port))  # 绑定服务器地址和端口
        self.server_socket.listen(5)  # 开始监听连接
        print(f"Server listening on {host}:{port}")

        ####################ke####################
        self.ur = urx.Robot('10.160.9.21')  # 与UR连接
        # 机器人只要一启动就需运动到初始位
        print('机械臂运动到初始位')
        init_q = [1.46, 7.60, -91.51, -95.88, 358.55, -0.22]
        print(init_q)
        init_pos = [math.radians(angle) for angle in init_q]
        self.ur.move_q(init_pos, speed=0.8, acceleration=0.2)  # 这里机械臂速度可以慢一点

        brick_plaster_intpos = [30.28, -103.20, 113.48, 77.81, 91.11, 120.26]  # 砖面涂抹等待位 示教点位

        self.plaster_intpos_rad = [math.radians(angle) for angle in brick_plaster_intpos]
        self.plaster_pos_type = None
        time.sleep(0.1)
        # ####################ke####################
        self.start_server()  # 与Liu连接启动服务器

    def start_server(self):
        while True:
            client_socket, addr = self.server_socket.accept()  # 接受客户端连接
            print(f"Connection from {addr}")
            threading.Thread(target=self.handle_client, args=(client_socket, addr)).start()  # 创建新线程处理客户端连接

    def handle_client(self, client_socket, addr):
        with client_socket:
            while True:
                data = client_socket.recv(1024)  # 接收客户端数据
                if not data:
                    print(f"Connection closed from {addr}")
                    break

                print(f"Received data: {binascii.hexlify(data)}")  # 打印接收到的数据的十六进制表示
                if len(data) >= 2:
                    command = data[1:2]  # 获取命令字节

                    if command == b'\xa1':  # 接收墙面涂抹指令
                        print("Received command A1")
                        self.plaster_pos_type = 'A1'
                        # 到达涂抹开始位后走servoJ开始涂抹
                        threading.Thread(target=self.execute_and_respond,
                                         args=(client_socket, b'\xEE\x00\x02\x00\x00\x00\x00\xDD', 'AAA.csv')).start()
                    elif command == b'\xa2':
                        print("Received command A2")  # 接收抹泥面1涂抹指令
                        self.plaster_pos_type = 'A2'

                        # 机械臂走moveJ指令运动到涂抹等待位
                        self.ur.check_rtde_c_connection()
                        self.ur.move_q(self.plaster_intpos_rad, speed=0.8, acceleration=0.5)  # 走moveJ指令快速运动到墙面涂抹开始位置
                        threading.Thread(target=self.execute_and_respond,
                                         args=(
                                         client_socket, b'\xEE\x00\x02\x02\x00\x00\x00\xDD', 's8028face1.csv')).start()
                    elif command == b'\xa3':  # 接收抹泥面2涂抹指令
                        print("Received command A3")
                        self.plaster_pos_type = 'A3'
                        self.ur.check_rtde_c_connection()
                        # 机械臂走moveJ指令运动到涂抹等待位
                        self.ur.move_q(self.plaster_intpos_rad, speed=0.8, acceleration=0.5)  # 走moveJ指令快速运动到墙面涂抹开始位置
                        threading.Thread(target=self.execute_and_respond,
                                         args=(
                                         client_socket, b'\xEE\x00\x02\x02\x02\x00\x00\xDD', 's8028face2.csv')).start()
                    else:
                        print("Unsupported command or invalid data")
                        client_socket.sendall(b'\xEE\x00\x02\xFF\x00\x00\x00\xDD')  # 发送错误响应给客户端

    def execute_and_respond(self, client_socket, response, path):
        try:
            print('控制UR')
            path_data = self.read_path(path)  # 从文件读取路径数据
            self.execute_path(path_data)  # 执行路径
            # 机械臂走moveJ指令运动到涂抹等待位
            time.sleep(0.1)
            # 判断是否上述步骤执行完成
            client_socket.sendall(response)  # 发送成功响应给客户端 涂抹轨迹执行完成就可以发送PLC完成指令

            self.ur.check_rtde_c_connection()
            time.sleep(1)
            print('涂抹轨迹执行完成！')
            if self.plaster_pos_type == 'A1':
                ############这段轨迹需要示教，避免奇异##################
                print('从墙体涂抹结束点到砖侧面涂抹等待点！')

                #pos1 = [28.30, 8.49, -89.62, -98.90, 331.71, -0.10]
                #pos1_rad = [math.radians(angle) for angle in pos1]
                #self.ur.move_q(pos1_rad, speed=1.5, acceleration=1)  # 走moveJ指令
                #print('pos1')

                #pos2 = [48.87, -11.32, -20.26, -137.13, 220.32, 6.99]
                pos2 = [59.45, -5.28, -40.71, -128.07, 193.28, -23.02]
                pos2_rad = [math.radians(angle) for angle in pos2]
                self.ur.move_q(pos2_rad, speed=2.5, acceleration=2)  # 走moveJ指令
                print('pos2')

                #pos3 = [55.27, -64.07, 70.02, -106.98, 188.02, 74.50]
                #pos3_rad = [math.radians(angle) for angle in pos3]
                #self.ur.move_q(pos3_rad, speed=1.5, acceleration=1)  # 走moveJ指令
                #print('pos3')

                pos4 = [30.28, -103.20, 113.48, 77.81, 91.11, 120.26]
                pos4_rad = [math.radians(angle) for angle in pos4]
                self.ur.move_q(pos4_rad, speed=2.5, acceleration=2)  # 走moveJ指令
                print('pos4')

            self.ur.move_q(self.plaster_intpos_rad, speed=1.5, acceleration=0.6)  # 走moveJ指令快速运动到砖面涂抹等待位置
            self.ur.__del__() #此处需要断开URTD连接

        except Exception as e:
            print(f"Error executing path: {e}")
            # 如果出现错误，也要发送错误响应给客户端
            client_socket.sendall(b'\xEE\x00\x02\xFF\x00\x00\x00\xDD')

    def read_path(self, file_path):
        print('读取轨迹')
        q_path = np.loadtxt(file_path, delimiter=',', encoding='utf-8')
        return np.deg2rad(q_path)  # 将角度转换为弧度

    def execute_path(self, path):
        # 这里只执行servoJ指令
        q_command = path[0]
        self.ur.move_q(q_command, speed=0.08, acceleration=0.2)
        print('len:', len(path))
        for i in range(len(path)):
            print('i:', i)
            val = path[i]
            print(val)
            t_start = self.ur.get_initTime()  # 获取初始化时间
            print('t_start:', t_start)
            print('伺服轨迹控制')
            self.ur.servo_joint(val, a=0.2, v=0.2)  # 控制机器人关节运动
            if self.plaster_pos_type == 'A1':
                if i == 1:
                    print("开阀开泵")
                    self.ur.__del__()  # 断开urtd连接
                    time.sleep(0.1)
                    self.ur.set_digital_out(4, True)  # 开阀
                    time.sleep(0.1)
                    self.ur.set_analog_out(0, 0.26)
                    self.ur.set_digital_out(0, True)  # 开泵
                    self.ur.check_rtde_c_connection()  # 开启URTD连接
                    time.sleep(1)
                elif i == 1118:
                    print("关阀关泵")
                    self.ur.__del__()  # 断开URTD连接
                    time.sleep(1)
                    self.ur.set_digital_out(0, False)  # 关泵
                    time.sleep(0.5)
                    self.ur.set_digital_out(4, False)  # 关阀

                    if (self.ur.get_digital_in(0, True) == 0):
                        print('泵已经关闭')
                    else:
                        self.ur.set_digital_out(0, False)
                        time.sleep(1)
                        print('泵已经再次关闭')
                    self.ur.check_rtde_c_connection()  # 开启URTD连接
                    time.sleep(1)
                    print("停止ServoJ指令执行")
                    break
            elif self.plaster_pos_type == 'A2':
                if i == 5:
                    print("开阀开泵")
                    self.ur.__del__()  # 断开urtd连接
                    time.sleep(0.1)
                    self.ur.set_digital_out(4, True)  # 开阀
                    time.sleep(0.1)
                    self.ur.set_analog_out(0, 0.15)
                    self.ur.set_digital_out(0, True)  # 开泵
                    self.ur.check_rtde_c_connection()  # 开启URTD连接
                    time.sleep(0.1)
                elif i == 165:
                    print("关阀关泵")
                    self.ur.__del__()  # 断开URTD连接
                    time.sleep(1)
                    self.ur.set_digital_out(0, False)  # 关泵
                    time.sleep(0.5)
                    self.ur.set_digital_out(4, False)  # 关阀

                    if (self.ur.get_digital_in(0, True) == 0):
                        print('泵已经关闭')
                    else:
                        self.ur.set_digital_out(0, False)
                        time.sleep(2)
                        print('泵已经再次关闭')
                    self.ur.check_rtde_c_connection()  # 开启URTD连接
                    time.sleep(1)
                    print("停止ServoJ指令执行")
                    break
            else:
                if i == 5:
                    print("开阀开泵")
                    self.ur.__del__()  # 断开urtd连接
                    time.sleep(0.1)
                    self.ur.set_digital_out(4, True)  # 开阀
                    time.sleep(0.1)
                    self.ur.set_analog_out(0, 0.15)
                    self.ur.set_digital_out(0, True)  # 开泵
                    self.ur.check_rtde_c_connection()  # 开启URTD连接
                    time.sleep(0.1)
                elif i == 165:
                    print("关阀关泵")
                    self.ur.__del__()  # 断开URTD连接
                    time.sleep(1)
                    self.ur.set_digital_out(0, False)  # 关泵
                    time.sleep(0.5)
                    self.ur.set_digital_out(4, False)  # 关阀

                    if (self.ur.get_digital_in(0, True) == 0):
                        print('泵已经关闭')
                    else:
                        self.ur.set_digital_out(0, False)
                        time.sleep(2)
                        print('泵已经再次关闭')
                    self.ur.check_rtde_c_connection()  # 开启URTD连接
                    time.sleep(1)
                    print("停止ServoJ指令执行")
                    break

            q_command = path[i]  # 更新目标关节位置
            # timeStamp = self.ur.get_timeStamp()
            # q = self.ur.get_q()
            # print('UR关节位置为：',q)
            # if q==q_command:
            #     print('涂抹轨迹已执行')
            wait_duration = 0.016  # 单位是秒（机械臂运动速度，越小越快）
            time.sleep(wait_duration)


if __name__ == '__main__':
    UR10eServer()