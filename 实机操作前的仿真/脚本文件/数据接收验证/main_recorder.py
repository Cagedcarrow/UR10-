import threading
import time
import os
# 确保你已经将之前的全量监控类保存为 ur_full_monitor.py
from ur_full_monitor import URFullMonitor


def run_recorder():
    """
    主轨迹记录逻辑
    """
    # 1. 配置机械臂 IP (根据你的环境设置为 10.160.9.21)
    robot_ip = '10.160.9.21'

    # 初始化全量监控类
    recorder = URFullMonitor(robot_ip=robot_ip)

    print("正在尝试连接到 UR10 机械臂...")
    if not recorder.connect():
        print(">>> 错误: 无法连接到机械臂。")
        print("请检查：\n1. 电脑 IP 是否设置为 10.160.9.x 网段\n2. 网线是否连接\n3. 机械臂是否已开机")
        return

    # 2. 设置文件名（以当前时间命名，避免覆盖之前的记录）
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"ur10_full_data_{timestamp}.csv"

    # 3. 创建线程停止标志
    stop_event = threading.Event()

    # 4. 准备录制线程
    record_thread = threading.Thread(
        target=recorder.start_logging,
        args=(filename, stop_event)
    )

    print("\n" + "=" * 40)
    print("UR10 全量数据记录系统已就绪")
    print(f"保存文件: {filename}")
    print("记录内容: 包含电流、电压、关节位姿、TCP速度等")
    print("=" * 40)

    input("\n请在示教器上准备好操作，按回车键 [Enter] 开始录制...")

    # 启动后台录制线程
    record_thread.start()

    print("\n>>> 正在录制中... (125Hz 采样频率)")
    input(">>> 完成操作后，按回车键 [Enter] 停止录制并保存数据...")

    # 触发停止标志并等待线程结束
    stop_event.set()
    record_thread.join()

    print("\n" + "=" * 40)
    print(f"录制完成！数据已安全保存至: {os.path.abspath(filename)}")
    print("=" * 40)


if __name__ == "__main__":
    run_recorder()