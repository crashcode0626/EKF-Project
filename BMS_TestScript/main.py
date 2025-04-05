import time
import threading
import serial.tools.list_ports
from BMS_Controller import BMSController
from E_LoaderController import ElectronicLoadController
from Storage import DataLogger
#1.1--3
# 全局配置
PERIOD = 0.6  # 采集间隔（秒）
stop_flag = False
data_lock = threading.Lock()
shared_data = {'BMS': {}, 'Eload': {}}


def bms_worker(bms, collect_event, bms_ready):
    """BMS数据采集线程"""
    global stop_flag
    while not stop_flag:
        collect_event.wait()
        data = bms.query_data()
        if data:
            with data_lock:
                shared_data['BMS'] = data
                shared_data['timestamp'] = int(time.time() * 1000)
        bms_ready.set()  # 标记BMS数据准备就绪
        collect_event.clear()


def eload_worker(eload, collect_event, eload_ready):
    """电子负载数据采集线程"""

    global stop_flag
    while not stop_flag:
        collect_event.wait()
        data = eload.query()
        with data_lock:
            shared_data['Eload'] = {k: v['value'] for k, v in data.items()}
        eload_ready.set()  # 标记电子负载数据准备就绪2
        collect_event.clear()


def select_port(port_list, device_name):
    """串口选择交互"""
    print(f"\n可用的{device_name}端口：")
    for i, port in enumerate(port_list):
        print(f"{i + 1}. {port}")
    while True:
        try:
            choice = int(input("请选择端口号：")) - 1
            return port_list[choice]
        except (ValueError, IndexError):
            print("输入无效，请重新选择")


def setup_discharge_tasks():
    """设置放电任务"""
    tasks = []
    task_count = int(input("\n请输入要创建的放电任务数量："))
    for i in range(task_count):
        print(f"\n任务 {i + 1} 参数设置：")
        voltage = float(input("停止电压(V)："))
        current = float(input("放电电流(A)："))
        duration = int(input("放电时间(秒)："))
        tasks.append((voltage, current, duration))
    return tasks


def main():
    global stop_flag

    # 扫描并选择端口
    bms_ports = [p.device for p in serial.tools.list_ports.comports()]
    eload_ports = ElectronicLoadController.scan_ch340_ports()

    bms_port = select_port(bms_ports, "BMS")
    eload_port = select_port(eload_ports, "电子负载")

    # 初始化设备连接
    try:
        bms = BMSController(bms_port)
        with ElectronicLoadController(eload_port) as eload:
            # 开启放电管
            if not bms.open_discharge():
                print("错误：放电管开启失败！")
                return

            # 设置放电任务
            tasks = setup_discharge_tasks()
            logger = DataLogger()

            # 创建同步事件
            collect_event = threading.Event()
            bms_ready = threading.Event()
            eload_ready = threading.Event()

            # 启动采集线程
            threads = [
                threading.Thread(target=bms_worker, args=(bms, collect_event, bms_ready)),
                threading.Thread(target=eload_worker, args=(eload, collect_event, eload_ready))
            ]
            for t in threads:
                t.daemon = True
                t.start()

            # 执行放电任务
            for task_idx, (voltage, current, duration) in enumerate(tasks):
                print(f"\n开始执行任务 {task_idx + 1}/{len(tasks)}")

                # 第一个任务前清零参数
                if task_idx == 0:
                    eload.reset_parameters()
                    time.sleep(1)

                # 设置负载参数
                eload.set_current(current=current)
                eload.set_stop_voltage(voltage=voltage)
                eload.set_timer(seconds=duration)
                eload.set_load_switch(on=True)

                # 任务执行循环
                start_time = time.time()
                while (time.time() - start_time) < duration and not stop_flag:
                    # 重置准备状态
                    bms_ready.clear()
                    eload_ready.clear()

                    # 触发采集
                    collect_event.set()

                    # 等待数据准备
                    bms_ready.wait(timeout=1)
                    eload_ready.wait(timeout=1)

                    # 立即存储数据
                    with data_lock:
                        logger.log_data(shared_data)

                        # 检查停止条件
                        if not shared_data['BMS'].get('DSG_STA', 0):
                            stop_flag = True
                            break

                    # 保持最小采集间隔
                    elapsed = time.time() - start_time
                    remaining = PERIOD - (elapsed % PERIOD)
                    if remaining > 0:
                        time.sleep(remaining)

                #eload.set_load_switch(on=False)
                if stop_flag:
                    break

            # 清理
            stop_flag = True
            for t in threads:
                t.join()

    except Exception as e:
        print(f"发生错误：{str(e)}")
    finally:
        print("\n测试结束，数据已保存")


if __name__ == "__main__":
    main()