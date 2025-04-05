import serial
import serial.tools.list_ports
from time import sleep

# ---------------------------- 全局配置 ----------------------------
QUERY_CMD_MAP = {
    'switch_status': (0x10, 'bool'),
    'voltage': (0x11, 'voltage'),
    'current': (0x12, 'current'),
    'time': (0x13, 'time'),
    'AH': (0x14, 'capacity'),
    'WH': (0x15, 'capacity'),
    'temperature': (0x16, 'temperature'),
    'set_current': (0x17, 'current'),
    'stop_voltage': (0x18, 'voltage'),
    'timer': (0x19, 'time')
}


class ElectronicLoadController:
    def __init__(self, port, baudrate=9600, timeout=1):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.ser.close()

    @staticmethod
    def scan_ch340_ports():
        return [p.device for p in serial.tools.list_ports.comports() if 'CH340' in p.description]

    def _send_frame(self, address, cmd_code, data_bytes):
        frame = [0xA1, 0xA2, address, cmd_code] + data_bytes
        checksum = sum(frame) & 0xFF
        full_frame = bytes(frame + [checksum, 0x73, 0x74])
        self.ser.write(full_frame)
        return self.ser.read(10)

    # ==================== 设置指令 ====================
    def set_load_switch(self, address=0x00, on=True):
        data = [0x00, 0x00, 0x01] if on else [0x00, 0x00, 0x00]
        return self._send_frame(address, 0x01, data)

    def set_current(self, address=0x00, current=0.0):
        integer = int(current * 100)
        data = [0x00, (integer >> 8) & 0xFF, integer & 0xFF]
        return self._send_frame(address, 0x02, data)

    def set_stop_voltage(self, address=0x00, voltage=0.0):
        integer = int(voltage * 100)
        data = [0x00, (integer >> 8) & 0xFF, integer & 0xFF]
        return self._send_frame(address, 0x03, data)

    def set_timer(self, address=0x00, seconds=0):
        data = [
            (seconds >> 16) & 0xFF,
            (seconds >> 8) & 0xFF,
            seconds & 0xFF
        ]
        return self._send_frame(address, 0x04, data)

    # ==================== 新增 "参数清零" 功能 ====================
    def reset_parameters(self, address=0x00):
        """
        发送参数清零指令
        """
        data = [0x00, 0x00, 0x00]  # 参数清零指令没有额外数据
        return self._send_frame(address, 0x05, data)

    # ==================== 查询指令 ====================
    def query(self, address=0x00, param_name=None):
        results = {}
        query_items = QUERY_CMD_MAP.items() if not param_name else [(param_name, QUERY_CMD_MAP[param_name])]

        for name, (cmd_code, data_type) in query_items:
            try:
                response = self._send_frame(address, cmd_code, [0x00, 0x00, 0x00])
                if len(response) != 10:
                    results[name] = {'value': None, 'unit': '', 'status': '无响应'}
                    continue
                data_bytes = list(response[4:7])
                parsed = self._parse_data(data_bytes, data_type, cmd_code)
                parsed['raw'] = data_bytes
                results[name] = parsed
            except Exception as e:
                results[name] = {'value': None, 'unit': '', 'status': f'错误: {str(e)}'}

        return results if not param_name else results.get(param_name)

    def _parse_data(self, data_bytes, data_type, cmd_code):
        """数据解析器"""

        def check_empty(value, threshold):
            return value < threshold  # 根据数据类型设置不同阈值

        parsers = {
            'bool': lambda: {
                'value': bool(data_bytes[2]),
                'unit': '',
                'status': '正常'
            },
            'voltage': lambda: {
                'value': ((data_bytes[1] << 8) | data_bytes[2]) / 100.0,
                'unit': 'V',
                'status': '空载' if check_empty(((data_bytes[1] << 8) | data_bytes[2]), 10) else '正常'
            },
            'current': lambda: {
                'value': ((data_bytes[1] << 8) | data_bytes[2]) / 1000.0,
                'unit': 'A',
                'status': '空载' if check_empty(((data_bytes[1] << 8) | data_bytes[2]), 1) else '正常'
            },
            'time': lambda: {
                'value': (data_bytes[0] << 16) | (data_bytes[1] << 8) | data_bytes[2],
                'unit': 's',
                'status': '正常'
            },
            'capacity': lambda: {
                'value': (data_bytes[0] << 16 | data_bytes[1] << 8 | data_bytes[2]) / 1000.0,
                'unit': 'AH' if cmd_code == 0x14 else 'WH',
                'status': '正常'
            },
            'temperature': lambda: {
                'value': data_bytes[2],
                'unit': '°C',
                'status': '高温报警' if data_bytes[2] > 80 else '正常'
            }
        }
        return parsers[data_type]()


# ==================== 测试用例 ====================
if __name__ == "__main__":
    # 设备检测
    ports = ElectronicLoadController.scan_ch340_ports()
    if not ports:
        print("未检测到CH340设备，请检查连接")
        exit()

    with ElectronicLoadController(ports[0]) as eload:
        # 连接检查提示
        input("请确认已连接电子负载和电源，按回车键继续...")

        # 发送 "参数清零" 指令
        print("\n=== 执行参数清零 ===")
        eload.reset_parameters()
        sleep(1)  # 等待设备响应

        print("\n=== 初始化设置 ===")
        eload.set_load_switch(on=True)
        eload.set_current(current=5)
        sleep(1)
        eload.set_stop_voltage(voltage=5.7)
        sleep(1)
        eload.set_timer(seconds=3600)
        sleep(1)  # 等待设备响应

        # 综合查询
        print("\n=== 设备状态 ===")
        data = eload.query()

        # 核心参数显示
        print(f"[负载开关] {'已开启' if data['switch_status']['value'] else '已关闭'}")
        print(
            f"[设定电流] {data['set_current']['value']:.2f}{data['set_current']['unit']} ({data['set_current']['status']})")
        print(
            f"[停止电压] {data['stop_voltage']['value']:.3f}{data['stop_voltage']['unit']} ({data['stop_voltage']['status']})")
        print(
            f"[实时温度] {data['temperature']['value']}{data['temperature']['unit']} ({data['temperature']['status']})")

        # 调试信息
        print("\n=== 调试数据 ===")
        print(f"温度原始数据: {data['temperature']['raw']}")
        print(f"电流原始数据: {data['current']['raw']}")
        print(f"电压原始数据: {data['voltage']['raw']}")
