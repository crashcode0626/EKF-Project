# BMS_Controller.py（兼容版）
import serial
import time
from datetime import datetime


class BMSController:
    def __init__(self, port):
        self.ser = serial.Serial(port=port, baudrate=115200, timeout=1)
        self.dsg_status = 0  # 放电管状态

    def open_discharge(self, retries=3):
        """开启放电管并验证状态"""
        cmd = bytes([0x01, 0x04, 0x55])  # 开启指令
        for attempt in range(1, retries + 1):
            self._send_command(cmd)
            time.sleep(0.5)
            if self._verify_dsg_status():
                return True
            print(f"开启尝试 {attempt}/{retries} 失败")
        return False

    def query_data(self):
        """查询完整BMS数据"""
        self._send_command(bytes([0x01, 0x01, 0x55]))
        time.sleep(0.2)
        return self._parse_response()

    def _send_command(self, command):
        """内部指令发送方法"""
        try:
            self.ser.write(command)
            print(f"[{datetime.now().strftime('%H:%M:%S.%f')}] 发送指令: {command.hex().upper()}")
        except Exception as e:
            print(f"发送失败: {str(e)}")
            raise

    def _parse_response(self):
        """解析响应数据"""
        try:
            raw = self.ser.read_all()
            decoded = raw.decode('utf-8', errors='replace').strip()
            if decoded.count(',') != 18:
                return None

            values = decoded.split(',')
            self.dsg_status = int(values[13])

            return {
                "SOC_0": float(values[0]),
                "SOC_1": float(values[1]),
                "SOC_2": float(values[2]),
                "SOC_6": float(values[3]),
                "SOC_7": float(values[4]),
                "SOC_8": float(values[5]),
                "Voltage_0": int(values[6]),
                "Voltage_1": int(values[7]),
                "Voltage_2": int(values[8]),
                "Voltage_6": int(values[9]),
                "Voltage_7": int(values[10]),
                "Voltage_8": int(values[11]),
                "Current": float(values[12]),
                "DSG_STA": self.dsg_status,
                "CHG_STA": int(values[14]),
                "OV_FLAG": int(values[15]),
                "UV_FLAG": int(values[16]),
                "Temp_1": float(values[17]),
                "Temp_2": float(values[18]),
            }
        except Exception as e:
            print(f"解析异常: {str(e)}")
            return None

    def _verify_dsg_status(self):
        """专用状态验证"""
        data = self.query_data()
        return data and data['DSG_STA'] == 1


# 保留原有函数接口以兼容旧代码
def open_dsg(ser):
    temp_controller = BMSController(ser.port)
    return temp_controller.open_discharge()


def send_info(ser):
    temp_controller = BMSController(ser.port)
    return temp_controller.query_data()


def parse_data(raw_data):
    # 此函数现在仅供外部兼容使用
    temp_controller = BMSController("COM1")  # 虚拟端口
    return temp_controller._parse_response(raw_data)
