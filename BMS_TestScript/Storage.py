# Storage.py
import csv
from datetime import datetime
import threading


class DataLogger:
    _instance = None
    _initialized = False

    def __new__(cls):
        if not cls._instance:
            cls._instance = super(DataLogger, cls).__new__(cls)
            # 在程序启动时固定文件名
            cls._instance.filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
            cls._instance._file_lock = threading.Lock()
            cls._instance._init_header()
            cls._initialized = True
        return cls._instance

    def _init_header(self):
        """只会在首次初始化时执行"""
        if not self._initialized:
            with self._file_lock:
                with open(self.filename, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    header = [
                        'timestamp',
                        # BMS数据
                        'SOC_0', 'SOC_1', 'SOC_2', 'SOC_6', 'SOC_7', 'SOC_8',
                        'Voltage_0', 'Voltage_1', 'Voltage_2', 'Voltage_6',
                        'Voltage_7', 'Voltage_8', 'Current', 'DSG_STA', 'CHG_STA',
                        'OV_FLAG', 'UV_FLAG', 'Temp_1', 'Temp_2',
                        # 电子负载数据
                        'switch_status', 'voltage(V)', 'current(A)', 'time(s)',
                        'AH', 'WH', 'temperature', 'stop_voltage(V)', 'timer(s)'
                    ]
                    writer.writerow(header)

    def log_data(self, data):
        if not data['BMS'] or not data['Eload']:
            return

        row = [
            data.get('timestamp', 0),
            # BMS数据
            data['BMS'].get('SOC_0', 0),
            data['BMS'].get('SOC_1', 0),
            data['BMS'].get('SOC_2', 0),
            data['BMS'].get('SOC_6', 0),
            data['BMS'].get('SOC_7', 0),
            data['BMS'].get('SOC_8', 0),
            data['BMS'].get('Voltage_0', 0),
            data['BMS'].get('Voltage_1', 0),
            data['BMS'].get('Voltage_2', 0),
            data['BMS'].get('Voltage_6', 0),
            data['BMS'].get('Voltage_7', 0),
            data['BMS'].get('Voltage_8', 0),
            data['BMS'].get('Current', 0),
            data['BMS'].get('DSG_STA', 0),
            data['BMS'].get('CHG_STA', 0),
            data['BMS'].get('OV_FLAG', 0),
            data['BMS'].get('UV_FLAG', 0),
            data['BMS'].get('Temp_1', 0),
            data['BMS'].get('Temp_2', 0),
            # 电子负载数据
            data['Eload'].get('switch_status', False),
            data['Eload'].get('voltage', 0),
            data['Eload'].get('current', 0),
            data['Eload'].get('time', 0),
            data['Eload'].get('AH', 0),
            data['Eload'].get('WH', 0),
            data['Eload'].get('temperature', 0),
            data['Eload'].get('stop_voltage', 0),
            data['Eload'].get('timer', 0)
        ]

        with self._file_lock:
            with open(self.filename, 'a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(row)