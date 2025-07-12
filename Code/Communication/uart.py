from machine import UART

class myUART:
    def __init__(self, uart_id: int, baudrate: int = 115200) -> None:
        """
        初始化UART通信
        :param uart_id: UART端口ID
        :param baudrate: 波特率
        """
        self.uart = UART(uart_id)
        self.uart.init(baudrate)  # 使用传入的波特率而不是硬编码的115200
        self.uart_id = uart_id
        self.baudrate = baudrate

    def read(self) -> bytes:
        """
        读取UART数据
        :return: 接收到的数据，如果没有数据则返回空字节串
        """
        buf_len = self.uart.any()
        if buf_len:
            buf = self.uart.read(buf_len)
            return buf
        return ""  # 返回空字节串当没有数据时

    def writeln(self, data: str) -> None:
        """
        写入数据并添加换行符
        :param data: 要写入的字符串数据
        """
        self.uart.write(data + "\n")
    
    def write_str(self, data: str, end : str = '\n') -> None:
        """
        写入字符串数据
        :param data: 要写入的字符串数据
        """
        self.uart.write(data + end)

    def write_bytes(self, data: bytes) -> None:
        """
        写入字节数据
        :param data: 要写入的字节数据
        """
        self.uart.write(data)

    def available(self) -> int:
        """
        检查可用的字节数
        :return: 可用的字节数
        """
        return self.uart.any()

    def flush(self) -> None:
        """
        刷新UART缓冲区
        """
        # MicroPython UART可能没有flush方法，这里留空
        pass
    
def printf(uart: myUART, buffer: str, print_console: bool = False, end : str = '\n') -> None:
    """
    通过UART输出字符串
    :param uart: myUART实例
    :param buffer: 要输出的字符串
    :param print_console: 是否打印到控制台，默认False
    """
    if print_console:
        print("printf:", buffer)  # 打印到控制台
    uart.write_str(buffer)
    
# 解析传入pid参数
def parse_pid_params(input_str: str):
    """
    解析PID参数字符串
    
    :param input_str: 输入字符串，格式为"Kp,Ki,Kd"或"Kp Ki Kd"
    :return: (Kp, Ki, Kd)的元组
    :raises ValueError: 当输入格式不正确时抛出异常
    """
    try:
        # 去除首尾空格
        input_str = input_str.strip()
        
        # 替换逗号为空格，然后分割
        parts = input_str.replace(',', ' ').split()
        
        # 检查参数数量
        if len(parts) != 3:
            raise ValueError(f"需要3个参数，但得到了{len(parts)}个")
        
        # 转换为浮点数
        kp = float(parts[0])
        ki = float(parts[1])
        kd = float(parts[2])
        
        return (kp, ki, kd)
        
    except ValueError as e:
        if "could not convert" in str(e):
            raise ValueError(f"无法解析参数为数字: {input_str}")
        else:
            raise e
    except Exception as e:
        raise ValueError(f"解析PID参数时发生错误: {str(e)}")

def read_pid_params_safe(uart: 'myUART'):
    """
    从UART读取并解析PID参数，自动处理无效文本和异常。
    :param uart: myUART实例
    :return: (kp, ki, kd) 或 None（无效或无数据）
    """
    pid_bytes = uart.read()
    return pid_bytes
    pid_str = pid_bytes.decode().strip() if pid_bytes else ""
    if not pid_str:
        return None
    try:
        kp, ki, kd = parse_pid_params(pid_str)
        return kp, ki, kd
    except Exception as e:
        print("PID参数解析失败：", e)
        return None

def parse_track_info(data: bytes):
    """
    解析ART.py发送的赛道类型信息
    :param data: UART接收到的原始字节数据（如b'\x05S_CURVE\r\n'）
    :return: (track_type: int, track_name: str) 或 None（格式不符）
    """
    if not data or len(data) < 2:
        return None
    track_type = data[0]
    # 查找第一个换行符，提取名称
    try:
        # 赛道名称以\r\n结尾
        name_end = data.find(b'\r\n', 1)
        if name_end == -1:
            name_end = len(data)
        track_name = data[1:name_end].decode(errors='ignore').strip()
        return track_type, track_name
    except Exception as e:
        return None

