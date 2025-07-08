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
            print("uart received ={:>6d},content:{}".format(buf_len, buf))
            self.uart.write("self.uart:")
            self.uart.write(buf)
            self.uart.write("\n")
            return buf
        return b""  # 返回空字节串当没有数据时

    def writeln(self, data: str) -> None:
        """
        写入数据并添加换行符
        :param data: 要写入的字符串数据
        """
        self.uart.write(data + "\n")
    
    def write_str(self, data: str) -> None:
        """
        写入字符串数据
        :param data: 要写入的字符串数据
        """
        self.writeln(data)

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
    
def printf(uart: myUART, buffer: str) -> None:
    """
    通过UART输出字符串
    :param uart: myUART实例
    :param buffer: 要输出的字符串
    """
    uart.write_str(buffer)
    
# 解析传入pid参数
def parse_pid_params(input_str: str) -> tuple[float, float, float]:
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

def validate_pid_params(kp: float, ki: float, kd: float) -> bool:
    """
    验证PID参数是否在合理范围内
    :param kp: 比例增益
    :param ki: 积分增益
    :param kd: 微分增益
    :return: 如果参数合理返回True，否则返回False
    """
    # 基本范围检查（可根据实际需求调整）
    if kp < 0 or kp > 1000:
        return False
    if ki < 0 or ki > 100:
        return False
    if kd < 0 or kd > 100:
        return False
    return True 
