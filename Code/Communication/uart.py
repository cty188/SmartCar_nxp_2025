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
    
def printf(uart: myUART, buffer :str):
    uart.write_str(buffer)
    

