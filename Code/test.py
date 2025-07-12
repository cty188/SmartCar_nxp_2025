# 包含 gc 与 time 类
import gc
import time
from machine import Pin, UART
from Communication.uart import myUART, printf, read_pid_params_safe

# 定义硬件引脚
led = Pin('C4', Pin.OUT, value=True)
switch2 = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

uart = myUART(1)

def parse_track_info(data: bytes):
    """
    解析ART.py发送的赛道类型信息
    支持多种格式：
    1. b'S_CURVE:2\r\n' - 名称:类型 格式
    2. b'\x05S_CURVE\r\n' - 字节+名称 格式
    :param data: UART接收到的原始字节数据
    :return: (track_type: int, track_name: str) 或 None（格式不符）
    """
    if not data:
        return None
    
    try:
        # 将字节数据解码为字符串
        data_str = data.decode('utf-8', errors='ignore').strip()
        
        # 移除所有的\r\n
        data_str = data_str.replace('\r\n', '')
        
        # 处理重复消息（如 "S_CURVE:2S_CURVE:2"）
        # 查找第一个冒号的位置
        colon_pos = data_str.find(':')
        if colon_pos != -1:
            # 提取第一个完整的消息
            track_name = data_str[:colon_pos]
            remaining = data_str[colon_pos+1:]
            
            # 提取数字部分
            track_type_str = ""
            for char in remaining:
                if char.isdigit():
                    track_type_str += char
                else:
                    break
            
            if track_type_str:
                track_type = int(track_type_str)
                return track_type, track_name
        
        # 如果没有冒号，检查是否是字节+名称格式
        # 这种情况下第一个字符应该是不可打印字符
        if len(data_str) > 1 and ord(data_str[0]) < 32:
            track_type = ord(data_str[0])
            track_name = data_str[1:]
            return track_type, track_name
        
        return None
    except Exception as e:
        print(f"解析错误: {e}")
        return None

while True:
    # 发送查询命令给ART模块
    uart.write_bytes(b'GET_TRACK')
    print("已发送: GET_TRACK")
    
    time.sleep_ms(50)  # 等待ART模块响应
    
    # 读取ART模块响应
    data = uart.read()
    if data:
        print(f"收到原始数据: {data}")
        # 添加原始字节查看
        print(f"原始字节: {[hex(b) for b in data]}")
        
    print("----------------------")
    
    # 翻转 C4 LED 电平
    led.toggle()
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    if switch2.value() != state2:
        print("Test program stop.")
        break

    # 回收内存
    gc.collect()
    
    # 等待2秒再次查询
    time.sleep_ms(2000)

