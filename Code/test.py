# 包含 gc 与 time 类
import gc
import time
from machine import Pin
from Communication.uart import myUART, printf, read_pid_params_safe

# 定义硬件引脚
led = Pin('C4', Pin.OUT, value=True)
switch2 = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

uart = myUART(2)

while True:
    # 每 500ms 读取一次 将数据再原样发回
    # uart.write_str("fine!")
    
    pid_params = read_pid_params_safe(uart)
    if pid_params:
        kp, ki, kd = pid_params
        print("PID parameters are valid.")
        print("Kp, Ki, Kd:", kp, ki, kd)
        
    time.sleep_ms(2000)
    # 翻转 C4 LED 电平
    led.toggle()
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break

    # 回收内存
    gc.collect()
