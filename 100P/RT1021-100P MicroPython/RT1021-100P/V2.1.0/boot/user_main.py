
# 本示例程序演示如何通过 boot.py 文件进行 soft-boot 控制后执行自己的源文件
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的拨码开关控制

# 示例程序运行效果为复位后执行本文件 通过 C18 电平状态跳转执行 user_main.py
# C4 LED 会一秒周期闪烁

# 从 machine 库包含所有内容
from machine import *

# 包含 gc 与 time 类
import gc
import time

# 核心板上 C4 是 LED
# 学习板上 C19  对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
switch2 = Pin('C19', Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

while True:
    time.sleep_ms(500)
    led.toggle()
    print("led ={:>6d}".format(led.value()))
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break

    gc.collect()
