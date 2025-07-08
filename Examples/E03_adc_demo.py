
# 本示例程序演示如何使用 machine 库的 ADC 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的四路电磁运放接口

# 示例程序运行效果为每 100ms(0.1s) 改变一次 RT1021-MicroPython 核心板的 C4 LED 亮灭状态
# 并通过 RT1021-MicroPython 核心板的 Type-C 的 CDC 虚拟串口输出一次转换数据结果
# 当 C19 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容
from machine import *

# 包含 gc 与 time 类
import gc
import time

# 核心板上 C4 是 LED
# 学习板上 C19 对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
switch2 = Pin('C19', Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

# 构造接口 是标准 MicroPython 的 machine.ADC 模块
#   Pin(pin)
#   pin     引脚名称    | 必要参数 引脚名称 本固件以核心板上引脚编号为准
adc_in1 = ADC('B14')
adc_in2 = ADC('B15')
adc_in3 = ADC('B26')
adc_in4 = ADC('B27')
# 运放对应的四个引脚都支持 ADC1 当引脚支持两个模块通道时优先分配到 ADC1 功能

while True:
    # 延时 100 ms
    time.sleep_ms(100)
    # 翻转 C4 LED 电平
    led.toggle()
    
    # 读取通过 read_u16 接口读取 无参数 数据返回范围是 0-65535
    print("adc={:>6d},{:>6d},{:>6d},{:>6d}.\r\n".format(
        adc_in1.read_u16(), adc_in2.read_u16(),
        adc_in3.read_u16(), adc_in4.read_u16()))
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
