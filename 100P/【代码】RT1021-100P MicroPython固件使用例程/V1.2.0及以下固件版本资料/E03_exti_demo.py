
# 本示例程序演示如何使用 machine 库的 Pin 类接口的外部中断
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的霍尔停车检测接口

# 示例程序运行效果为每 1000ms(1s) 改变一次 RT1021-MicroPython 核心板的 C4 LED 亮灭状态
# 当 C21 引脚电平出现由高拉低的下降沿时 触发一次外部中断回调
# 中断回调通过 RT1021-MicroPython 核心板的 Type-C 的 CDC 虚拟串口输出触发次数
# 当 C19 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容
from machine import *

# 包含 gc 与 time 类
import gc
import time

# 核心板上 C4 是 LED
# 学习板上 C21 对应霍尔停车检测接口
# 学习板上 C19 对应二号拨码开关

# 调用 machine 库的 Pin 类实例化一个引脚对象
# 配置参数为 引脚名称 引脚方向 模式配置 默认电平
# 详细内容参考 固件接口说明
led     = Pin('C4' , Pin.OUT, pull = Pin.PULL_UP_47K, value = True)
hall    = Pin('C21', Pin.IN , pull = Pin.PULL_UP_47K, value = True)
switch2 = Pin('C19', Pin.IN , pull = Pin.PULL_UP_47K, value = True)

state2  = switch2.value()

# 新建一个变量用于计数
hall_count = 0

# 定义一个回调函数 必须有一个参数用于传递实例本身
def hall_handler(x):
    # global 是全局变量修饰 说明这里使用的是一个全局变量
    # 否则函数会新建一个临时变量
    global hall_count

    # 计数递增 并通过 Type-C 的 CDC 虚拟串口输出触发次数
    hall_count = hall_count + 1
    print("hall_count ={:>6d}, hall_state ={:>6d}".format(hall_count, x.value()))

# 配置 Pin 的中断 也就是外部中断 EXTI
# 由于选择的是下降沿触发 因此回调函数中的状态一般是 0 如果改成上升沿则是 1
hall.irq(hall_handler, Pin.IRQ_FALLING, False) # IRQ_RISING IRQ_FALLING

while True:
    time.sleep_ms(1000)
    led.toggle()
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
