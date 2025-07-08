
# 本示例程序演示如何使用 smartcar 库的 encoder 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的编码器接口测试

# 示例程序运行效果为每 200ms(0.2s) C4 LED 改变亮灭状态
# 并通过 Type-C 的 CDC 虚拟串口输出一次信息
# 可以通过 C19 的电平状态来控制是否退出测试程序

# encoder 的采集周期计算方式
# Ticker 通过 start(y) 启动时 y 代表 Ticker 的周期
# 此时每 y 毫秒会触发一次 encoder 的更新
# 因此 encoder 的采集周期时间等于 y 本例程中就是 10ms

# 从 machine 库包含所有内容
from machine import *

# 从 smartcar 库包含 ticker encoder
from smartcar import ticker
from smartcar import encoder

# 包含 gc 类
import gc

# 核心板上 C4 是 LED
# 学习板上 C19 对应二号拨码开关

# 调用 machine 库的 Pin 类实例化一个引脚对象
# 配置参数为 引脚名称 引脚方向 模式配置 默认电平
# 详细内容参考 固件接口说明
led     = Pin('C4' , Pin.OUT, pull = Pin.PULL_UP_47K, value = True)
switch2 = Pin('C19', Pin.IN , pull = Pin.PULL_UP_47K, value = True)

state2  = switch2.value()

# 实例化 encoder 模块
# 对应学习板的编码器接口 1/2
# 总共三个参数 两个必要参数一个可选参数 [pinA,pinB,invert]
# pinA - 编码器 A 相或 PLUS 引脚
# pinB - 编码器 B 相或 DIR 引脚
# invert - 可选参数 是否反向 可以通过这个参数调整编码器旋转方向数据极性
# 详细内容参考 固件接口说明
encoder_l = encoder("D0", "D1", True)
encoder_r = encoder("D2", "D3")

ticker_flag = False
ticker_count = 0

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    ticker_flag = True  # 否则它会新建一个局部变量
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)

# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit1 = ticker(1)

# 通过 capture 接口更新数据 但在这个例程中被 ticker.capture_list 模块接管了
# encoder_l.capture()
# encoder_r.capture()
# 关联采集接口 最少一个 最多八个 (imu, ccd, key...)
# 可关联 smartcar 的 ADC_Group_x 与 encoder_x
# 可关联 seekfree 的  IMU660RA, IMU963RA, KEY_HANDLER 和 TSL1401
pit1.capture_list(encoder_l, encoder_r)

# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit1.start(10)

# 需要注意的是 ticker 是底层驱动的 这导致 Thonny 的 Stop 命令在这个固件版本中无法停止它
# 因此一旦运行了使用了 ticker 模块的程序 要么通过复位核心板重新连接 Thonny
# 或者像本示例一样 使用一个 IO 控制停止 Ticker 后再使用 Stop/Restart backend 按钮
# V1.1.2 以上版本则可以直接通过 Stop/Restart backend 按钮停止 Ticker

while True:
    if (ticker_flag and ticker_count % 20 == 0):
        led.toggle()
        # 通过 get 接口读取数据
        encl_data = encoder_l.get()
        encr_data = encoder_r.get()
        print("enc ={:>6d}, {:>6d}\r\n".format(encoder_l.get(), encoder_r.get()))
        ticker_flag = False
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        pit1.stop()
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
