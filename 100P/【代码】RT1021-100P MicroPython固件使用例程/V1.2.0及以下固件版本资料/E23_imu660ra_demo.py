
# 本示例程序演示如何使用 seekfree 库的 IMU660RA 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板与 IMU660RA 模块测试

# 示例程序运行效果为每 200ms(0.2s) 通过 Type-C 的 CDC 虚拟串口输出信息
# 当 C19 引脚电平出现变化时退出测试程序
# 如果看到 Thonny Shell 控制台输出 ValueError: Module init fault. 报错
# 就证明 IMU660RA 模块连接异常 或者模块型号不对 或者模块损坏
# 请检查模块型号是否正确 接线是否正常 线路是否导通 无法解决时请联系技术支持

# IMU660RA 的更新周期计算方式
# IMU660RA 通过 IMU660RA(x) 初始化构建对象时 传入的 x 代表需要进行几次触发才会更新一次数据
# Ticker 通过 start(y) 启动时 y 代表 Ticker 的周期
# 此时每 y 毫秒会触发一次 IMU660RA 的更新
# 当触发次数大于等于 x 时 IMU660RA 才会更新一次数据
# 因此 IMU660RA 的更新周期时间等于 y * x 本例程中就是 10ms * 1 = 10ms

# 从 machine 库包含所有内容
from machine import *

# 从 smartcar 库包含 ticker
from smartcar import ticker

# 从 seekfree 库包含 IMU660RA
from seekfree import IMU660RA

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

# 调用 IMU660RA 模块获取 IMU660RA 实例
# 参数是采集周期 调用多少次 capture 更新一次数据
# 可以不填 默认参数为 1 调整这个参数相当于调整采集分频
imu = IMU660RA()

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

# 通过 capture 接口更新数据 但在这个例程中被 ticker 模块接管了
# imu.capture()
# 关联采集接口 最少一个 最多八个 (imu, ccd, key...)
# 可关联 smartcar 的 ADC_Group_x 与 encoder_x
# 可关联 seekfree 的  IMU660RA, IMU963RA, KEY_HANDLER 和 TSL1401
pit1.capture_list(imu)

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
        # 翻转 C4 LED 电平
        led.toggle()
        # 通过 get 接口读取数据
        imu_data = imu.get()
        print("acc = {:>6d}, {:>6d}, {:>6d}.".format(imu_data[0], imu_data[1], imu_data[2]))
        print("gyro = {:>6d}, {:>6d}, {:>6d}.".format(imu_data[3], imu_data[4], imu_data[5]))
        ticker_flag = False
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        pit1.stop()
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()
