import sensor, image
import seekfree, pyb
from machine import UART

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(10)                      # 设置跳过的帧数
sensor.set_auto_gain(False)                 # 关闭自动增益
sensor.set_brightness(500)                  # 设置曝光
uart = UART(2, baudrate=115200)             # 配置串口

while(True):
    img = sensor.snapshot()
    #img.lens_corr(1.8)                                                                                                 # 进行镜头畸变校正
    for code in img.find_qrcodes():                                                                                     # 遍历查找二维码
        # code.rect()中包含x,y,w,h四个数据信息，分别对应检测到的二维码的左上角x,y坐标，宽度、高度。
        # 各数据信息也可单独获取。例如可以直接使用code.x()的方式直接获取左上角x坐标数据信息
        img.draw_rectangle(code.rect(), color = (255, 0, 0),thickness = 1)                                              # 绘制矩形框
        img.draw_string(code.x(), code.y(), code.payload(), color = (255, 0, 0), thickness = 2)                         # 将二维码文本信息叠加显示在图像上
        uart.write("x,y :" + str(code.x() + int(code.w() / 2)) + ',' + str(code.y() + int(code.h() / 2)) + "\t")        # 发送二维码中心坐标
        uart.write("text:" + code.payload() + "\r\n")                                                                   # 发送二维码文本信息
#        print(code)                                                                                                    # 终端显示二维码列表信息

