from machine import Pin
import sensor, image, time
import seekfree, pyb

# 初始化激光灯控制引脚，并置为高电平
laser_light=Pin("C9", Pin.OUT)
laser_light.value(1)

# 初始化TFT180屏幕
lcd = seekfree.LCD180(3)

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565) # 设置图像色彩格式为RGB565格式
sensor.set_framesize(sensor.QQVGA)  # 设置图像大小为160*120
sensor.set_auto_whitebal(True)      # 设置自动白平衡
sensor.set_brightness(3000)         # 设置亮度为3000
sensor.skip_frames(time = 20)       # 跳过帧

clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()

# -----跟踪激光部分-----
    # 设置激光颜色阈值
    red_td = [(56, 100, 45, 127, -128, 127)]
    # 根据阈值找到色块
    for b in img.find_blobs(red_td,pixels_threshold=2, area_threshold=15, merge=True,invert = 0):
        # 在屏幕上画出色块
        img.draw_rectangle(b.rect(), color = (0, 255, 0), scale = 2, thickness = 2)
        
        # 打印激光色块的中心位置  下面获取的坐标基于图像的左上角为原点(0, 0),X轴向右增大，Y轴向下增大
        # 使用b.x()获取色块矩形左上角X坐标
        # 使用b.y()获取色块矩形左上角Y坐标
        # 使用b.w()获取色块矩形宽度
        # 使用b.h()获取色块矩形高度
        # 矩形中心坐标为（x + w/2，y + h/2)
        print(f"rect = {b.x() + b.w()/2},{b.y() + b.h()/2}")
        break

    # 显示到屏幕上，此部分会降低帧率
    lcd.show_image(img, 160, 120, 0, 0, zoom=0)  #屏幕显示

    # 打印帧率
    print(clock.fps())
