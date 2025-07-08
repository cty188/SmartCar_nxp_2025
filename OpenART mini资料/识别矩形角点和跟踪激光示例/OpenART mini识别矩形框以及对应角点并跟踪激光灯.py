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

# -----矩形框部分-----
    # 在图像中寻找矩形
    for r in img.find_rects(threshold = 10000):
        # 判断矩形边长是否符合要求
        if r.w() > 20 and r.h() > 20:
            # 在屏幕上框出矩形
            img.draw_rectangle(r.rect(), color = (255, 0, 0), scale = 4)
            # 获取矩形角点位置
            corner = r.corners()
            # 在屏幕上圈出矩形角点
            img.draw_circle(corner[0][0], corner[0][1], 5, color = (0, 0, 255), thickness = 2, fill = False)
            img.draw_circle(corner[1][0], corner[1][1], 5, color = (0, 0, 255), thickness = 2, fill = False)
            img.draw_circle(corner[2][0], corner[2][1], 5, color = (0, 0, 255), thickness = 2, fill = False)
            img.draw_circle(corner[3][0], corner[3][1], 5, color = (0, 0, 255), thickness = 2, fill = False)
            
            # 角点坐标打印详见OpenART mini识别矩形框以及对应角点文件
            # 获取的坐标基于图像的左上角为原点(0, 0),X轴向右增大，Y轴向下增大
# -----跟踪激光部分-----
    # 设置激光颜色阈值
    red_td = [(56, 100, 45, 127, -128, 127)]
    # 根据阈值找到色块
    for b in img.find_blobs(red_td,pixels_threshold=2, area_threshold=15, merge=True,invert = 0):
        # 在屏幕上画出色块
        img.draw_rectangle(b.rect(), color = (0, 255, 0), scale = 2, thickness = 2)
        break

        # 坐标打印详见OpenART mini跟踪激光灯文件
        # 获取的坐标基于图像的左上角为原点(0, 0),X轴向右增大，Y轴向下增大
    # 显示到屏幕上，此部分会降低帧率
    lcd.show_image(img, 160, 120, 0, 0, zoom=0)  #屏幕显示

    # 打印帧率
    print(clock.fps())
