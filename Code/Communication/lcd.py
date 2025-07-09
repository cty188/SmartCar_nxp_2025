# 从 machine 库包含所有内容
from machine import *

# 包含 display 库
from display import *
from seekfree import TSL1401
# 包含 gc time 类
import gc
import time

class LCDDisplay:
    """封装LCD屏幕操作，提供简洁的API接口"""
    
    def __init__(self, spi_index=1, baudrate=60000000, 
                 cs_pin='C5', rst_pin='B9', dc_pin='B8', 
                 lcd_type=LCD_Drv.LCD200_TYPE):
        """
        初始化LCD显示控制器
        
        参数:
            spi_index: SPI接口索引 (默认1)
            baudrate: SPI通信波特率 (默认60MHz)
            cs_pin: 片选引脚 (默认C5)
            rst_pin: 复位引脚 (默认B9)
            dc_pin: 数据/命令选择引脚 (默认B8)
            lcd_type: LCD类型 (默认LCD200)
        """
        # 初始化引脚
        self.cs = Pin(cs_pin, Pin.OUT, value=True)
        self.rst = Pin(rst_pin, Pin.OUT, value=True)
        self.dc = Pin(dc_pin, Pin.OUT, value=True)
        self.blk = Pin('C4', Pin.OUT, value=True)  # 背光控制引脚
        
        # 确保通信时序正常
        self.cs.high()
        self.cs.low()
        
        # 创建LCD驱动和显示对象
        self.drv = LCD_Drv(SPI_INDEX=spi_index, 
                          BAUDRATE=baudrate, 
                          DC_PIN=self.dc, 
                          RST_PIN=self.rst, 
                          LCD_TYPE=lcd_type)
        
        self.lcd = LCD(self.drv)
        
        # 设置默认颜色和方向
        self.set_color(0xFFFF, 0x0000)  # 白字黑底
        self.set_orientation(2)          # 竖屏180度旋转
        self.clear(0x0000)               # 清屏为黑色
        
        print("LCD初始化完成")
    
    def set_color(self, foreground=0xFFFF, background=0x0000):
        """
        设置前景色和背景色
        
        参数:
            foreground: 前景色 (RGB565格式，默认白色)
            background: 背景色 (RGB565格式，默认黑色)
        """
        self.lcd.color(foreground, background)
    
    def set_orientation(self, orientation):
        """
        设置屏幕方向
        
        参数:
            orientation: 显示方向
                0: 竖屏
                1: 横屏
                2: 竖屏180度旋转
                3: 横屏180度旋转
        """
        self.lcd.mode(orientation)
    
    def clear(self, color=None):
        """
        清屏
        
        参数:
            color: 清屏颜色 (RGB565格式，None表示使用当前背景色)
        """
        if color is not None:
            self.lcd.clear(color)
        else:
            self.lcd.clear()
    
    def draw_text(self, x, y, text, size=16, color=None):
        """
        在指定位置绘制文本
        
        参数:
            x, y: 文本起始坐标
            text: 要显示的文本
            size: 字体大小 (12/16/24/32)
            color: 文本颜色 (None表示使用当前前景色)
        """
        if size == 12:
            self.lcd.str12(x, y, text, color)
        elif size == 16:
            self.lcd.str16(x, y, text, color)
        elif size == 24:
            self.lcd.str24(x, y, text, color)
        elif size == 32:
            self.lcd.str32(x, y, text, color)
        else:
            print(f"警告: 不支持的字号大小 {size}，使用默认16号")
            self.lcd.str16(x, y, text, color)
    
    def draw_line(self, x1, y1, x2, y2, color=None, thickness=1):
        """
        在两点之间绘制直线
        
        参数:
            x1, y1: 起点坐标
            x2, y2: 终点坐标
            color: 线条颜色 (None表示使用当前前景色)
            thickness: 线条粗细 (默认1像素)
        """
        self.lcd.line(x1, y1, x2, y2, color, thickness)
    
    def draw_waveform(self, x, y, width, height, data, max_value=4095):
        """
        绘制波形数据
        
        参数:
            x, y: 波形区域左上角坐标
            width, height: 波形区域的宽度和高度
            data: 波形数据 (列表或数组)
            max_value: 数据的最大值 (用于缩放)
        """
        self.lcd.wave(x, y, width, height, data, max = max_value)
    
    def update_ccd_display(self, ccd_tracker):
        """Display CCD dual-channel waveform and angle info on LCD"""
        # Channel 0 waveform
        data0 = ccd_tracker.get_raw_data(0)
        self.draw_waveform(0, 0, len(data0), 64, data0, max_value=4095)
        angle0 = ccd_tracker.get_line_angle(0)
        
        # Channel 1 waveform (lower half)
        if len(ccd_tracker.channels) > 1:
            data1 = ccd_tracker.get_raw_data(1)
            self.draw_waveform(0, 64, len(data1), 64, data1, max_value=4095)
            angle1 = ccd_tracker.get_line_angle(1)
        else:
            angle1 = None
        
        # Show angle info
        self.draw_text(0, 134, f"A0: {angle0:.2f}", size=16, color=0xFFFF)
        if angle1 is not None:
            self.draw_text(0, 150, f"A1: {angle1:.2f}", size=16, color=0xFFFF)
    
    def show_system_status(self, pitch, left_speed, right_speed, voltage, is_running):
        """Display system status info on LCD"""
        self.draw_text(5, 164, f"Pitch: {pitch:.2f}", size=16, color=0xF800)
        self.draw_text(5, 184, f"Left: {left_speed:.2f}m/s", size=16, color=0x07E0)
        self.draw_text(5, 204, f"Right: {right_speed:.2f}m/s", size=16, color=0x07E0)
        self.draw_text(5, 224, f"Voltage: {voltage:.2f}V", size=16, color=0x001F)
        status = "Running" if is_running else "Stopped"
        color = 0x07E0 if is_running else 0xF800
        self.draw_text(120, 244, status, size=16, color=color)
