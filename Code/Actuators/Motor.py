# 从 machine 库包含所有内容
from machine import *

# 从 seekfree 库包含 MOTOR_CONTROLLER
from seekfree import MOTOR_CONTROLLER
from smartcar import ticker
from smartcar import encoder

import math
# 包含 gc 类
import gc   
# 包含 time 类
import time

class Motor:
    """电机控制类，封装单个电机的所有操作"""
    
    def __init__(self, pwm_pin, dir_pin, encoder_a_pin, encoder_b_pin, 
                 invert=False, freq=13000, duty=0, reverse_encoder=False):
        """
        初始化电机控制器
        
        参数:
        pwm_pin (str): PWM控制引脚
        dir_pin (str): 方向控制引脚
        encoder_a_pin (str): 编码器A相引脚
        encoder_b_pin (str): 编码器B相引脚
        invert (bool): 是否反转电机方向
        freq (int): PWM频率
        duty (int): 初始占空比
        reverse_encoder (bool): 是否反转编码器计数
        """
        # 初始化电机驱动器
        self.controller = MOTOR_CONTROLLER(
            getattr(MOTOR_CONTROLLER, f"PWM_{pwm_pin}_DIR_{dir_pin}"),
            freq, 
            duty=duty, 
            invert=invert
        )
        
        # 初始化编码器
        self.encoder = encoder(encoder_a_pin, encoder_b_pin, reverse_encoder)
        
        # 电机参数
        self.pwm_value = 0  # 当前PWM值
        self.encoder_count = 0  # 当前编码器计数值

        # --- 累加编码器脉冲用于总里程 ---
        self.encoder_total = 0

        # 物理参数 (根据实际硬件调整)
        self.wheel_diameter = 0.064  # 车轮直径(m)
        self.encoder_resolution = 2500  # 编码器每转脉冲数
        self.distance_per_pulse = math.pi * self.wheel_diameter / self.encoder_resolution

        print(f"电机初始化完成: {pwm_pin}/{dir_pin}, 编码器: {encoder_a_pin}/{encoder_b_pin}")
    
    def set_pwm(self, pwm_value):
        """设置电机PWM值"""
        # 限定PWM范围 -3000到3000，避免损坏电机驱动器
        clamped_pwm = max(min(pwm_value, 3000), -3000)
        
        # 设置实际PWM
        self.controller.duty(clamped_pwm)
        self.pwm_value = clamped_pwm
    
    def update_encoder(self):
        """更新编码器计数并返回脉冲增量，同时累加总脉冲"""
        now = self.encoder.get()
        self.encoder_total += now
 
        self.encoder_count = now
        return now
    def get_total_encoder(self):
        """获取累计编码器脉冲数"""
        return self.encoder_total

    def get_total_distance(self):
        """获取累计行驶距离（米）"""
        return self.encoder_total * self.distance_per_pulse
    
    def get_speed(self, dt):
        """计算电机当前速度(m/s)"""
        # 速度 = 距离 / 时间
        # 距离 = 脉冲增量 * 每脉冲距离
        speed = self.update_encoder() * self.distance_per_pulse / dt
        return speed
    
    def reset(self):
        """重置电机状态"""
        self.set_pwm(0)
        self.last_count = 0
        self.current_count = self.encoder.get()
        print("电机状态已重置")


