# 注意：以下是修改后的代码，主要将电机操作封装为Motor类
# 其余部分代码保持不变，仅展示修改部分

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
        
        # 物理参数 (根据实际硬件调整)
        self.wheel_diameter = 0.065  # 车轮直径(m)
        self.encoder_resolution = 1200  # 编码器每转脉冲数
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
        """更新编码器计数并返回脉冲增量"""
        self.encoder_count = self.encoder.get()
        return self.encoder_count 
    
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


class BalanceCarController:
    """平衡小车控制主类"""
    
    def __init__(self):
        # 初始化左侧电机
        self.left_motor = Motor(
            pwm_pin="C24", 
            dir_pin="C26",
            encoder_a_pin="D0",
            encoder_b_pin="D1",
            invert=True,
            reverse_encoder=True
        )
        
        # 初始化右侧电机
        self.right_motor = Motor(
            pwm_pin="C25", 
            dir_pin="C27",
            encoder_a_pin="D2",
            encoder_b_pin="D3",
            invert=True,
            reverse_encoder=True
        )
        
        # 其他初始化保持不变...
    
    def read_encoders(self):
        """更新并获取编码器值"""
        self.left_motor.update_encoder()
        self.right_motor.update_encoder()
        return self.left_motor.current_count, self.right_motor.current_count
    
    def calculate_speed(self):
        """计算小车速度"""
        # 获取采样间隔
        dt = 1.0 / CONTROL_FREQUENCY
        
        # 计算左右轮速度
        left_speed = self.left_motor.get_speed(dt)
        right_speed = self.right_motor.get_speed(dt)
        
        # 计算小车线速度
        linear_speed = (left_speed + right_speed) / 2.0
        
        return left_speed, right_speed
    
    def set_motor_pwm(self, left_pwm, right_pwm):
        """设置电机PWM值"""
        self.left_motor.set_pwm(left_pwm)
        self.right_motor.set_pwm(right_pwm)
    
    def update_balance_control(self):
        """更新平衡控制"""
        # 其他代码保持不变...
        
        # 在调试输出中使用封装后的PWM属性
        if self.debug_enabled and self.loop_count % self.debug_interval == 0:
            print(f"电机PWM: 左={self.left_motor.pwm_value}, 右={self.right_motor.pwm_value}")
    
    def start(self):
        """启动平衡小车"""
        print("启动平衡小车...")
        
        # 重置电机状态
        self.left_motor.reset()
        self.right_motor.reset()
        
        # 其他代码保持不变...
    
    def stop(self):
        """停止平衡小车"""
        # 安全停止电机
        self.left_motor.set_pwm(0)
        self.right_motor.set_pwm(0)
        
        # 其他代码保持不变...