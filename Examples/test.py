
# 本示例程序演示如何通过 boot.py 文件进行 soft-boot 控制后执行自己的源文件
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的拨码开关控制

# 示例程序运行效果为复位后执行本文件 通过 C18 电平状态跳转执行 user_main.py
# C4 LED 会一秒周期闪烁

# 从 machine 库包含所有内容
from machine import *
from smartcar import ticker
from smartcar import encoder
from seekfree import KEY_HANDLER
# 包含 gc 与 time 类
import gc
import time
import math
from display import *
from seekfree import TSL1401
from seekfree import MOTOR_CONTROLLER
from seekfree import WIRELESS_UART
from display import *
from seekfree import IMU660RX# 从 seekfree 库包含 IMU963RA

from Imu import IMUProcessor
from KalmanFilter import KalmanFilter,PitchKalmanFilter
from MahonyFilter import MahonyFilter
from pid import PIDController

# 核心板上 C4 是 LED
# 学习板上 C19  对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
switch2 = Pin('C19', Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()


CONTROL_FREQUENCY = 10.0

class BalanceCarController:
    """平衡小车控制主类"""
    
    def __init__(self):
        """
        初始化平衡小车控制器
        """
        # 初始化IMU处理器 (100Hz采样率)
        self.imu_processor = IMUProcessor(sample_rate=CONTROL_FREQUENCY, kp=0.5, ki=0.01)

        # 执行陀螺仪零偏校准
        print("执行陀螺仪零偏校准...")
        self.imu_processor.calibrate_gyro()
        
        # 初始化Pitch角卡尔曼滤波器
        self.pitch_filter = PitchKalmanFilter(1.0 / CONTROL_FREQUENCY)

        # 初始化平衡PID控制器 (位置式PID)
        # 参数: 模式, Kp, Ki, Kd, 最大输出, 最大积分输出
        self.balance_pid = PIDController(
            mode=PIDController.PID_POSITION,
            Kp=20.0,   # 比例增益
            Ki=0.5,    # 积分增益
            Kd=0.8,    # 微分增益
            max_out=1000.0,  # 最大输出限制
            max_iout=500.0   # 最大积分限制
        )
        
        # 小车状态
        self.base_target_pitch = 0.0  # 基础平衡俯仰角
        self.target_pitch = 0.0  # 目标俯仰角 
        self.is_running = False  # 小车运行状态
        
        # 电机控制相关
        self.left_motor = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C24_DIR_C26, 13000, duty = 0, invert = True)
        self.right_motor = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C25_DIR_C27, 13000, duty = 0, invert = True)
        self.left_motor_pwm = 0  # 左电机PWM值
        self.right_motor_pwm = 0  # 右电机PWM值

        # 调试信息
        self.debug_enabled = True
        self.debug_interval = 50  # 调试信息输出间隔(循环次数)
        self.loop_count = 0
        
        # 安全参数
        self.max_pitch_angle = 30.0  # 最大允许俯仰角(度)
        self.min_battery_voltage = 6.0  # 最低电池电压(V)
        
        # 编码器相关 
        self.left_encoder = encoder("D0", "D1", True)
        self.right_encoder = encoder("D2", "D3", True)
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.last_left_encoder_count = 0
        self.last_right_encoder_count = 0

        # 速度控制相关
        self.left_speed = 0.0
        self.right_speed = 0.0

        # 初始化平衡小车的平衡中断
        self.balance_ticker = ticker(1)
        self.balance_ticker.capture_list(self.left_encoder, self.right_encoder, self.imu_processor.imu)
        self.balance_ticker.callback(self._balance_tick_handler)

        # 速度控制PID
        self.speed_pid = PIDController(
            mode=PIDController.PID_POSITION,
            Kp=5.0,
            Ki=0.1,
            Kd=0.05,
            max_out=10.0,
            max_iout=5.0
        )
        self.target_speed = 0.0  # 目标速度(m/s)
        
        # 转向控制PID
        self.steering_pid = PIDController(
            mode=PIDController.PID_POSITION,
            Kp=2.0,
            Ki=0.05,
            Kd=0.1,
            max_out=5.0,
            max_iout=2.0
        )
        self.target_heading = 0.0  # 目标偏航角(度)
        
        print("平衡小车控制器初始化完成")
    
    def read_encoders(self):
        """读取编码器值 (需要根据实际硬件实现)"""
        # 这里应该是从硬件读取编码器值的代码
        # 返回左右编码器的计数值
        # 示例: 返回模拟值
        return self.left_encoder.get(), self.right_encoder.get()
    
    def calculate_speed(self):
        """计算小车速度 (需要根据实际硬件实现)"""
        # 编码器参数 (需要根据实际车轮尺寸和编码器分辨率调整)
        wheel_diameter = 0.065  # 车轮直径(m)
        encoder_resolution = 1000  # 编码器每转脉冲数
        
        # 读取当前编码器值
        self.left_encoder_count, self.right_encoder_count = self.read_encoders()

        # 计算脉冲差
        left_delta = self.left_encoder_count - self.last_left_encoder_count
        right_delta = self.right_encoder_count - self.last_right_encoder_count

        # 更新上次编码器值
        self.last_left_encoder_count = self.left_encoder_count
        self.last_right_encoder_count = self.right_encoder_count

        # 计算速度 (m/s)
        # 每个脉冲对应的距离 = π * 车轮直径 / 编码器分辨率
        distance_per_pulse = math.pi * wheel_diameter / encoder_resolution
        
        # 假设采样周期为0.01秒 (100Hz)
        dt = 1.0 / CONTROL_FREQUENCY
        
        # 计算左右轮速度
        left_speed = left_delta * distance_per_pulse / dt
        right_speed = right_delta * distance_per_pulse / dt
        
        # 计算小车线速度
        linear_speed = (left_speed + right_speed) / 2.0
        
        return linear_speed
    
    def set_motor_pwm(self, left_pwm, right_pwm):
        """设置电机PWM值 """
        # 这里应该是控制电机驱动的代码
        self.left_motor.duty(left_pwm)
        self.right_motor.duty(right_pwm)

        # 调试输出
        if self.debug_enabled and self.loop_count % self.debug_interval == 0:
            print(f"设置电机PWM: 左={left_pwm}, 右={right_pwm}")
    
    def read_battery_voltage(self):
        """读取电池电压"""
        # 这里应该是读取电池电压的代码
        # 返回模拟值
        return 12.3  # 示例电压值
    
    def safety_check(self):
        """执行安全检查"""
        # 检查电池电压
        battery_voltage = self.read_battery_voltage()
        if battery_voltage < self.min_battery_voltage:
            print(f"电池电压过低: {battery_voltage}V < {self.min_battery_voltage}V")
            return False
        
        # 检查俯仰角是否超过安全范围
        current_pitch = self.pitch_filter.get_last_filtered_pitch()
        if abs(current_pitch) > self.max_pitch_angle:
            print(f"俯仰角超过安全范围: {current_pitch:.1f}° > {self.max_pitch_angle}°")
            return False
        
        return True
    
    def update_balance_control(self):
        """更新平衡控制"""
        # 1. 更新IMU数据
        self.imu_processor.update()
        
        # 2. 获取原始传感器数据
        raw_gyro = self.imu_processor.get_raw_gyro()
        pitch_angle = self.imu_processor.get_pitch()
        
        # 3. 使用卡尔曼滤波器处理Pitch角
        # 注意: Mahony滤波已经提供了角度估计，但卡尔曼滤波可以提供更平滑的结果
        '''filtered_pitch = self.pitch_filter.update(
            pitch_rate=raw_gyro[1],  # 假设Y轴是俯仰轴
            pitch_measurement=pitch_angle
        )'''
        filtered_pitch = raw_gyro[1]
        # 4. 串级PID控制：外环速度PID -> 内环角度PID
        # 计算速度控制输出 (外环)
        current_speed = self.calculate_speed()
        target_pitch = self.speed_pid.compute(
            feedback=current_speed,
            setpoint=self.target_speed
        )
        
        # 约束目标俯仰角的调整量 (防止过大角度)
        target_pitch = max(min(target_pitch, 15.0), -15.0)

        # 计算实际目标俯仰角 (基础0度 + 速度调整)
        effective_target_pitch = self.base_target_pitch + target_pitch
        
        # 5. 计算内环(角度)控制输出
        balance_output = self.balance_pid.compute(
            feedback=filtered_pitch,
            setpoint=effective_target_pitch
        )
        
        # 6. 计算转向控制输出
        '''current_yaw = self.imu_processor.get_yaw()
         = self.steering_pid.compute(
            feedback=current_yaw,
            setpoint=self.target_heading
        )'''
        steering_output = 0
        # 7. 计算左右电机PWM
        base_output = balance_output
        left_pwm = base_output - steering_output
        right_pwm = base_output + steering_output
        
        # 8. 设置电机输出
        #self.set_motor_pwm(left_pwm, right_pwm)
        
        # 9. 调试输出
        if self.debug_enabled and self.loop_count % self.debug_interval == 0:
            print(f"实际俯仰角: {filtered_pitch:.2f}°, 目标俯仰角: {effective_target_pitch:.2f}°")
            #print(f"当前速度: {current_speed:.2f}m/s, 目标速度: {self.target_speed:.2f}m/s")
            #print(f"速度调整量: {target_pitch:.2f}°")
            #print(f"平衡输出: {balance_output:.1f}, 转向输出: {steering_output:.1f}")
            #print(f"电机PWM: 左={left_pwm:.1f}, 右={right_pwm:.1f}")

    def _balance_tick_handler(self, time):
        """定时器中断处理函数 - 在固定频率执行平衡控制任务"""
        if not self.is_running:
            return
        
        # 执行安全检查
        '''if not self.safety_check():
            print("安全条件不满足，停止小车")
            self.stop()
            return'''
        
        # 更新控制
        self.update_balance_control()

    def start(self):
        """启动平衡小车"""
        print("启动平衡小车...")
        self.is_running = True
        
        # 重置滤波器状态
        self.pitch_filter.reset()
        self.balance_pid.reset()
        self.speed_pid.reset()
        self.steering_pid.reset()
        
        # 启动平衡控制定时器
        self.balance_ticker.start(1000 // CONTROL_FREQUENCY)  # 毫秒计算

        try:
            # 主循环仅处理非实时任务
            while self.is_running:
                # 此处可以处理非实时任务，如用户输入、数据显示等
                time.sleep_ms(100)  # 降低CPU负载   
        except switch2.value() != state2:
            print("Test program stop.")
        gc.collect()
    
    def stop(self):
        """停止平衡小车"""
        print("停止平衡小车...")
        self.is_running = False
        self.set_motor_pwm(0, 0)  # 停止电机
        self.balance_ticker.stop()  # 停止定时器
    
    def set_target_speed(self, speed):
        """设置目标速度"""
        self.target_speed = speed
        print(f"设置目标速度: {speed:.2f}m/s")
    
    def set_target_heading(self, heading):
        """设置目标航向"""
        self.target_heading = heading
        print(f"设置目标航向: {heading:.1f}°")
    
    def adjust_pid_parameters(self, Kp=None, Ki=None, Kd=None):
        """调整PID参数"""
        if Kp is not None:
            self.balance_pid.Kp = Kp
        if Ki is not None:
            self.balance_pid.Ki = Ki
        if Kd is not None:
            self.balance_pid.Kd = Kd
        
        print(f"更新PID参数: Kp={self.balance_pid.Kp}, Ki={self.balance_pid.Ki}, Kd={self.balance_pid.Kd}")


# 主程序入口
if __name__ == "__main__":
    # 创建平衡小车控制器
    car_controller = BalanceCarController()
    
    # 启动平衡小车
    car_controller.start()

