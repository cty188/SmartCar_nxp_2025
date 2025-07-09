# 从 machine 库包含所有内容
from machine import *
from smartcar import ticker

from seekfree import KEY_HANDLER
# 包含 gc 与 time 类
import gc
import time
import math
from display import *

from seekfree import WIRELESS_UART

from display import *
from seekfree import IMU660RX# 从 seekfree 库包含 IMU660RX

from Sensors.Imu import IMUProcessor
from Sensors.ccd import CCDTracker  # 从 Sensors.CCD 包含 CCDTracker

from Algorithm.KalmanFilter import KalmanFilter,PitchKalmanFilter
from Algorithm.MahonyFilter import MahonyFilter
from Algorithm.pid import PIDController

from Communication.lcd import LCDDisplay  # 从 Actuators 包含 LCDDisplay

from Actuators.Motor import Motor

# 核心板上 C4 是 LED
# 学习板上 C19  对应二号拨码开关
led     = Pin('C4' , Pin.OUT, value = True)
switch2 = Pin('C19', Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()


CONTROL_FREQUENCY = 100.0

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
        
        # 小车状态
        self.is_running = False  # 小车运行状态
        
        # 电机控制相关
        self.left_motor = Motor(pwm_pin="C24", dir_pin="C26", 
                   encoder_a_pin="D2", encoder_b_pin="D3",
                   invert=True)
        self.right_motor = Motor(pwm_pin="C25", dir_pin="C27", 
                   encoder_a_pin="D0", encoder_b_pin="D1",
                   invert=True)

        # 调试信息
        self.debug_enabled = True
        self.debug_interval = 50  # 调试信息输出间隔(循环次数)
        self.loop_count = 0
        
        # 安全参数
        self.max_pitch_angle = 30.0  # 最大允许俯仰角(度)
        self.min_battery_voltage = 6.0  # 最低电池电压(V)

        # 集成CCD双通道循迹模块
        self.ccd_tracker = CCDTracker(capture_div=10, channels=(0,1))
        # 初始化平衡小车的平衡中断
        self.balance_ticker = ticker(1)
        self.balance_ticker.capture_list(self.left_motor.encoder, self.right_motor.encoder, self.imu_processor.imu, self.ccd_tracker.ccd)
        self.balance_ticker.callback(self._balance_tick_handler)
        
        # 1. 角度环PID控制器 (输出目标速度)
        self.angle_pid = PIDController(
            mode=PIDController.PID_POSITION,
            Kp=0.1,     # 角度环比例增益（值越大，恢复平衡越强）
            Ki=0.0050,     # 角度环积分增益
            Kd=0.0,     # 角度环微分增益（减小过冲）
            max_out=10.0, # 最大输出速度(m/s)
            max_iout=5.0 # 最大积分输出
        )
        self.base_target_pitch=9.5  # 基础平衡俯仰角
        self.target_pitch = 0.0  # 目标俯仰角
        
        # 2. 速度环PID控制器 (输出电机PWM)
        self.left_speed_pid = PIDController(
            mode=PIDController.PID_POSITION,
            Kp=2000.0,   # 速度环比例增益（值越大，响应越快）
            Ki=100.0,     # 速度环积分增益（消除稳态误差）
            Kd=0.0,    # 速度环微分增益（抑制振荡）
            max_out=6000.0,  # 最大输出PWM
            max_iout=1000.0  # 最大积分输出
        )
        self.right_speed_pid = PIDController(
            mode=PIDController.PID_POSITION,
            Kp=2000.0,   # 速度环比例增益（值越大，响应越快）
            Ki=100.0,     # 速度环积分增益（消除稳态误差）
            Kd=0.0,    # 速度环微分增益（抑制振荡）
            max_out=6000.0,  # 最大输出PWM
            max_iout=1000.0  # 最大积分输出
        )
        self.target_speed = 0.0  # 目标速度(m/s)
        
        # 转向控制PID
        self.steering_pid = PIDController(
            mode=PIDController.PID_POSITION,
            Kp=0.0,  # -0.01 暂时禁用转向控制
            Ki=0.0,
            Kd=0.0,
            max_out=1.0,
            max_iout=0.5
        )
        self.target_heading = 0.0  # 目标偏航角(度)
        
        # 速度环外部目标速度->期望倾角 PID 控制器
        self.speed_to_angle_pid = PIDController(
            mode=PIDController.PID_POSITION,
            Kp=-10.0,   # 速度到倾角的比例增益（需根据实际调试）
            Ki=0.01,  # 积分增益
            Kd=0.0,   # 微分增益
            max_out=10.0,  # 最大期望倾角（度）
            max_iout=5.0   # 最大积分输出
        )
        
        # LCD显示
        self.lcd_display = LCDDisplay()
        
        print("平衡小车控制器初始化完成")
    
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
        roll, pitch, yaw = self.imu_processor.get_attitude_deg()
        
        current_left_speed = self.left_motor.get_speed(1/CONTROL_FREQUENCY)
        current_right_speed = self.right_motor.get_speed(1/CONTROL_FREQUENCY)
        
        # 3. 使用卡尔曼滤波器处理Pitch角
        filtered_pitch = self.pitch_filter.update(
            pitch_rate=raw_gyro[1],  # 假设Y轴是俯仰轴
            pitch_measurement=pitch_angle
        )
        
        # 4. 速度环外部：目标速度->期望倾角
        current_speed = (current_left_speed + current_right_speed) / 2.0
        delta_pitch = self.speed_to_angle_pid.compute(
            feedback=current_speed,
            setpoint=self.target_speed
        )

        # 5. 串级PID控制：外环角度PID -> 内环速度PID
        target_speed_from_angle = self.angle_pid.compute(
            feedback=filtered_pitch,
            setpoint=self.base_target_pitch  # 暂时禁用速度补偿: + delta_pitch
        )

        # 6. 计算转向控制输出
        self.target_heading = self.ccd_tracker.get_line_angle(0)
        target_speed_from_steering = self.steering_pid.compute(
            feedback=0,
            setpoint=self.target_heading
        )
        
        # 7. 设置目标速度
        left_target_speed = target_speed_from_angle - target_speed_from_steering
        right_target_speed = target_speed_from_angle + target_speed_from_steering

        # 8. 计算电机控制输出
        left_motor_output = self.left_speed_pid.compute(
            feedback=current_left_speed,
            setpoint=left_target_speed
        )
        right_motor_output = self.right_speed_pid.compute(
            feedback=current_right_speed,
            setpoint=right_target_speed
        )
        
        # 9. 设置电机输出
        self.left_motor.set_pwm(left_motor_output)
        self.right_motor.set_pwm(right_motor_output)

        # 10. 调试输出
        if self.debug_enabled and self.loop_count % self.debug_interval == 0:
            print(f"实际俯仰角: {filtered_pitch:.2f}°, 目标俯仰角: {self.base_target_pitch:.2f}°")
            print(f"左轮速度: {current_left_speed:.2f}m/s, 右轮速度: {current_right_speed:.2f}m/s")
            print(f"左轮PWM: {left_motor_output:.1f}, 右轮PWM: {right_motor_output:.1f}")

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
        self.loop_count += 1
    
    def start(self):
        """启动平衡小车"""
        print("启动平衡小车...")
        self.is_running = True
        
        # 重置滤波器状态
        self.pitch_filter.reset()
        self.angle_pid.reset()
        self.left_speed_pid.reset()
        self.right_speed_pid.reset()
        self.steering_pid.reset()
        
        # 启动平衡控制定时器
        self.balance_ticker.start(1000 // CONTROL_FREQUENCY)  # 毫秒计算
        # 主循环仅处理非实时任务
        while self.is_running:
            # 更新CCD数据显示和系统状态
            self.lcd_display.update_ccd_display(self.ccd_tracker)
            self.lcd_display.show_system_status(
                pitch=self.pitch_filter.get_last_filtered_pitch(),
                left_speed=self.left_motor.get_speed(1/CONTROL_FREQUENCY),
                right_speed=self.right_motor.get_speed(1/CONTROL_FREQUENCY),
                voltage=self.read_battery_voltage(),
                is_running=self.is_running
            )
            
            # 检查拨码开关状态
            if switch2.value() != state2:
                print("测试程序停止")
                break
            
            time.sleep_ms(100)
        gc.collect()
    
    def stop(self):
        """停止平衡小车"""
        if self.is_running:
            print("Stopping balance car...")
            self.is_running = False
            self.left_motor.set_pwm(0)  # 停止左侧电机
            self.right_motor.set_pwm(0)  # 停止右侧电机
            self.balance_ticker.stop()  # 停止定时器
            self.lcd_display.draw_text(100, 244, "Stopped", size=16, color=0xF800)
    
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
    
    # 可选：主控调用CCD循迹
    def get_ccd_offset(self):
        """获取CCD循迹偏差"""
        return self.ccd_tracker.get_line_offset()


# 主程序入口
if __name__ == "__main__":
    # 创建平衡小车控制器
    car_controller = BalanceCarController()
    
    # 启动平衡小车
    car_controller.start()

