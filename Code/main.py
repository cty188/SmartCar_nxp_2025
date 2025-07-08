# 本示例程序演示如何通过 boot.py 文件进行 soft-boot 控制后执行自己的源文件
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的拨码开关控制

# 示例程序运行效果为复位后执行本文件 通过 C18 电平状态跳转执行 user_main.py
# C4 LED 会一秒周期闪烁

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
        self.base_target_pitch=9.5# 基础平衡俯仰角
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
            Kp=0.0,#-0.01,
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
        # 注意: Mahony滤波已经提供了角度估计，但卡尔曼滤波可以提供更平滑的结果
        
        filtered_pitch = self.pitch_filter.update(
            pitch_rate=raw_gyro[1],  # 假设Y轴是俯仰轴
            pitch_measurement=pitch_angle
        )
        
        # 4. 速度环外部：目标速度->期望倾角
        # 期望倾角 = speedtoangle_pid(实际速度, 目标速度)
        target_speed = 1.0
        current_speed = (current_left_speed + current_right_speed) / 2.0
        delta_pitch = self.speed_to_angle_pid.compute(
            feedback=current_speed,
            setpoint=self.target_speed
        )

        # 4. 串级PID控制：外环角度PID -> 内环速度PID
        # 计算速度控制输出 (外环)
        target_speed_from_angle = self.angle_pid.compute(
            feedback=filtered_pitch,
            setpoint=self.base_target_pitch #+ delta_pitch
        )

        # 5. 计算转向控制输出
        #current_yaw = self.imu_processor.get_yaw()
        self.target_heading = self.ccd_tracker.get_line_angle(0)
        target_speed_from_steering = self.steering_pid.compute(
            feedback=0,
            setpoint=self.target_heading
        )
        # 合并遥控目标速度
        #print(target_speed_from_steering)
        left_target_speed  = target_speed_from_angle  - target_speed_from_steering
        right_target_speed = target_speed_from_angle + target_speed_from_steering

        # 6. 计算电机控制输出
        left_motor_output = self.left_speed_pid.compute(
            feedback=current_left_speed,
            setpoint=left_target_speed
        )
        # 右轮速度PID控制
        right_motor_output = self.right_speed_pid.compute(
            feedback=current_right_speed,
            setpoint=right_target_speed
        )
        # 8. 设置电机输出
        self.left_motor.set_pwm(left_motor_output)
        self.right_motor.set_pwm(right_motor_output)

        # 9. 调试输出
        if self.debug_enabled and self.loop_count % self.debug_interval == 0:
            print(f"实际俯仰角: {filtered_pitch:.2f}°, 目标俯仰角: {raw_gyro[1]:.2f}°")
            #print(f"当前左侧速度: {current_left_speed:.2f}m/s, 当前右侧速度: {current_right_speed:.2f}m/s")
            #print(f"速度调整量: {target_pitch:.2f}°")
            #print(f"平衡输出: {balance_output:.1f}, 转向输出: {steering_output:.1f}")
            #print(f"电机PWM: 左={left_motor_output:.1f}, 右={right_motor_output:.1f}")

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

    def init_lcd(self):
        """初始化LCD屏幕用于显示CCD数据"""
        from display import LCD, LCD_Drv
        cs = Pin('C5', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
        cs.high()
        cs.low()
        rst = Pin('B9', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
        dc = Pin('B8', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
        blk = Pin('C4', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
        drv = LCD_Drv(SPI_INDEX=1, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
        lcd = LCD(drv)
        lcd.color(0xFFFF, 0x0000)
        lcd.mode(2)
        lcd.clear(0x0000)
        self.lcd = lcd
        print("LCD初始化完成")

    def update_ccd_display(self):
        """在LCD上显示CCD双通道数据波形，并输出偏差角"""
        if not hasattr(self, 'lcd'):
            return
        # 通道0波形
        data0 = self.ccd_tracker.get_raw_data(0)
        self.lcd.wave(0, 0, len(data0), 64, data0, max=4095)
        angle0 = self.ccd_tracker.get_line_angle(0)
        # 通道1波形（下半屏）
        if len(self.ccd_tracker.channels) > 1:
            data1 = self.ccd_tracker.get_raw_data(1)
            self.lcd.wave(0, 64, len(data1), 64, data1, max=4095)
            angle1 = self.ccd_tracker.get_line_angle(1)
        else:
            angle1 = None
        # 如需在屏幕显示角度，可根据实际LCD驱动方法补充
        # 例如: self.lcd.draw_string(0, 70, "A0: %.2f° A1: %.2f°" % (angle0, angle1 if angle1 is not None else 0), 0xFFFF, 0x0000)

    def start(self):
        """启动平衡小车"""
        print("启动平衡小车...")
        self.is_running = True
        self.init_lcd()  # 初始化LCD
        
        # 重置滤波器状态
        self.pitch_filter.reset()
        self.angle_pid.reset()
        self.left_speed_pid.reset()
        self.right_speed_pid.reset()
        self.steering_pid.reset()
        
        # 启动平衡控制定时器
        self.balance_ticker.start(1000 // CONTROL_FREQUENCY)  # 毫秒计算
        try:
            # 主循环仅处理非实时任务
            while self.is_running:
                # 显示CCD双通道数据和角度
                self.update_ccd_display()
                # 实时打印CCD双通道角度偏差
                #for idx, ch in enumerate(self.ccd_tracker.channels):
                    #angle = self.ccd_tracker.get_line_angle(idx)
                    #print(f"[CCD{ch}] 当前角度偏差: {angle:.2f}°")
                #print(f"[CCD{0}] 当前角度偏差: {self.ccd_tracker.get_line_angle(0):.2f}°")
                if switch2.value() != state2:
                    print("Test program stop.")
                    break
                time.sleep_ms(100)
        except Exception as e:
            print("Exception occurred:", e)
        gc.collect()
    
    def stop(self):
        """停止平衡小车"""
        print("停止平衡小车...")
        self.is_running = False
        self.left_motor.set_pwm(0)  # 停止左侧电机
        self.right_motor.set_pwm(0)  # 停止右侧电机
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