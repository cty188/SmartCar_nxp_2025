import math
import time
from seekfree import IMU660RX
from MahonyFilter import MahonyFilter
from machine import Pin
from smartcar import ticker
import gc

class IMUProcessor:
    """IMU数据处理封装类 - 使用Mahony滤波器"""
    
    def __init__(self, sample_rate=100, kp=0.5, ki=0.01):
        """
        初始化IMU处理器
        :param sample_rate: 采样率 (Hz)
        :param kp: Mahony滤波器比例增益
        :param ki: Mahony滤波器积分增益
        """
        # 硬件初始化
        self.imu = IMU660RX()  # 默认分频数为1
        
        # 创建并配置Ticker
        self.pit = ticker(1)  # 使用ticker1
        self.pit.capture_list(self.imu)  # 关联IMU到ticker
        self.pit.start(10)  # 启动10ms周期的ticker
        
        # 获取数据引用（数据会自动更新）
        self.imu_data = self.imu.get()
        
        # Mahony滤波器
        dt = 1.0 / sample_rate
        self.filter = MahonyFilter(Kp=kp, Ki=ki, dt=dt)
        
        # 原始传感器数据
        self.raw_acc = [0, 0, 0]
        self.raw_gyro = [0, 0, 0]
        
        # 处理后的数据
        self.attitude_deg = [0.0, 0.0, 0.0]  # Roll, Pitch, Yaw (度)
        self.accel = [0, 0, 0]
        self.gyro = [0, 0, 0]
        
        # 传感器校准数据
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.calibrated = False
        
        # 转换系数（根据实际传感器规格）
        # 示例值：±2g量程时，16384 LSB/g；±2000dps量程时，16.4 LSB/dps
        self.acc_scale = 16384.0  # LSB/g
        self.gyro_scale = 16.4    # LSB/dps
        
        # 用于计算变化量的前一次角度值
        self.prev_angles = [0.0, 0.0, 0.0]
    
    def stop(self):
        """停止数据采集"""
        self.pit.stop()
    
    def calibrate_gyro(self, samples=500, max_retries=3):
        """
        执行陀螺仪零偏校准，仅检查与期望值的偏差
        :param samples: 校准采样次数
        :param max_retries: 最大重试次数
        """
        # 定义期望的校准值
        expected_acc_bias = [1980, -141.5, 3459.4]
        expected_gyro_bias = [0.8580, -7.3568, 1.2432]
        
        retry_count = 0
        calibration_success = False
        
        while not calibration_success and retry_count < max_retries:
            print(f"正在校准陀螺仪零偏（尝试 {retry_count+1}/{max_retries}），请保持设备静止...")
            
            # 初始化累加器
            gyro_sum = [0.0, 0.0, 0.0]
            acc_sum = [0.0, 0.0, 0.0]
            
            # 收集样本
            for i in range(samples):
                # 直接从已关联的数据列表获取
                acc_x, acc_y, acc_z = self.imu_data[0], self.imu_data[1], self.imu_data[2]
                gyro_x, gyro_y, gyro_z = self.imu_data[3], self.imu_data[4], self.imu_data[5]

                # 累加数据
                acc_sum[0] += acc_x
                acc_sum[1] += acc_y
                acc_sum[2] += acc_z
                gyro_sum[0] += gyro_x
                gyro_sum[1] += gyro_y
                gyro_sum[2] += gyro_z

                # 等待下一采样周期
                time.sleep(self.filter.dt)
            
            # 计算平均值作为零偏
            acc_bias = [acc_sum[0] / samples, acc_sum[1] / samples, acc_sum[2] / samples]
            gyro_bias = [gyro_sum[0] / samples, gyro_sum[1] / samples, gyro_sum[2] / samples]
            
            # 检查与期望值的偏差
            valid, failures = self._check_deviation(acc_bias, gyro_bias, expected_acc_bias, expected_gyro_bias)
            
            if valid:
                # 保存有效的校准值
                self.acc_bias = acc_bias
                self.gyro_bias = gyro_bias
                self.filter.init_with_accel(self.acc_bias)
                
                print(f"校准成功 - 加速度计零偏: X={acc_bias[0]:.4f}, Y={acc_bias[1]:.4f}, Z={acc_bias[2]:.4f}")
                print(f"陀螺仪零偏: X={gyro_bias[0]:.4f}, Y={gyro_bias[1]:.4f}, Z={gyro_bias[2]:.4f}")
                
                calibration_success = True
                self.calibrated = True
            else:
                # 输出失败信息
                print("校准失败原因:")
                for failure in failures:
                    print(failure)
                
                print("将重新校准...")
                retry_count += 1
                time.sleep(1)  # 等待1秒后重试
        
        if not calibration_success:
            print(f"校准失败！经过 {max_retries} 次尝试仍无法获得有效校准值")
            print("使用期望校准值作为后备方案")
            self._use_expected_calibration(expected_acc_bias, expected_gyro_bias)

    def _check_deviation(self, acc_bias, gyro_bias, expected_acc, expected_gyro):
        """
        检查测量值与期望值的偏差
        :return: (是否有效, 失败信息列表)
        """
        valid = True
        failures = []
        ACC_ALLOWED_DEVIATION = 0.10  # 10%
        GYRO_ALLOWED_DEVIATION = 3.00 # 300%
        
        # 检查加速度计偏差
        for i, axis in enumerate(['X', 'Y', 'Z']):
            # 计算偏差百分比
            if abs(expected_acc[i]) > 1e-6:  # 避免除以零
                deviation = abs(acc_bias[i] - expected_acc[i]) / abs(expected_acc[i])
                if deviation > ACC_ALLOWED_DEVIATION:
                    valid = False
                    failures.append(
                        "加速度计%s轴偏差过大: %.2f%% (测量值=%.4f, 期望值=%.4f)" % 
                        (axis, deviation*100, acc_bias[i], expected_acc[i])
                    )    
        # 检查陀螺仪偏差
        for i, axis in enumerate(['X', 'Y', 'Z']):
            # 计算偏差百分比
            if abs(expected_gyro[i]) > 1e-6:  # 避免除以零
                deviation = abs(gyro_bias[i] - expected_gyro[i]) / abs(expected_gyro[i])
                if deviation > GYRO_ALLOWED_DEVIATION:
                    valid = False
                    failures.append(
                        "陀螺仪%s轴偏差过大: %.2f%% (测量值=%.4f, 期望值=%.4f)" % 
                        (axis, deviation*100, gyro_bias[i], expected_gyro[i])
                    )        
        return valid, failures

    def _use_expected_calibration(self, expected_acc, expected_gyro):
        """使用期望校准值作为后备方案"""
        self.acc_bias = expected_acc
        self.gyro_bias = expected_gyro
        
        # 初始化滤波器
        self.filter.init_with_accel(self.acc_bias)
        
        print("使用期望校准值:")
        print(f"加速度计零偏: X={self.acc_bias[0]}, Y={self.acc_bias[1]}, Z={self.acc_bias[2]}")
        print(f"陀螺仪零偏: X={self.gyro_bias[0]}, Y={self.gyro_bias[1]}, Z={self.gyro_bias[2]}")
        
        self.calibrated = True
    def convert_units(self):
        """
        转换原始传感器数据到合适单位
        :return: (加速度计m/s², 陀螺仪度/秒)
        """
        # 直接从已关联的数据列表获取
        acc_x, acc_y, acc_z = self.imu_data[0], self.imu_data[1], self.imu_data[2]
        gyro_x, gyro_y, gyro_z = self.imu_data[3], self.imu_data[4], self.imu_data[5]
        
        # 应用校准数据
        if self.calibrated:
            gyro_x -= self.gyro_bias[0]
            gyro_y -= self.gyro_bias[1]
            gyro_z -= self.gyro_bias[2]
        
        # 加速度计转换：原始值 -> g -> m/s²
        acc_x_g = acc_x / self.acc_scale
        acc_y_g = acc_y / self.acc_scale
        acc_z_g = acc_z / self.acc_scale
        acc_x_ms2 = acc_x_g * 9.80665
        acc_y_ms2 = acc_y_g * 9.80665
        acc_z_ms2 = acc_z_g * 9.80665
        
        # 陀螺仪转换：原始值 -> dps
        gyro_x_dps = gyro_x / self.gyro_scale
        gyro_y_dps = gyro_y / self.gyro_scale
        gyro_z_dps = gyro_z / self.gyro_scale
        
        return [acc_x_ms2, acc_y_ms2, acc_z_ms2], [gyro_x_dps, gyro_y_dps, gyro_z_dps]
    
    def update(self):
        """更新IMU数据并处理姿态估计"""
        # 转换到合适单位
        self.accel, self.gyro = self.convert_units()
        
        # 保存原始数据（未转换单位）
        self.raw_acc = [self.imu_data[0], self.imu_data[1], self.imu_data[2]]
        self.raw_gyro = [self.imu_data[3], self.imu_data[4], self.imu_data[5]]

        # 输入到Mahony滤波器
        self.filter.input_data(self.gyro, self.accel)
        
        # 执行滤波器更新
        self.filter.update()
        
        # 获取姿态角
        pitch, roll, yaw = self.filter.get_attitude_degrees()
        self.attitude_deg = [roll, pitch, yaw]
        
        # 返回角度变化量
        delta_roll = roll - self.prev_angles[0]
        delta_pitch = pitch - self.prev_angles[1]
        delta_yaw = yaw - self.prev_angles[2]
        
        # 保存当前角度作为下一次的前值
        self.prev_angles = [roll, pitch, yaw]
        
        return [delta_roll, delta_pitch, delta_yaw]
    
    def get_roll(self):
        """获取滚转角 (度)"""
        return self.attitude_deg[0]
    
    def get_pitch(self):
        """获取俯仰角 (度)"""
        return self.attitude_deg[1]
    
    def get_yaw(self):
        """获取偏航角 (度)"""
        return self.attitude_deg[2]
    
    def get_raw_acc(self):
        """获取原始加速度计数据 (LSB)"""
        return self.accel
    
    def get_raw_gyro(self):
        """获取原始陀螺仪数据 (LSB)"""
        return self.gyro
    
    def get_quaternion(self):
        """获取四元数"""
        return self.filter.q0, self.filter.q1, self.filter.q2, self.filter.q3
    
    def get_attitude_deg(self):
        """获取完整姿态数据 (度)"""
        return self.attitude_deg
    
    def reset_orientation(self, pitch=0.0, roll=0.0, yaw=0.0):
        """重置姿态估计"""
        self.filter.reset(pitch, roll, yaw)

def main():
    # 初始化IMU处理器
    imu = IMUProcessor(sample_rate=100)
    
    # 校准陀螺仪
    imu.calibrate_gyro()
    
    print("\n开始数据采集（按Ctrl+C停止）...")
    # 添加标题行
    print("AccX\tAccY\tAccZ\tGyroX\tGyroY\tGyroZ\tRoll\tPitch\tYaw\tΔRoll\tΔPitch\tΔYaw")
    
    try:
        while True:
            # 更新IMU数据并获取角度变化量
            delta_angles = imu.update()
            
            # 获取原始传感器数据
            raw_acc = imu.get_raw_acc()
            raw_gyro = imu.get_raw_gyro()
            
            # 获取当前姿态角度
            roll, pitch, yaw = imu.get_attitude_deg()
            
            # 获取角度变化量
            delta_roll, delta_pitch, delta_yaw = delta_angles
            
            # 格式化输出所有数据
            print(("%d\t"*6 + "%.2f\t"*6) % (
                raw_acc[0], raw_acc[1], raw_acc[2],
                raw_gyro[0], raw_gyro[1], raw_gyro[2],
                roll, pitch, yaw,
                delta_roll, delta_pitch, delta_yaw
            ))
            
            # 控制输出频率（约10Hz）
            time.sleep(0.01)
            gc.collect()  # 垃圾回收
            
    except KeyboardInterrupt:
        print("\n测试结束")
        imu.stop()  # 停止数据采集

if __name__ == "__main__":
    main()

