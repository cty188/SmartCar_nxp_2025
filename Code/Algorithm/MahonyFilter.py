import math 
class MahonyFilter:
    """Mahony滤波器Python实现（无Numpy依赖）"""
    
    def __init__(self, Kp=0.5, Ki=0.01, dt=0.01):
        """
        初始化Mahony滤波器
        :param Kp: 比例增益
        :param Ki: 积分增益
        :param dt: 采样时间间隔(秒)
        """
        # 滤波器参数
        self.Kp = Kp
        self.Ki = Ki
        self.dt = dt
        
        # 状态变量 - 初始化为默认水平位置
        self.q0 = 1.0  # 四元数w
        self.q1 = 0.0  # 四元数x
        self.q2 = 0.0  # 四元数y
        self.q3 = 0.0  # 四元数z
        
        # 误差积分项
        self.exInt = 0.0
        self.eyInt = 0.0
        self.ezInt = 0.0
        
        # 旋转矩阵 - 初始化为单位矩阵
        self.rMat = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]
        
        # 输出姿态角 - 初始化为0
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        
        # 传感器数据 - 初始化为0
        self.gyro = [0.0, 0.0, 0.0]  # [x, y, z]
        self.acc = [0.0, 0.0, 0.0]   # [x, y, z]
        
        # 初始校准标志 - 表示是否完成基于加速度计的初始姿态估计
        self.initialized = False
        
        # 初始化计数器 - 用于收集初始样本
        self.init_samples = 20
        self.init_count = 0
        self.acc_accumulator = [0.0, 0.0, 0.0]
    
    def rotation_matrix_update(self):
        """更新旋转矩阵"""
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        
        # 计算四元数乘积项
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        
        q0q1 = q0 * q1
        q0q2 = q0 * q2
        q0q3 = q0 * q3
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q2q3 = q2 * q3
        
        # 更新旋转矩阵
        self.rMat[0][0] = 1.0 - 2.0 * q2q2 - 2.0 * q3q3
        self.rMat[0][1] = 2.0 * (q1q2 - q0q3)
        self.rMat[0][2] = 2.0 * (q1q3 + q0q2)
        
        self.rMat[1][0] = 2.0 * (q1q2 + q0q3)
        self.rMat[1][1] = 1.0 - 2.0 * q1q1 - 2.0 * q3q3
        self.rMat[1][2] = 2.0 * (q2q3 - q0q1)
        
        self.rMat[2][0] = 2.0 * (q1q3 - q0q2)
        self.rMat[2][1] = 2.0 * (q2q3 + q0q1)
        self.rMat[2][2] = 1.0 - 2.0 * q1q1 - 2.0 * q2q2
    
    def vector_cross(self, a, b):
        """计算两个向量的叉积"""
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        ]
    
    def vector_norm(self, v):
        """计算向量的模"""
        return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    
    def vector_normalize(self, v):
        """归一化向量"""
        norm = self.vector_norm(v)
        if norm > 1e-6:  # 避免除以零
            return [v[0]/norm, v[1]/norm, v[2]/norm]
        return v
    
    def init_with_accel(self, accel):
        """
        使用加速度计初始化姿态
        :param accel: 加速度计数据 [x, y, z]
        """
        # 计算重力方向
        norm = self.vector_norm(accel)
        if norm < 1e-6:
            return
        
        ax, ay, az = [v/norm for v in accel]
        
        # 计算初始俯仰角 (确保分母不为零)
        denominator = math.sqrt(ax*ax + az*az)
        if denominator < 1e-6:
            roll = math.copysign(math.pi/2, ay)
        else:
            roll = math.atan2(ay, denominator)
        
        # 计算初始滚转角
        pitch = math.atan2(-ax, az) if abs(az) > 1e-6 else 0.0
        
        # 设置初始姿态
        self.reset(pitch, roll, 0.0)
        self.initialized = True
    
    def input_data(self, gyro, acc):
        """
        输入传感器数据
        :param gyro: 陀螺仪数据 [x, y, z] (度/秒)
        :param acc: 加速度计数据 [x, y, z] (任意单位)
        """
        self.gyro = list(gyro)
        self.acc = list(acc)
        
        # 执行初始姿态计算（如果未完成初始化）
        if not self.initialized:
            # 收集加速度计样本
            self.acc_accumulator[0] += acc[0]
            self.acc_accumulator[1] += acc[1]
            self.acc_accumulator[2] += acc[2]
            self.init_count += 1
            
            # 收集足够样本后执行初始化
            if self.init_count >= self.init_samples:
                avg_acc = [v/self.init_samples for v in self.acc_accumulator]
                self.init_with_accel(avg_acc)
    
    def update(self):
        """执行Mahony滤波更新"""
        # 如果尚未完成初始姿态估计，跳过更新
        if not self.initialized:
            return
        
        # 归一化加速度计数据
        acc_norm = self.vector_norm(self.acc)
        if acc_norm > 1e-6:  # 避免除以零
            self.acc = [x/acc_norm for x in self.acc]
        
        # 计算重力方向误差
        # 估计的重力方向是旋转矩阵的第三行
        estimated_gravity = self.rMat[2]
        
        # 计算加速度计测量值与估计重力方向的叉积
        error = self.vector_cross(self.acc, estimated_gravity)
        ex, ey, ez = error
        
        # 积分误差
        self.exInt += self.Ki * ex * self.dt
        self.eyInt += self.Ki * ey * self.dt
        self.ezInt += self.Ki * ez * self.dt
        
        # 修正陀螺仪读数
        gyro_corrected = [
            self.gyro[0] + self.Kp * ex + self.exInt,
            self.gyro[1] + self.Kp * ey + self.eyInt,
            self.gyro[2] + self.Kp * ez + self.ezInt
        ]
        
        # 将陀螺仪数据从度/秒转换为弧度/秒
        gyro_rad = [
            math.radians(gyro_corrected[0]),
            math.radians(gyro_corrected[1]),
            math.radians(gyro_corrected[2])
        ]
        gx, gy, gz = gyro_rad
        
        # 四元数导数 (一阶近似)
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        half_dt = 0.5 * self.dt
        
        # 四元数更新
        self.q0 += (-q1 * gx - q2 * gy - q3 * gz) * half_dt
        self.q1 += ( q0 * gx + q2 * gz - q3 * gy) * half_dt
        self.q2 += ( q0 * gy - q1 * gz + q3 * gx) * half_dt
        self.q3 += ( q0 * gz + q1 * gy - q2 * gx) * half_dt
        
        # 归一化四元数
        q_norm = math.sqrt(self.q0**2 + self.q1**2 + self.q2**2 + self.q3**2)
        if q_norm > 1e-6:  # 避免除以零
            self.q0 /= q_norm
            self.q1 /= q_norm
            self.q2 /= q_norm
            self.q3 /= q_norm
        
        # 更新旋转矩阵
        self.rotation_matrix_update()
        
        # 更新姿态角
        self.update_attitude()
    
    def update_attitude(self):
        """从旋转矩阵计算姿态角"""
        # 旋转矩阵元素
        r20 = self.rMat[2][0]
        r21 = self.rMat[2][1]
        r22 = self.rMat[2][2]
        r10 = self.rMat[1][0]
        r00 = self.rMat[0][0]
        
        # 计算俯仰角 (pitch)
        self.pitch = -math.asin(max(-1.0, min(1.0, r20)))  # 限制在[-1,1]范围内
        
        # 计算滚转角 (roll)
        self.roll = math.atan2(r21, r22)
        
        # 计算偏航角 (yaw)
        self.yaw = math.atan2(r10, r00)
    
    def get_attitude_degrees(self):
        """获取姿态角（度）"""
        pitch_deg = math.degrees(self.pitch)
        roll_deg = math.degrees(self.roll)
        yaw_deg = math.degrees(self.yaw)
        return pitch_deg, roll_deg, yaw_deg
    
    def get_attitude_radians(self):
        """获取姿态角（弧度）"""
        return self.pitch, self.roll, self.yaw
    
    def reset(self, pitch=0.0, roll=0.0, yaw=0.0):
        """重置滤波器状态"""
        # 将欧拉角转换为四元数
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        self.q0 = cr * cp * cy + sr * sp * sy
        self.q1 = sr * cp * cy - cr * sp * sy
        self.q2 = cr * sp * cy + sr * cp * sy
        self.q3 = cr * cp * sy - sr * sp * cy
        
        # 归一化四元数
        q_norm = math.sqrt(self.q0**2 + self.q1**2 + self.q2**2 + self.q3**2)
        if q_norm > 1e-6:
            self.q0 /= q_norm
            self.q1 /= q_norm
            self.q2 /= q_norm
            self.q3 /= q_norm
        
        # 重新生成旋转矩阵
        self.rotation_matrix_update()
        
        # 重置误差积分项
        self.exInt = 0.0
        self.eyInt = 0.0
        self.ezInt = 0.0
        
        # 更新姿态角
        self.update_attitude()
    
    def print_state(self):
        """打印当前滤波器状态（用于调试）"""
        print(f"初始化状态: {'完成' if self.initialized else '进行中'}")
        print(f"四元数: [{self.q0:.4f}, {self.q1:.4f}, {self.q2:.4f}, {self.q3:.4f}]")
        print(f"姿态角: Pitch={math.degrees(self.pitch):.2f}°, Roll={math.degrees(self.roll):.2f}°, Yaw={math.degrees(self.yaw):.2f}°")
        print(f"误差积分: exInt={self.exInt:.4f}, eyInt={self.eyInt:.4f}, ezInt={self.ezInt:.4f}")
    
    def mohony_calculate(self, gyro_data, accel_data):
        """简化的更新方法：输入数据、更新状态并返回姿态角"""
        # 输入数据到滤波器
        self.input_data(gyro_data, accel_data)
        
        # 只有完成初始化后才执行更新
        if self.initialized:
            self.update()
            
        # 获取姿态角（如果未初始化则返回默认值）
        if self.initialized:
            return self.get_attitude_degrees()
        else:
            # 返回初始水平姿态（或收集更多样本）
            return 0.0, 0.0, 0.0