
import math
import time
import random

try:
    # 尝试加载完整版random函数
    from random import gauss
except ImportError:
    # 在gauss不可用时提供替代实现
    def gauss(mu, sigma):
        """Box-Muller变换生成高斯分布随机数"""
        import math
        u1 = random.random()
        u2 = random.random()
        if u1 < 1e-15:
            u1 = 1e-15  # 防止log(0)
        z0 = math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)
        return mu + z0 * sigma
    
class Matrix:
    """
    矩阵类，用于表示和操作二维数值数组
    """
    def __init__(self, rows, cols, data=None):
        """
        矩阵初始化
        :param rows: 矩阵行数
        :param cols: 矩阵列数
        :param data: 可选的初始化数据（二维列表）
        """
        self.rows = rows
        self.cols = cols
        
        # 如果没有提供数据，创建全零矩阵
        if data is None:
            self.data = [[0.0] * cols for _ in range(rows)]
        else:
            self.data = data
    
    def __str__(self):
        """以可读格式返回矩阵字符串表示"""
        return '\n'.join([' '.join([f'{x:.4f}' for x in row]) for row in self.data])
    
    def __getitem__(self, index):
        """支持索引访问矩阵行"""
        return self.data[index]
    
    def __setitem__(self, index, value):
        """支持索引设置矩阵行"""
        self.data[index] = value

def matrix_init(matrix, rows, cols, data=None):
    """
    矩阵初始化函数
    :param matrix: 要初始化的矩阵对象
    :param rows: 行数
    :param cols: 列数
    :param data: 可选的数据
    """
    if data is not None:
        matrix.data = data
    else:
        # 创建全零矩阵
        matrix.data = [[0.0] * cols for _ in range(rows)]
    matrix.rows = rows
    matrix.cols = cols

def matrix_multiply(a, b):
    """
    矩阵乘法
    :param a: 左侧矩阵
    :param b: 右侧矩阵
    :return: a*b的结果矩阵
    """
    # 验证矩阵维度是否兼容
    if a.cols != b.rows:
        raise ValueError("矩阵维度不兼容，无法相乘")
    
    # 创建结果矩阵
    result = Matrix(a.rows, b.cols)
    
    # 三重循环实现矩阵乘法
    for i in range(a.rows):
        for j in range(b.cols):
            sum_val = 0.0
            for k in range(a.cols):
                sum_val += a.data[i][k] * b.data[k][j]
            result.data[i][j] = sum_val
    
    return result

def matrix_add(a, b):
    """
    矩阵加法
    :param a: 第一个矩阵
    :param b: 第二个矩阵
    :return: a+b的结果矩阵
    """
    # 验证矩阵维度是否相同
    if a.rows != b.rows or a.cols != b.cols:
        raise ValueError("矩阵维度不兼容，无法相加")
    
    # 创建结果矩阵
    result = Matrix(a.rows, a.cols)
    
    # 逐元素相加
    for i in range(a.rows):
        for j in range(a.cols):
            result.data[i][j] = a.data[i][j] + b.data[i][j]
    
    return result

def matrix_subtract(a, b):
    """
    矩阵减法
    :param a: 被减矩阵
    :param b: 减数矩阵
    :return: a-b的结果矩阵
    """
    # 验证矩阵维度是否相同
    if a.rows != b.rows or a.cols != b.cols:
        raise ValueError("矩阵维度不兼容，无法相减")
    
    # 创建结果矩阵
    result = Matrix(a.rows, a.cols)
    
    # 逐元素相减
    for i in range(a.rows):
        for j in range(a.cols):
            result.data[i][j] = a.data[i][j] - b.data[i][j]
    
    return result

def matrix_transpose(a):
    """
    矩阵转置
    :param a: 要转置的矩阵
    :return: a的转置矩阵
    """
    # 创建转置矩阵，行数列数互换
    result = Matrix(a.cols, a.rows)
    
    # 行列交换
    for i in range(a.rows):
        for j in range(a.cols):
            result.data[j][i] = a.data[i][j]
    
    return result

def matrix_inverse(a):
    """
    矩阵求逆（使用高斯-约当消元法）
    :param a: 要求逆的矩阵
    :return: a的逆矩阵
    """
    # 验证矩阵是否为方阵
    if a.rows != a.cols:
        raise ValueError("只有方阵才能求逆")
    
    n = a.rows
    
    # 创建增广矩阵[A|I]，大小为n x 2n
    aug = Matrix(n, 2*n)
    
    # 填充矩阵左侧部分A
    for i in range(n):
        for j in range(n):
            aug.data[i][j] = a.data[i][j]
        # 右侧部分填充单位矩阵
        aug.data[i][n+i] = 1.0
    
    # 高斯-约当消元法
    for i in range(n):
        # 寻找主元（当前列中绝对值最大的行）
        max_row = i
        for k in range(i+1, n):
            if abs(aug.data[k][i]) > abs(aug.data[max_row][i]):
                max_row = k
        
        # 交换当前行与主元行
        aug.data[i], aug.data[max_row] = aug.data[max_row], aug.data[i]
        
        # 检查主元是否为0（奇异矩阵）
        if abs(aug.data[i][i]) < 1e-10:
            raise ValueError("矩阵奇异，无法求逆")
        
        # 将主元缩放为1
        pivot = aug.data[i][i]
        for j in range(2*n):
            aug.data[i][j] /= pivot
        
        # 使用当前行消去其他行
        for k in range(n):
            if k != i:
                factor = aug.data[k][i]
                for j in range(2*n):
                    aug.data[k][j] -= factor * aug.data[i][j]
    
    # 从增广矩阵中提取逆矩阵
    inv = Matrix(n, n)
    for i in range(n):
        for j in range(n):
            # 右侧部分就是逆矩阵
            inv.data[i][j] = aug.data[i][j+n]
    
    return inv

class KalmanFilter:
    """
    卡尔曼滤波器类，实现完整卡尔曼滤波算法
    """
    def __init__(self, xhat_size, u_size, z_size):
        """
        卡尔曼滤波器初始化
        :param xhat_size: 状态向量维度
        :param u_size: 控制向量维度
        :param z_size: 测量向量维度
        """
        self.xhat_size = xhat_size  # 状态向量维数
        self.u_size = u_size        # 控制向量维数
        self.z_size = z_size        # 测量向量维数
        
        # 测量相关设置
        self.UseAutoAdjustment = False  # 是否启用自动调整功能
        self.MeasurementValidNum = 0    # 当前有效的测量数量
        self.MeasurementMap = [0] * z_size      # 每个测量对应的状态索引（从1开始）
        self.MeasurementDegree = [0.0] * z_size # 测量项的系数（影响H矩阵）
        self.MatR_DiagonalElements = [0.0] * z_size  # R矩阵对角线元素（测量噪声方差）
        self.StateMinVariance = [0.0] * xhat_size    # 状态最小方差（防止过度收敛）
        self.temp = [0] * z_size           # 临时数组，用于自动调整
        
        # 滤波器输入输出数据
        self.FilteredValue = [0.0] * xhat_size  # 滤波后状态向量（易于访问的形式）
        self.MeasuredVector = [0.0] * z_size    # 测量值向量（用户输入）
        self.ControlVector = [0.0] * u_size     # 控制值向量（用户输入）
        
        # 核心矩阵初始化 #
        # 状态向量
        self.xhat = Matrix(xhat_size, 1)          # x(k|k) - 后验估计
        self.xhatminus = Matrix(xhat_size, 1)     # x(k|k-1) - 先验估计
        
        # 控制相关矩阵
        if u_size > 0:
            self.u = Matrix(u_size, 1)      # 控制向量
            self.B = Matrix(xhat_size, u_size)  # 控制矩阵
        else:
            self.u = None
            self.B = None
        
        # 测量相关矩阵
        self.z = Matrix(z_size, 1)        # 测量向量
        self.H = Matrix(z_size, xhat_size) # 测量矩阵
        self.HT = Matrix(xhat_size, z_size) # H的转置
        self.R = Matrix(z_size, z_size)    # 测量噪声协方差矩阵
        
        # 状态协方差矩阵
        self.P = Matrix(xhat_size, xhat_size)     # P(k|k) - 后验协方差
        self.Pminus = Matrix(xhat_size, xhat_size) # P(k|k-1) - 先验协方差
        
        # 状态转移相关矩阵
        self.F = Matrix(xhat_size, xhat_size)   # 状态转移矩阵
        self.FT = Matrix(xhat_size, xhat_size)  # F的转置
        self.Q = Matrix(xhat_size, xhat_size)   # 过程噪声协方差矩阵
        
        # 卡尔曼增益矩阵
        self.K = Matrix(xhat_size, z_size)       # 卡尔曼增益
        
        # 临时存储矩阵（减少内存分配开销）
        self.S = Matrix(xhat_size, xhat_size)             # 中间计算矩阵
        self.temp_matrix = Matrix(xhat_size, xhat_size)    # 临时矩阵1
        self.temp_matrix1 = Matrix(xhat_size, xhat_size)   # 临时矩阵2
        self.temp_vector = Matrix(xhat_size, 1)            # 临时向量1
        self.temp_vector1 = Matrix(xhat_size, 1)           # 临时向量2
        
        # 步骤跳过标志（用于扩展KF算法）
        self.SkipEq1 = False  # 跳过方程1（状态预测）
        self.SkipEq2 = False  # 跳过方程2（协方差预测）
        self.SkipEq3 = False  # 跳过方程3（卡尔曼增益计算）
        self.SkipEq4 = False  # 跳过方程4（状态更新）
        self.SkipEq5 = False  # 跳过方程5（协方差更新）
        
        # 用户自定义函数（可用于扩展功能）
        self.User_Func0_f = None  # 测量处理后的回调
        self.User_Func1_f = None  # 状态预测后的回调
        self.User_Func2_f = None  # 协方差预测后的回调
        self.User_Func3_f = None  # 卡尔曼增益计算后的回调
        self.User_Func4_f = None  # 状态更新后的回调
        self.User_Func5_f = None  # 协方差更新后的回调
        self.User_Func6_f = None  # 最终结果输出前的回调

    def H_K_R_Adjustment(self):
        """修复后的自动调整方法"""
        self.MeasurementValidNum = 0
        
        # 安全的复制方式
        z_data = self.MeasuredVector.copy()
        self.MeasuredVector = [0.0] * self.z_size
        
        # 每次调整前初始化矩阵
        self.R = Matrix(self.z_size, self.z_size)
        self.H = Matrix(self.z_size, self.xhat_size)
        self.z = Matrix(self.z_size, 1)  # 确保尺寸正确
        
        valid_indices = []  # 记录有效测量的索引
        for i in range(self.z_size):
            if z_data[i] != 0:  # 非零表示有效测量
                # 安全赋值
                if i < len(self.z.data):
                    self.z.data[i][0] = z_data[i]
                
                self.temp[self.MeasurementValidNum] = i
                
                state_idx = self.MeasurementMap[i] - 1
                if 0 <= state_idx < self.xhat_size and i < len(self.H.data):
                    self.H.data[i][state_idx] = self.MeasurementDegree[i]
                
                self.MeasurementValidNum += 1
                valid_indices.append(i)
        
        # 重建R矩阵（对角阵）
        for i, idx in enumerate(valid_indices):
            if i < self.z_size and i < len(self.R.data):
                self.R.data[i][i] = self.MatR_DiagonalElements[idx]
        
        # 根据有效测量调整矩阵
        if self.MeasurementValidNum > 0:
            # 新H矩阵（有效测量数×状态维数）
            new_H = Matrix(self.MeasurementValidNum, self.xhat_size)
            for i in range(self.MeasurementValidNum):
                row_idx = valid_indices[i]
                for j in range(self.xhat_size):
                    if j < len(new_H.data[i]):
                        new_H.data[i][j] = self.H.data[row_idx][j]
            self.H = new_H
            
            # 新R矩阵（有效测量数×有效测量数）
            new_R = Matrix(self.MeasurementValidNum, self.MeasurementValidNum)
            for i in range(self.MeasurementValidNum):
                if i < len(new_R.data):
                    new_R.data[i][i] = self.MatR_DiagonalElements[valid_indices[i]]
            self.R = new_R
            
            # 新z向量（有效测量数×1）
            new_z = Matrix(self.MeasurementValidNum, 1)
            for i in range(self.MeasurementValidNum):
                if i < len(new_z.data):
                    new_z.data[i][0] = z_data[valid_indices[i]]
            self.z = new_z
            
            # 调整K矩阵（状态维数×有效测量数）
            self.K = Matrix(self.xhat_size, self.MeasurementValidNum)
    
    def measure(self):
        """测量处理方法"""
        if self.UseAutoAdjustment:
            self.H_K_R_Adjustment()
        else:
            # 确保z矩阵尺寸正确
            if not hasattr(self.z, 'data') or len(self.z.data) != self.z_size:
                self.z = Matrix(self.z_size, 1)
                
            # 复制测量值
            for i in range(self.z_size):
                if i < len(self.z.data):
                    self.z.data[i][0] = self.MeasuredVector[i]
            
            self.MeasuredVector = [0.0] * self.z_size
            
        # 复制控制值
        if self.u_size > 0:
            # 确保u矩阵尺寸正确
            if not hasattr(self.u, 'data') or len(self.u.data) != self.u_size:
                self.u = Matrix(self.u_size, 1)
                
            for i in range(self.u_size):
                if i < len(self.u.data):
                    self.u.data[i][0] = self.ControlVector[i]
    
    def xhatMinusUpdate(self):
        """
        卡尔曼滤波方程1：状态预测
        xhat'(k) = F * xhat(k-1) + B * u(k-1)
        """
        if not self.SkipEq1:
            if self.u_size > 0 and self.B:
                # 如果有控制输入：temp_vector = F * xhat
                temp_vector = matrix_multiply(self.F, self.xhat)
                # temp_vector1 = B * u
                temp_vector1 = matrix_multiply(self.B, self.u)
                # 状态预测 = F * xhat + B * u
                self.xhatminus = matrix_add(temp_vector, temp_vector1)
            else:
                # 没有控制输入：状态预测 = F * xhat
                self.xhatminus = matrix_multiply(self.F, self.xhat)
    
    def PminusUpdate(self):
        """
        卡尔曼滤波方程2：协方差预测
        P'(k) = F * P(k-1) * F^T + Q
        """
        if not self.SkipEq2:
            # 计算F的转置
            self.FT = matrix_transpose(self.F)
            # temp_matrix = F * P
            temp1 = matrix_multiply(self.F, self.P)
            # temp_matrix = F * P * F^T
            self.temp_matrix = matrix_multiply(temp1, self.FT)
            # 协方差预测 = F * P * F^T + Q
            self.Pminus = matrix_add(self.temp_matrix, self.Q)
    
    def set_K(self):
        """
        卡尔曼滤波方程3：卡尔曼增益计算
        K(k) = P'(k) * H^T * inv(H * P'(k) * H^T + R)
        """
        if not self.SkipEq3:
            # 如果没有有效测量且使用自动调整，跳过计算
            if self.MeasurementValidNum == 0 and self.UseAutoAdjustment:
                return
            
            # 计算H的转置
            self.HT = matrix_transpose(self.H)
            # temp1 = H * Pminus
            temp1 = matrix_multiply(self.H, self.Pminus)
            # temp_matrix1 = H * Pminus * H^T
            self.temp_matrix1 = matrix_multiply(temp1, self.HT)
            # S = H * Pminus * H^T + R
            self.S = matrix_add(self.temp_matrix1, self.R)
            
            # 尝试计算S的逆矩阵（如果S维度有效）
            if self.S.rows > 0 and self.S.cols > 0:
                try:
                    S_inv = matrix_inverse(self.S)
                except Exception as e:
                    # 矩阵求逆失败处理（通常因为奇异矩阵）
                    print("矩阵求逆错误:", e)
                    return
                
                # temp_matrix = Pminus * H^T
                self.temp_matrix = matrix_multiply(self.Pminus, self.HT)
                # K = Pminus * H^T * S_inv
                self.K = matrix_multiply(self.temp_matrix, S_inv)
    
    def xhatUpdate(self):
        """
        卡尔曼滤波方程4：状态更新
        xhat(k) = xhat'(k) + K * (z(k) - H * xhat'(k))
        """
        if not self.SkipEq4:
            # 如果没有有效测量且使用自动调整，直接使用预测值
            if self.MeasurementValidNum == 0 and self.UseAutoAdjustment:
                self.xhat = self.xhatminus  # 没有测量，使用预测值
                return
            
            # temp_vector = H * xhatminus (预测的测量值)
            temp_vector = matrix_multiply(self.H, self.xhatminus)
            # temp_vector1 = z - H*xhatminus (测量残差)
            self.temp_vector1 = matrix_subtract(self.z, temp_vector)
            # temp_vector = K * (测量残差) (卡尔曼增益乘以残差)
            self.temp_vector = matrix_multiply(self.K, self.temp_vector1)
            # 状态更新 = 预测状态 + 校正项
            self.xhat = matrix_add(self.xhatminus, self.temp_vector)
    
    def P_Update(self):
        """
        卡尔曼滤波方程5：协方差更新
        P(k) = (I - K * H) * P'(k) 
        ≈ P'(k) - K * H * P'(k) (简化形式，数值更稳定)
        """
        if not self.SkipEq5:
            # 如果没有有效测量且使用自动调整，直接使用预测协方差
            if self.MeasurementValidNum == 0 and self.UseAutoAdjustment:
                self.P = self.Pminus
                return
            
            # temp_matrix = K * H
            temp_matrix = matrix_multiply(self.K, self.H)
            # temp_matrix1 = K * H * Pminus
            self.temp_matrix1 = matrix_multiply(temp_matrix, self.Pminus)
            # P = Pminus - temp_matrix1
            self.P = matrix_subtract(self.Pminus, self.temp_matrix1)
            
            # 强制设置最小方差，防止过度收敛
            for i in range(self.xhat_size):
                if self.P.data[i][i] < self.StateMinVariance[i]:
                    self.P.data[i][i] = self.StateMinVariance[i]
    
    def update(self):
        """
        执行完整的卡尔曼滤波更新步骤
        返回滤波后的状态向量（一维列表）
        """
        # 1. 测量处理阶段
        self.measure()
        
        # 用户定义的预测量处理函数
        if self.User_Func0_f:
            self.User_Func0_f(self)
        
        # 2. 预测阶段 - 状态预测
        self.xhatMinusUpdate()
        if self.User_Func1_f:
            self.User_Func1_f(self)
        
        # 3. 预测阶段 - 协方差预测
        self.PminusUpdate()
        if self.User_Func2_f:
            self.User_Func2_f(self)
        
        # 4. 更新阶段（仅当有测量或不使用自动调整时）
        if not self.UseAutoAdjustment or self.MeasurementValidNum > 0:
            # 计算卡尔曼增益
            self.set_K()
            if self.User_Func3_f:
                self.User_Func3_f(self)
            
            # 状态更新
            self.xhatUpdate()
            if self.User_Func4_f:
                self.User_Func4_f(self)
            
            # 协方差更新
            self.P_Update()
        else:
            # 没有有效测量时使用预测值
            self.xhat = self.xhatminus
            self.P = self.Pminus
        
        # 用户定义的后处理函数
        if self.User_Func5_f:
            self.User_Func5_f(self)
        
        # 再次强制最小方差（保险）
        for i in range(self.xhat_size):
            if self.P.data[i][i] < self.StateMinVariance[i]:
                self.P.data[i][i] = self.StateMinVariance[i]
        
        # 将状态矩阵复制到易于访问的列表
        for i in range(self.xhat_size):
            self.FilteredValue[i] = self.xhat.data[i][0]
        
        # 最终用户函数（可访问最终结果）
        if self.User_Func6_f:
            self.User_Func6_f(self)
        
        # 返回滤波后的状态
        return self.FilteredValue
    

class PitchKalmanFilter:
    """Pitch角卡尔曼滤波器封装类"""
    def __init__(self, dt=0.01):
        """
        初始化Pitch角滤波器
        :param dt: 采样时间间隔(秒)
        """
        # 状态维度: Pitch角 (1维)
        # 控制输入维度: 俯仰角速度 (1维)
        # 测量维度: Pitch角测量值 (1维)
        self.kf = KalmanFilter(xhat_size=1, u_size=1, z_size=1)
        
        # 时间步长
        self.dt = dt
        
        # 初始化滤波器参数
        self.kf.xhat = Matrix(1, 1, [[0.0]])      # 初始Pitch角估计
        
        # 初始状态协方差 - 增大初始不确定性
        self.kf.P = Matrix(1, 1, [[50.0]])       # 初始不确定性
        
        # 状态转移矩阵 (Pitch角 = Pitch角 + 俯仰角速度 × dt)
        self.kf.F = Matrix(1, 1, [[1.0]])        # F = [1]
        
        # 控制矩阵 (控制输入 = 俯仰角速度)
        self.kf.B = Matrix(1, 1, [[dt]])         # B = [dt]
        
        # 过程噪声协方差 (角度变化噪声)
        # 增加过程噪声以适应更大的动态变化
        pitch_rate_noise_std = 4.0                # 增大俯仰角速度噪声标准差 (度/秒)
        self.kf.Q = Matrix(1, 1, [[(pitch_rate_noise_std * dt) ** 2]])
        
        # 测量噪声 (传感器噪声方差)
        # 减小测量噪声以增加对测量的信任
        pitch_noise_std = 2.0                     # 减小Pitch角测量噪声标准差 (度)
        self.kf.MatR_DiagonalElements = [pitch_noise_std ** 2]
        
        # 最小方差限制 - 防止过度收敛
        self.kf.StateMinVariance = [0.1]         # 0.1度² (稍微增大)
        
        # 启用自动调整
        self.kf.UseAutoAdjustment = True
        
        # 测量映射关系 (直接测量Pitch角)
        self.kf.MeasurementMap = [1]
        self.kf.MeasurementDegree = [1.0]
        
        # 上一次滤波后的Pitch角
        self.last_filtered_pitch = 0.0
        
        # 角度约束范围
        self.min_angle = -90.0
        self.max_angle = 90.0
    def update(self, pitch_rate, pitch_measurement):
        """
        执行卡尔曼滤波更新
        :param pitch_rate: 俯仰角速度测量值 (度/秒)
        :param pitch_measurement: Pitch角测量值 (度)
        :return: 滤波后的Pitch角 (度)
        """
        # 设置控制输入（俯仰角速度）
        self.kf.ControlVector = [pitch_rate]
        
        # 设置测量输入（Pitch角测量值）
        self.kf.MeasuredVector = [pitch_measurement]
        
        # 执行卡尔曼滤波更新
        filtered_state = self.kf.update()
        
        # 获取滤波后的Pitch角
        filtered_pitch = filtered_state[0]
        
        # 应用角度约束（确保在合理范围内）
        filtered_pitch = self._constrain_angle(filtered_pitch)
        
        # 保存上一次结果
        self.last_filtered_pitch = filtered_pitch
        
        return filtered_pitch
    
    def _constrain_angle(self, angle):
        """将角度限制在合理范围内"""
        if angle < self.min_angle:
            return self.min_angle
        elif angle > self.max_angle:
            return self.max_angle
        return angle
    
    def get_last_filtered_pitch(self):
        """获取上一次滤波后的Pitch角"""
        return self.last_filtered_pitch
    
    def reset(self, initial_pitch=0.0):
        """重置滤波器状态"""
        self.kf.xhat = Matrix(1, 1, [[initial_pitch]])
        self.kf.P = Matrix(1, 1, [[50.0]])  # 重置协方差
        self.last_filtered_pitch = initial_pitch
    
    def set_noise_parameters(self, process_noise_std=4.0, measurement_noise_std=2.0):
        """
        动态设置噪声参数
        :param process_noise_std: 过程噪声标准差 (度/秒)
        :param measurement_noise_std: 测量噪声标准差 (度)
        """
        # 更新过程噪声
        self.kf.Q = Matrix(1, 1, [[(process_noise_std * self.dt) ** 2]])
        
        # 更新测量噪声
        self.kf.MatR_DiagonalElements = [measurement_noise_std ** 2]
    
    def set_angle_constraints(self, min_angle=-90.0, max_angle=90.0):
        """
        设置角度约束范围
        :param min_angle: 最小允许角度 (度)
        :param max_angle: 最大允许角度 (度)
        """
        self.min_angle = min_angle
        self.max_angle = max_angle

#调用示例
#pitch_filter = PitchKalmanFilter(dt=0.01)  
#filtered_pitch = pitch_filter.update(pitch_rate, pitch_meas)