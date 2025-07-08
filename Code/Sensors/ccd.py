# 导入TSL1401线性CCD传感器模块
from seekfree import TSL1401

class CCDTracker:
    """
    TSL1401 CCD循迹模块封装类
    支持多通道CCD数据采集和偏差角计算
    
    功能特点：
    1. 支持多通道同时采集
    2. 自适应阈值计算
    3. 带边缘检测的循迹偏移计算
    4. 偏差角计算
    5. 状态打印和模块信息查询
    """
    
    def __init__(self, capture_div=10, channels=(0,), resolution=TSL1401.RES_12BIT):
        """
        初始化CCD循迹模块
        
        参数:
        capture_div -- 采集分频数，控制采样频率（建议与硬件定时器配合使用）
        channels -- 要使用的通道元组，如(0,)或(0,1)，支持多通道同时采集
        resolution -- 采样精度，可选TSL1401.RES_8BIT或RES_12BIT
        """
        # 创建TSL1401对象
        self.ccd = TSL1401(capture_div)
        # 设置采样精度
        self.ccd.set_resolution(resolution)
        # 存储要使用的通道
        self.channels = channels
        # 初始化每个通道的数据（获取初始值）
        self.data = [self.ccd.get(ch) for ch in channels]
        # 获取CCD的像素宽度（假设所有通道宽度相同）
        self.width = len(self.data[0])
        # 初始化每个通道的偏移量记录（用于急弯处理）
        self.last_offset = [0 for _ in channels]
        # 初始化每个通道的中心位置（用于边缘检测起点）
        self.last_center = [self.width // 2 for _ in channels]
        print("CCDTracker 初始化完成，通道:", channels)

    def update(self):
        """
        更新所有通道的数据
        
        说明:
        在需要获取最新数据时调用此方法
        遍历所有通道并获取最新CCD数据
        """
        for i, ch in enumerate(self.channels):
            self.data[i] = self.ccd.get(ch)
            
    def get_raw_data(self, idx=0):
        """
        获取指定通道的原始数据列表
        
        参数:
        idx -- 通道索引（默认为0）
        
        返回值:
        该通道的原始数据列表
        """
        return self.data[idx]
    
    def adaptive_threshold(self, data, trim_num=5):
        """
        计算自适应阈值（去极值平均法）
        
        参数:
        data -- 原始CCD数据列表
        trim_num -- 要移除的极值点数（默认为5）
        
        返回值:
        计算出的自适应阈值
        
        说明:
        1. 对数据进行排序
        2. 移除trim_num个最大值和最小值
        3. 计算剩余数据的平均值作为阈值
        这种算法可以有效抑制数据中的毛刺和异常点
        """
        # 数据长度不足以修剪时，返回简单平均值
        if len(data) < 2 * trim_num:
            return (max(data) + min(data)) * 0.5
            
        # 对数据进行排序
        sorted_data = sorted(data)
        # 移除两端的极值点
        trimmed_data = sorted_data[trim_num:-trim_num]
        # 计算平均值作为阈值
        return sum(trimmed_data) / len(trimmed_data)
    
    def find_line_segment(self, binary_data, start_pos):
        """
        从指定位置向两侧寻找线段的边缘
        
        参数:
        binary_data -- 二值化后的数据（0或1）
        start_pos -- 开始搜索的位置
        
        返回值:
        (left_edge, right_edge) -- 线段的左右边界
        
        说明:
        1. 向左寻找下降沿（从1到0的变化）作为线段左边界
        2. 向右寻找上升沿（从0到1的变化）作为线段右边界
        3. 这种边缘检测方法可以提高检测的准确性
        """
        width = len(binary_data)
        # 初始化左右边界为起始位置
        left_edge = start_pos
        right_edge = start_pos
        
        # 向左寻找下降沿（线段左边缘）
        i = start_pos
        while i > 0:
            # 检测下降沿（从1到0的变化）
            if binary_data[i] == 1 and binary_data[i-1] == 0:
                left_edge = i
                break
            i -= 1
        
        # 向右寻找上升沿（线段右边缘）
        i = start_pos
        while i < width - 1:
            # 检测上升沿（从0到1的变化）
            if binary_data[i] == 1 and binary_data[i+1] == 0:
                right_edge = i + 1  # 边界位于变化后的第一个1的位置
                break
            i += 1
            
        return (left_edge, right_edge)

    def get_line_offset(self, idx=0, threshold=None):
        """
        返回指定通道的循迹偏移量（中心为0，左负右正）
        
        参数:
        idx -- 通道索引（默认为0）
        threshold -- 二值化阈值（None表示使用自适应阈值）
        
        返回值:
        当前偏移量（单位：像素），中心为0，左侧为负，右侧为正
        
        处理步骤:
        1. 获取当前通道的原始数据
        2. 计算自适应阈值（如果未提供）
        3. 进行智能二值化（包括平滑过渡处理）
        4. 从上次的中心位置开始寻找线段边缘
        5. 校验线段有效性（防止毛刺干扰）
        6. 计算线段中心点相对于图像中心的偏移量
        7. 急弯冲出处理（使用上次偏移量）
        """
        # 获取当前通道的数据
        data = self.get_raw_data(idx)
        width = len(data)
        
        # 1. 计算自适应阈值（如果未提供）
        if threshold is None:
            threshold = self.adaptive_threshold(data)
        
        # 2. 二值化处理（带平滑过渡检测）
        binary = [0] * width  # 初始化为全0
        for i in range(1, width-1):  # 跳过边界（无法检测两侧）
            # 高于阈值 -> 1
            if data[i] > threshold:
                binary[i] = 1
            # 平滑过渡区域处理（防止由于镜头畸变导致的误判）
            elif (data[i-1] > threshold and 
                  data[i+1] > threshold and 
                  abs(data[i] - threshold) < threshold/3):
                binary[i] = 1
        
        # 3. 从上次的中心位置开始寻找线段（使用边界保护）
        # 确保起始位置在有效范围内（避免越界错误）
        start_pos = min(max(self.last_center[idx], 10), width - 10)
        # 寻找线段边缘
        left, right = self.find_line_segment(binary, start_pos)
        
        # 4. 校验线段有效性（避免毛刺干扰）
        segment_width = right - left + 1
        
        # 有效线段条件：宽度在合理范围内（排除过窄或过宽的情况）
        if 5 <= segment_width <= width * 0.8:
            # 计算线段中心点
            center = (left + right) // 2
            # 计算相对于图像中心的偏移量
            offset = center - (width // 2)
            # 更新状态（用于下一次搜索）
            self.last_center[idx] = center
            self.last_offset[idx] = offset
            return offset
        
        # 5. 无效线段处理：急弯冲出弯道处理
        # 常见于急转弯或丢失赛道，此时使用上次的偏移值保持控制连续性
        print(f"警告[通道{idx}]: 未检测到有效线段（宽度={segment_width}），使用上次偏移值")
        return self.last_offset[idx]

    def get_line_angle(self, idx=0, threshold=None, pixel_length=128, fov_deg=120):
        """
        返回指定通道的夹角（度，左负右正）
        
        参数:
        idx -- 通道索引（默认为0）
        threshold -- 二值化阈值（None表示使用自适应阈值）
        pixel_length -- CCD像素长度（默认为128）
        fov_deg -- 视野角度（默认为60度）
        
        返回值:
        当前偏差角度（单位：度），左侧为负，右侧为正
        
        说明:
        基于像素偏移量计算角度偏差，适用于转向控制
        """
        # 获取像素偏移量
        offset = self.get_line_offset(idx, threshold)
        # 计算每个像素对应的角度
        angle_per_pixel = fov_deg / pixel_length
        # 计算当前角度偏差
        angle = offset * angle_per_pixel
        return angle

    def print_status(self, idx=0, threshold=None):
        """
        打印指定通道的循迹偏差和夹角
        
        参数:
        idx -- 通道索引（默认为0）
        threshold -- 二值化阈值（None表示使用自适应阈值）
        
        说明:
        实时显示当前循迹状态，便于调试
        """
        # 获取当前像素偏移
        offset = self.get_line_offset(idx, threshold)
        # 获取当前角度偏差
        angle = self.get_line_angle(idx, threshold)
        # 格式化输出状态
        print(f"[CCD{self.channels[idx]}] 偏移量: {offset:.2f} 像素, 夹角: {angle:.2f}°")

    def help(self):
        """
        显示TSL1401帮助信息
        
        说明:
        调用底层驱动库的帮助方法
        """
        TSL1401.help()
    
    def info(self):
        """
        显示当前CCD对象信息
        
        说明:
        调用底层驱动库的信息输出方法
        """
        self.ccd.info()
