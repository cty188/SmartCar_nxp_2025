from smartcar import ticker
from smartcar import encoder
from seekfree import KEY_HANDLER
from machine import *
import gc
import time
from display import *
from seekfree import TSL1401
from seekfree import MOTOR_CONTROLLER
from seekfree import WIRELESS_UART
from display import *
from seekfree import IMU963RA# 从 seekfree 库包含 IMU963RA
import gc
import time
import math
import ustruct
# 新增OpenART初始化
from seekfree import OpenART
art = OpenART()
imu = IMU963RA()
ccd = TSL1401(10)
ticker_flag = False
err_1 = 0
err_sum_1 = 0
err_x_1 = 0
err_last_1 = 0
err_2 = 0
err_sum_2 = 0
err_x_2 = 0
err_last_2 = 0
err_3 = 0
err_sum_3 = 0
err_x_3 = 0
err_last_3 = 0
errt= 0
errt_sum = 0
errt_x = 0
errt_last = 0
min_value = 0
max_value = 0
loud=100
ticker_count0=0
ticker_count = 0
ticker_flag2 = 0
Filter_data=[0,0,0]
PI=3.14159265358
last_yaw=0
end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
end_state = end_switch.value()
# 定义片选引脚
fenmingqi = Pin('C9' , Pin.OUT, pull = Pin.PULL_UP_47K, value = 0)
         
cs = Pin('C5' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 拉高拉低一次 CS 片选确保屏幕通信时序正常
cs.high()
cs.low()
key     = KEY_HANDLER(5)
# 定义控制引脚
rst = Pin('B9' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
dc  = Pin('B8' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
blk = Pin('C4' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 新建 LCD 驱动实例 这里的索引范围与 SPI 示例一致 当前仅支持 IPS200
drv = LCD_Drv(SPI_INDEX=1, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
# 新建 LCD 实例
lcd = LCD(drv)
# color 接口设置屏幕显示颜色 [前景色,背景色]
lcd.color(0x0000, 0xFFFF)
# mode 接口设置屏幕显示模式 [0:竖屏,1:横屏,2:竖屏180旋转,3:横屏180旋转]
lcd.mode(2)
# 清屏
lcd.clear(0xFFFF)
motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C24_DIR_C26, 13000, duty = 0, invert = True)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C25_DIR_C27, 13000, duty = 0, invert = True)
motor_dir = 1
motor_duty = 0
motor_duty_max = 300
ticker_count3 = 0
turn=0
led     = Pin('C4' , Pin.OUT, pull = Pin.PULL_UP_47K, value = True)
beep    = Pin('D24', Pin.OUT, pull = Pin.PULL_UP_47K, value = False)
switch1 = Pin('D8' , Pin.IN , pull = Pin.PULL_UP_47K, value = True)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K, value = True)
encoder_l = encoder("D0", "D1", True)
encoder_r = encoder("D2", "D3")
############################串口############################
wireless = WIRELESS_UART(115200)
# uart3 = UART(2)
# uart3.init(115200)
# uart3.write("Test.\r\n")
# buf_len = 0
# data_analysis 数据解析接口 适配逐飞助手的无线调参功能
data_flag = wireless.data_analysis()
data_wave = [0,0,0,0,0,0,0,0]
wireless.get_data(0)
# for i in range(0,8):
#     # get_data 获取调参通道数据 只有一个参数范围 [0-7]	wireless.get_data(i)--此为获取调参通道的数据函数
#     data_wave[i] = imu_data[i]

###########################################################
state1  = switch1.value()
state2  = switch2.value()
ticker_flag = False
ticker_count = 0

# 调用 TSL1401 模块获取 CCD 实例
# 参数是采集周期 调用多少次 capture 更新一次数据
# 默认参数为 1 调整这个参数相当于调整曝光时间倍数
ccd = TSL1401(10)
ticker_flag = False
ticker_count = 0
runtime_count = 0
i=0
ccd1_zhongzhi = 64    # 在函数外部定义并初始化全局变量
ccd2_zhongzhi = 64
left1 = 0
right1 = 0
left2 = 0
right2 = 0
flag_stute1 = 0
flag_stute2 = 0
flag_shizi1 = 0
flag_shizi2 = 0
shreshlod1 = 0
shreshlod2 = 0
##############################################ccd_ips
lline=0
rline=0
lline2=0
rline2=0
zThreshold=15
zThreshold2 =15
zhong=64
zhong2=64
rline_real=0
lline_real=0
rline2_last=0
lline2_last=0
cha=0
Kpc=0
banma_flag=0
stop=1
ting=0
ysbzw=1
lline_track=0
rline_track=0
ccd_sum=10
track_stop=0
huang_tag=0
huang_l_flag=0
huang_r_flag=0
huang_l_zt=0
huang_r_zt=0
anx=0
bnx=0
bizhan_l=0
bizhan_r=0
txzb=0
bizhan_flag=0
cha2=0
lline_last=0
rline_last=0
huang_l_zt2_min=0
huang_r_zt2_max=0
zuocha=0
zuocha2=0
flag_shizi1 = 0
flag_shizi2 = 0
txzb = 1
banma_flag = 0
zebra = 0
road2=0
jifen=0
art1=0
art2=0
art3=0
puodao_flag=0
bizhan_r_flag=0
bizhan_l_flag=0
##############################################UART
uart2 = UART(2)
uart2.init(115200)

distance=0
id_number=0   # 初始化变量m为整数







##############################################control
#////////////角速度//////////////////////
angle_kp =-1870.0#1870
angle_ki =-2.26#20.26
angle_kd =-120.0#400
#////////////角度//////////////////////
roll_angle_Kp =0.0682#0.1002
roll_angle_Ki =0.000001#0.00081
roll_angle_Kd =0.0811#0.2011
#////////////速度//////////////////////

speed_Kp = 0.06#0.07
speed_Ki = 0.002#0.0012
speed_Kd =0.0
angle_1 = 0
speed_1 = 0
counts =0
motor1=0
motor2=0
angle_1=0
speed_1=0
med_roll_angle=-35
med_speed1 = 40

#////////////////转向//////////////////////
a=0.0032#0.026
Kpc=1.781#7.721
turn_ki=0.008#0.014
turn_kd=15.64#0.62
turn_kd2 =-110.12
####################################################
#################################################encoder
KAL_P=0.02 #估算协方差
KAL_G=0.0 #卡尔曼增益
KAL_Q=0.70 #过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
KAL_R=200 #测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
KAL_Output=0.0 #卡尔曼滤波器输出 

KAL_P2=0.02 #估算协方差
KAL_G2=0.0 #卡尔曼增益
KAL_Q2=0.70 #过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
KAL_R2=200 #测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
KAL_Output2=0.0 #卡尔曼滤波器输出 

######################################################
# 定义赛道类型
class TrackType:
    STRAIGHT = 0
    CURVE = 1
    CROSS = 2
    RAMP = 3      # 黄色立方体
    LEFT_LOOP = 4 # 绿色立方体
    S_CURVE = 5   # 红色立方体

current_track = TrackType.STRAIGHT
last_detection_time = 0
cube_detection_interval = 3000  # 立方体检测间隔(ms)

def process_art_detection():
    global current_track, last_detection_time
    
    # 获取OpenART识别结果
    objects = art.get_detection()
    now = time.ticks_ms()
    
    # 优先处理立方体（特殊元素）
    for obj in objects:
        if obj.type == 'cube' and time.ticks_diff(now, last_detection_time) > cube_detection_interval:
            if obj.color == 'yellow':
                current_track = TrackType.RAMP
                last_detection_time = now
            elif obj.color == 'green':
                current_track = TrackType.LEFT_LOOP
                last_detection_time = now
            elif obj.color == 'red':
                current_track = TrackType.S_CURVE
                last_detection_time = now
    
    # 如果没有特殊元素，检测赛道类型
    if current_track not in [TrackType.RAMP, TrackType.LEFT_LOOP, TrackType.S_CURVE]:
        lines = art.get_line()
        if lines:
            if len(lines) >= 3:  # 检测到十字
                current_track = TrackType.CROSS
            elif abs(lines[0].angle) > 15:  # 弯道判断
                current_track = TrackType.CURVE
            else:
                current_track = TrackType.STRAIGHT

def handle_special_track():
    global med_speed1, turn, huang_l_flag, huang_l_zt, road, circle
    
    if current_track == TrackType.STRAIGHT:
        # 直道保持原有逻辑
        pass
        
    elif current_track == TrackType.CURVE:
        # 弯道保持原有CCD巡线逻辑
        pass
        
    elif current_track == TrackType.CROSS:
        # 十字路口处理（保持原有十字处理逻辑）
        flag_shizi1 = 1
        fenmingqi.high()
        zhong2 = 64
        
    elif current_track == TrackType.RAMP:
        # 坡道处理（加速通过）
        med_speed1 = 30  # 提高速度
        # 可以加入IMU角度检测确保爬坡稳定性
        
    elif current_track == TrackType.LEFT_LOOP:
        # 左环岛处理（触发原有环岛状态机）
        if huang_l_zt == 0:
            huang_l_flag = 1
            huang_tag = 1
            huang_l_zt = 1
            
    elif current_track == TrackType.S_CURVE:
        # S弯处理（增强转向灵敏度）
        turn *= 1.5  # 增大转向量
        med_speed1 = 10  # 适当降速
######################################################
def limit(value,min_value,max_value):
    if value<min_value:
        value=min_value
    elif value>max_value:
        value=max_value
    else:
        value=value
    return value
def calculate_pid(err, err_sum, err_last, med, value, kp, ki, kd):
    """
    辅助函数，用于计算PID控制的基本部分，提取公共逻辑
    """
    err = med - value
    err_sum += err
    err_x = err - err_last
    pwm = kp * err + ki * err_sum + kd * err_x
    err_last=err
    return pwm


def pid_position_1(med, value, kp, ki, kd):
    global err_1, err_sum_1, err_x_1, err_last_1
    pwm_1= calculate_pid(err_1, err_sum_1, err_last_1, med, value, kp, ki, kd)
    err_last_1 = err_1
    return pwm_1


def pid_position_2(med, value, kp, ki, kd):
    global err_2, err_sum_2, err_x_2, err_last_2
    pwm_2= calculate_pid(err_2, err_sum_2, err_last_2, med, value, kp, ki, kd)
    err_last_2 = err_2
    return pwm_2


def pid_position_3(med, value, kp, ki, kd):
    global err_3, err_sum_3, err_x_3, err_last_3
    pwm_3= calculate_pid(err_3, err_sum_3, err_last_3, med, value, kp, ki, kd)
    err_last_3 = err_3
    return pwm_3
def pid_turn(med, value, kp, ki, kd):
    global errt, errt_sum, errt_x, errt_last
    pwmt= calculate_pid(errt, errt_sum, errt_last, med, value, kp, ki, kd)
    errt_last = errt
    return pwmt

class anjian:
    def __init__(self,KEY_1,KEY_2,KEY_3,KEY_4):
        self.One=KEY_1
        self.Two=KEY_2
        self.Three=KEY_3
        self.Four=KEY_4
KEY=anjian(0,0,0,0)
def Key():
    key_data = key.get()
    KEY.One=key_data[0]
    KEY.Two=key_data[1]
    KEY.Three=key_data[2]
    KEY.Four=key_data[3]
    if (KEY.One+KEY.Two+KEY.Three+KEY.Four)!= 0 :
        print(f"{KEY.One}")
        print(f"{KEY.Two}")
        print(f"{KEY.Three}")
        print(f"{KEY.Four}")

class bianmaqi:
    def __init__(self,KAL_templ_pluse,KAL_tempr_pluse):
        self.KAL_templ_pluse=KAL_templ_pluse
        self.KAL_tempr_pluse=KAL_tempr_pluse
Encoders=bianmaqi(0,0)
class Imu_element:
    def __init__(self,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,Pitch,Roll,Yaw,X,Y,Z,Total_Yaw):
        self.acc_x=acc_x
        self.acc_y=acc_y
        self.acc_z=acc_z
        self.gyro_x=gyro_x
        self.gyro_y=gyro_y
        self.gyro_z=gyro_z
        self.Pitch=Pitch
        self.Roll=Roll
        self.Yaw=Yaw
        self.X=X
        self.Y=Y
        self.Z=Z
        self.Total_Yaw=Total_Yaw
Imu=Imu_element(0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00)
#############################################################
class param():
    def __init__(self,param_Kp,param_Ki):
        self.param_Kp=param_Kp
        self.param_Ki=param_Ki
Param=param(15.5,0.006)
# 
# Kp = 15.5 #比例增益控制加速度计/磁强计的收敛速度
# Ki = 0.006 #积分增益控制陀螺偏差的收敛速度
# halfT = 0.0025 #采样周期的一半
# #传感器框架相对于辅助框架的四元数(初始化四元数的值)
# q0 = 1
# q1 = 0
# q2 = 0
# q3 = 0
# #由Ki缩放的积分误差项(初始化)
# exInt = 0
# eyInt = 0
# ezInt = 0
# Pitch=0.00
# Roll=0.00
# Yaw=0.00

class QInfo:
    def __init__(self):
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
Q_info = QInfo()
delta_T = 0.001  # 假设采样周期为5ms
# param_Kp = 15.5  # 比例系数
# param_Ki = 0  # 积分系数
I_ex, I_ey, I_ez = 0.0, 0.0, 0.0  # 积分误差
def invSqrt(x):
    return 1.0 #/ (math.sqrt(x))
#############################################################
max_gyro_x=0

############################陀螺仪############################

def Limit(value):
    if value>1:
        value=1
    elif value <-1:
        value=-1
    return value
def Imu963():
    #if (ticker_flag and ticker_count % 20 == 0):
    alpha=0.3
    global imu_data,max_gyro_x
    if abs(imu_data[3])<30 or abs(imu_data[3])>30000:
        imu_data[3]=0
    if abs(imu_data[4])<30 or abs(imu_data[4])>30000:
        imu_data[4]=0
    if abs(imu_data[5])<30 or abs(imu_data[5])>30000:
        imu_data[5]=0
    
    Imu.X=int(imu_data[3]/16.4)
    Imu.Y=int(imu_data[4]/16.4) #俯仰角
    Imu.Z=int(imu_data[5]/16.4)
    Imu.gyro_x=round((float(imu_data[3])-Filter_data[0]),3)* PI / 180 / 16.4
    Imu.gyro_y=round((float(imu_data[4])-Filter_data[1]),3)* PI / 180 / 16.4
    Imu.gyro_z=round((float(imu_data[5])-Filter_data[2]),3)* PI / 180 / 14.4
    Imu.acc_x=round(((float(imu_data[0])*alpha)/ 4096 +Imu.acc_x* (1 - alpha)),3)
    Imu.acc_y=round(((float(imu_data[1])*alpha)/ 4096 +Imu.acc_y* (1 - alpha)),3)
    Imu.acc_z=round(((float(imu_data[2])*alpha)/ 4096 +Imu.acc_z* (1 - alpha)),3)
#     Imu.Pitch,Imu.Roll,Imu.Yaw=Update_IMU(Imu.acc_x,Imu.acc_y,Imu.acc_z,Imu.gyro_x,Imu.gyro_y,Imu.gyro_z)
    IMU_AHRSupdate(Imu.gyro_x,Imu.gyro_y,Imu.gyro_z,Imu.acc_x,Imu.acc_y,Imu.acc_z)
    if abs(max_gyro_x)<abs(Imu.Pitch):
        max_gyro_x=Imu.Pitch
#    print(f"pitch:{Imu.Pitch}")

#    imu_data = imu.get() 		# 通过 get 接口读取数据		用ticker自动采集列表关联了就可以不用这个采集了

#    print("acc = {:>6d}, {:>6d}, {:>6d}.".format(imu_data[0], imu_data[1], imu_data[2]))
#    print("groy = {:>6d}, {:>6d}, {:>6d}.".format(imu_data[3], imu_data[4], imu_data[5]))	##imu_data[3]左右	imu_data[4]上下	imu_data[5]平移

def Imu963ra_Init():
    global Filter_data
    global imu_data
    Filter_data[0]=0
    Filter_data[1]=0
    Filter_data[2]=0

    for i in range(0,1000):
        imu_data = imu.get()
        Filter_data[0]+=imu_data[3]
        Filter_data[1]+=imu_data[4]
        Filter_data[2]+=imu_data[5]
        time.sleep_ms(1)
    Filter_data[0]=float(Filter_data[0]/1000)
    Filter_data[1]=float(Filter_data[1]/1000)
    Filter_data[2]=float(Filter_data[2]/1000)
#     print(f"Filter_data[0]:{Filter_data[0]}")
#     print(f"Filter_data[1]:{Filter_data[1]}")
#     print(f"Filter_data[2]:{Filter_data[2]}")


def Update_IMU(ax,ay,az,gx,gy,gz):
    global q0
    global q1
    global q2
    global q3
    global exInt
    global eyInt
    global ezInt
    # print(q0)
    
    #测量正常化
    norm = math.sqrt(ax*ax+ay*ay+az*az)
    print(ax*ax+ay*ay+az*az)
    #单元化
    ax = ax/norm
    ay = ay/norm
    az = az/norm
    
    #估计方向的重力
    vx = 2*(q1*q3 - q0*q2)
    vy = 2*(q0*q1 + q2*q3)
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3
    
    #错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
    ex = (ay*vz - az*vy)
    ey = (az*vx - ax*vz)
    ez = (ax*vy - ay*vx)
    
    #积分误差比例积分增益
    exInt += ex*Ki
    eyInt += ey*Ki
    ezInt += ez*Ki
    
    #调整后的陀螺仪测量
    gx += Kp*ex + exInt
    gy += Kp*ey + eyInt
    gz += Kp*ez + ezInt
    
    #整合四元数
    q0 += (-q1*gx - q2*gy - q3*gz)*halfT
    q1 += (q0*gx + q2*gz - q3*gy)*halfT
    q2 += (q0*gy - q1*gz + q3*gx)*halfT
    q3 += (q0*gz + q1*gy - q2*gx)*halfT
    
    #正常化四元数
    norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
    q0 /= norm
    q1 /= norm
    q2 /= norm
    q3 /= norm
    
    #获取欧拉角 pitch、roll、yaw
    pitch = round(math.asin(-2*q1*q3+2*q0*q2)*57.3,3)
    roll = round(math.atan2(2*q2*q3+2*q0*q1,-2*q1*q1-2*q2*q2+1)*57.3,3)
    yaw = round(math.atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*57.3,3)

    return pitch,roll,yaw

def IMU_AHRSupdate(gx, gy, gz, ax, ay, az):
    global I_ex, I_ey, I_ez, last_yaw
    halfT = 0.5 * delta_T
    value1=0
    # 当前的机体坐标系上的重力单位向量
    vx, vy, vz = 0.0, 0.0, 0.0
    ex, ey, ez = 0.0, 0.0, 0.0
    q0,q1,q2,q3= 0.0, 0.0, 0.0, 0.0
    # 四元数计算相关变量
#     q0 = Q_info.q0
#     q1 = Q_info.q1
#     q2 = Q_info.q2
#     q3 = Q_info.q3
    q0q0 = Q_info.q0 * Q_info.q0	
    q0q1 = Q_info.q0 * Q_info.q1	
    q0q2 = Q_info.q0 * Q_info.q2
    q1q1 = Q_info.q1 * Q_info.q1
    q1q3 = Q_info.q1 * Q_info.q3
    q2q2 = Q_info.q2 * Q_info.q2
    q2q3 = Q_info.q2 * Q_info.q3
    q3q3 = Q_info.q3 * Q_info.q3
    
    # 对加速度数据进行归一化
    norm = invSqrt(ax*ax + ay*ay + az*az)
    ax *= norm
    ay *= norm
    az *= norm
    
    # 计算当前重力单位向量
    vx = 2 * (q1q3 - q0q2)
    vy = 2 * (q0q1 + q2q3)
    vz = q0q0 - q1q1 - q2q2 + q3q3
    
    # 计算误差
    ex = ay * vz - az * vy
    ey = az * vx - ax * vz
    ez = ax * vy - ay * vx
    
    # 用误差进行PI修正
    I_ex += delta_T * ex  # 积分误差
    I_ey += delta_T * ey
    I_ez += delta_T * ez

    gx += Param.param_Kp * ex + Param.param_Ki * I_ex
    gy += Param.param_Kp * ey + Param.param_Ki * I_ey
    gz += Param.param_Kp * ez + Param.param_Ki * I_ez
    
    # 四元数微分方程
    q0 = Q_info.q0
    q1 = Q_info.q1
    q2 = Q_info.q2
    q3 = Q_info.q3
    
    Q_info.q0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT
    Q_info.q1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT
    Q_info.q2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT
    Q_info.q3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT

    # 归一化四元数
    norm = invSqrt(Q_info.q0**2 + Q_info.q1**2 + Q_info.q2**2 + Q_info.q3**2)
    #norm=invSqrt(q0q0 + q1q1 + q2q2 + q3q3)
    Q_info.q0 *= norm
    Q_info.q1 *= norm
    Q_info.q2 *= norm
    Q_info.q3 *= norm
    #print(f"{(-2 * Q_info.q1 * Q_info.q3 + 2 * Q_info.q0 * Q_info.q2)}")
    # 计算欧拉角
    value1=Limit(-2 * Q_info.q1 * Q_info.q3 + 2 * Q_info.q0 * Q_info.q2)
    Imu.Roll = round(math.asin(value1) * 180 / math.pi,3)  # pitch
    Imu.Pitch = round(math.atan2(2 * Q_info.q2 * Q_info.q3 + 2 * Q_info.q0 * Q_info.q1, -2 * Q_info.q1**2 - 2 * Q_info.q2**2 + 1) * 180 / math.pi,3 ) # roll
    Imu.Yaw = round(math.atan2(2 * Q_info.q1 * Q_info.q2 + 2 * Q_info.q0 * Q_info.q3, -2 * Q_info.q2**2 - 2 * Q_info.q3**2 + 1) * 180 / math.pi,3) # yaw
    # 计算偏航角的误差
    error_yaw = Imu.Yaw - last_yaw
    if error_yaw < -360:
        error_yaw += 360
    if error_yaw > 360:
        error_yaw -= 360
    Imu.Total_Yaw += error_yaw
    last_yaw = Imu.Yaw
   
    
    # 保证total_yaw在0到360度之间
    if Imu.Total_Yaw > 360:
        Imu.Total_Yaw -= 360
    if Imu.Total_Yaw < 0:
        Imu.Total_Yaw += 360
# 
# def pingheng():
#     global counts,motor1,motor2,pitch_deg,roll_deg
#     
#     
# #     motor1=pid_position_1(angle_1,gy,angle_kp,angle_ki,angle_kd)#-turn
# #     #     motor1=0-turn
#     motor1=limit(motor1,-3000,3000)
# #     motor2=pid_position_1(angle_1,gy,angle_kp,angle_ki,angle_kd)#+turn
# #     motor2=0+turn
#     motor2=limit(motor2,-3000,3000)
#     motor_l.duty(-motor1)
#     motor_r.duty(-motor2)
#     




# class DRV():
#     def __init__(self,Duty,P,I,D,Error,Last_Error,D_Error):
#         self.Duty=Duty
#         self.P=P
#         self.I=I
#         self.D=D
#         self.Error=Error
#         self.Last_Error=Last_Error
#         self.D_Error=D_Error
# Motor=DRV(0,25,0,5,0,0,0)
# Angle=DRV(0,15,0,0,0,0,0)
def Wireless():
    data_flag = wireless.data_analysis()			# 定期进行数据解析
    if (data_flag[0]):	# 判断哪个通道有数据更新
        Motor.P=wireless.get_data(0)
    if (data_flag[1]):  
        Motor.D=wireless.get_data(1)
    if (data_flag[2]):  
        Angle.P=wireless.get_data(2)
    if (data_flag[3]):  
        Angle.D=wireless.get_data(3)
#             data_wave[i] = imu_data[i]				#########
#     Param.param_Kp=wireless.get_data(0)		# 数据更新到缓冲         这个是获取调参助手数值的函数
#     Param.param_Ki=wireless.get_data(1)
    data_wave[0]=Angle.Duty
    data_wave[1]=Motor.Duty
    data_wave[2]=Imu.acc_z
    data_wave[3]=Imu.gyro_x
#     data_wave[4]=(float(imu_data[4])*PI/180/16.4)
#     data_wave[5]=(float(imu_data[5])*PI/180/16.4)
    data_wave[4]=Imu.gyro_y
    data_wave[5]=Imu.Roll
    data_wave[6]=Imu.Pitch
#     data_wave[7]=
    # send_oscilloscope 将最多八个通道虚拟示波器数据上传到逐飞助手
    wireless.send_oscilloscope(
        data_wave[0],data_wave[1],data_wave[2],data_wave[3],
        data_wave[4],data_wave[5],data_wave[6],data_wave[7])
    ticker_flag = False

def KalmanFilter(input):
    global KAL_P
    global KAL_G
    global KAL_Output
#    enc3_data = encoder_3.get()
#    enc4_data = encoder_4.get()
    KAL_P = KAL_P + KAL_Q			#估算协方差方程：当前 估算协方差 = 上次更新 协方差 + 过程噪声协方差
    KAL_G = KAL_P / (KAL_P + KAL_R)	#//卡尔曼增益方程：当前 卡尔曼增益 = 当前 估算协方差 / （当前 估算协方差 + 测量噪声协方差）
    #更新最优值方程：当前 最优值 = 当前 估算值 + 卡尔曼增益 * （当前 测量值 - 当前 估算值）
    KAL_Output = KAL_Output + KAL_G * (input - KAL_Output);	#当前 估算值 = 上次 最优值
    KAL_P = (1 - KAL_G) * KAL_P;							#更新 协方差 = （1 - 卡尔曼增益） * 当前 估算协方差
    return KAL_Output;

def KalmanFilter2(input):
    global KAL_P2
    global KAL_G2
    global KAL_Output2
#    enc3_data = encoder_3.get()
#    enc4_data = encoder_4.get()
    KAL_P2 = KAL_P2 + KAL_Q2			#估算协方差方程：当前 估算协方差 = 上次更新 协方差 + 过程噪声协方差
    KAL_G2 = KAL_P2 / (KAL_P2 + KAL_R2)	#//卡尔曼增益方程：当前 卡尔曼增益 = 当前 估算协方差 / （当前 估算协方差 + 测量噪声协方差）
    #更新最优值方程：当前 最优值 = 当前 估算值 + 卡尔曼增益 * （当前 测量值 - 当前 估算值）
    KAL_Output2 = KAL_Output2 + KAL_G2 * (input - KAL_Output2);	#当前 估算值 = 上次 最优值
    KAL_P2 = (1 - KAL_G2) * KAL_P2;							#更新 协方差 = （1 - 卡尔曼增益） * 当前 估算协方差
    return KAL_Output2;
def ips200_display():
    ccd_data1 = ccd.get(0)
    ccd_data2 = ccd.get(1)
    # 用 format 或者 "%..."%(...) 统一处理为字符串对象
    
    # 显示字符串的函数 [x,y,str,color]
    # x - 起始显示 X 坐标
    # y - 起始显示 Y 坐标
    # str - 字符串
    # color - 字符颜色 可以不填使用默认的前景色
    lcd.str16(0,  0, "Pitch={:f}.".format(round(Imu.Pitch,3)),0x001F)
    lcd.str16(0,  16, "Yaw={:f}.".format(Imu.Yaw), 0x001F)
    lcd.str16(0, 150, "med_speed{:>6d}, turn{:>6f}.".format(med_speed,turn), 0x001F)
    lcd.str16(0, 168, f"{road:.2f}".format(road), 0x001F)
#     lcd.str16(0,  16, "Motor.P={:1f}.".format(Motor.P), 0xF800)
#     lcd.str16(0,  16*2, "Motor.D={:1f}.".format(Motor.D), 0xF800)
#     lcd.str16(0,  16*3, "Angle.P={:1f}.".format(Angle.P), 0xF800)
#     lcd.str16(0,  16*4, "Angle.D={:1f}.".format(Angle.D), 0xF800)
    lcd.str16(0, 285,f"motor1: {motor1:.2f},motor2:{motor2:.2f}", 0x001F)
    lcd.str16(0, 300, f"Track: {current_track}", 0x001F)
    
#     lcd.str16(0, 202, f"Roll: {roll_deg:.2f}, Pitch: {pitch_deg:.2f}", 0xF800)
    lcd.str16(0, 223, "g = {:>6f}, {:>6f}, {:>6f}.".format(Imu.gyro_x,Imu.gyro_y, Imu.gyro_z), 0x001F)
    lcd.str12(0, 243, "lline2{:>6d}, rline2{:>6d}, zhong2{:>6d}.".format(lline2,rline2,zhong2), 0x001F)
    lcd.str12(0, 263, "lline{:>6d}, rline{:>6d}, zhong{:>6d}.".format(lline,rline,zhong), 0x001F)
    
    lcd.str16(0, 209, "enc ={:>6f}, {:>6f}\r\n".format(Encoders.KAL_templ_pluse,Encoders.KAL_tempr_pluse), 0x001F)
#     lcd.wave(0,  40, 128, 60, ccd_data1, max = 255)
#     lcd.wave(0, 80, 128,60, ccd_data2, max = 255)
#     lcd.line(zhong2,40,zhong2,120,color=0xF000,thick=1)
#     lcd.line(lline2,40,lline2,120,color=0x07E0,thick=1)
#     lcd.line(rline2,40,rline2,120,color=0x07E0,thick=1)
#     print(ccd_data1)



    
     
def control_turn(zhong2):
    errt=(zhong2-64)
    turn_kp=a*abs(errt)+Kpc
    turn=pid_turn(64,zhong2, turn_kp, turn_ki, turn_kd)+Imu.gyro_z*turn_kd2
#     print(zhong)
    return turn
def angle_speed1(med_gyro,cur_gyro):
    motor=pid_position_1(med_gyro,cur_gyro,angle_kp,angle_ki,angle_kd)#-turn
    
    motor=limit(motor,-4000,4000)
    return 	(motor)
def angle(med_roll_angle,cur_roll_angle):
    global angle_1
    
    angle_1=pid_position_2(med_roll_angle,cur_roll_angle,roll_angle_Kp,roll_angle_Ki,roll_angle_Kd)
    return (angle_1)
def speed(med_speed,cur_speed):
    global speed_1
    speed_1=pid_position_3(med_speed,cur_speed,speed_Kp,speed_Ki,speed_Kd)
    return (speed_1)
#     print(speed_1)
def parse_data():
    if uart2.any():
        line = uart2.readline()  # 读取一行数据（直到换行符）
        if line is None:
            return None, None
        try:
            line_str = line.decode('utf-8').strip()  # 转换为字符串并去除两端空白
        except UnicodeDecodeError:
            return None, None  # 解码失败时返回None
        parts = line_str.split(',')  # 按逗号分割
        if len(parts) >= 2:
            try:
                var1 = float(parts[0])  # 转换第一部分为浮点数
                var2 = int(parts[1])    # 转换第二部分为整数
                return var1, var2
            except ValueError:
                pass  # 转换失败时忽略
    return None, None  # 无数据或数据无效时返回None

road = 0
fix = 0
circle = 0
zebra2 = 0
banma_slow = 0
stop_flag = 0
def ccd_ips():
    global lline
    global rline
    global lline2
    global rline2
    global zThreshold
    global zThreshold2 
    global zhong
    global zhong2
    global rline_real
    global lline_real
    global rline_last
    global lline_last
    global ticker_flag
    global runtime_count
    global turn,fix,circle
    global        banma_flag,banma_slow
    global        stop
    global        ting,stop_flag
    global        ysbzw,zebra2
    global        lline_track
    global        rline_track
    global        ccd_sum,road,road2
    global   track_stop,zebra
    global huang_tag, huang_l_flag, huang_r_flag
    global huang_l_zt, huang_r_zt, anx, bnx
    global lline, rline, lline2, rline2, jifenjiaodu
    global huang_tag, huang_l_flag, huang_r_flag
    global lline2_last, rline2_last
    global lline_last, rline_last
    global        txzb,bizhan_flag,flag_shizi1,flag_shizi2
    global bizhan_l,bizhan_r, loud,cha2,huang_l_zt2_min,huang_r_zt2_max,zuocha,zuocha2,puodao_flag,jifen,art1,art2,art3,bizhan_l_flag,bizhan_r_flag
    
        # 通过 get 接口读取数据 参数 [0,1] 对应学习板上 CCD1/2 接口
    ccd_data1 = ccd.get(0)
    ccd_data2 = ccd.get(1)            
             
      
    for i in range(zhong,0,-1):
        if (ccd_data1[i+5]-ccd_data1[i])*128/(ccd_data1[i+5]+ccd_data1[i]+0.1)>zThreshold:
            
            lline=i
            break
        if(i==3):
            lline=0
            break
        
    for i in range(zhong,127,1):
        if (ccd_data1[i-5]-ccd_data1[i])*128/(ccd_data1[i-5]+ccd_data1[i]+0.1)>zThreshold:
            rline=i
            break
        if(i==124):
            rline=127
            break
    for i in range(64,0,-1):           
        if (ccd_data2[i+5]-ccd_data2[i])*128/(ccd_data2[i+5]+ccd_data2[i]+0.1)>zThreshold2:
            lline2=i
            break
            if(i==3):
                lline2=0
                break
    for i in range(64,127,1):
        if (ccd_data2[i-5]-ccd_data2[i])*128/(ccd_data2[i-5]+ccd_data2[i]+0.1)>zThreshold2:
            rline2=i
            break
            if(i==124):
                rline2=127
                break
    for i in range(lline,rline,1):
            ccd_sum=ccd_data1[i]
    # ====================== 环岛处理核心逻辑 ======================
######################################################################################left
    if (huang_tag == 0 and  flag_shizi1 ==0 and
        huang_l_zt ==0 and 
        lline <= 18 and huang_r_zt ==0 and
        78<=rline < 92 and abs(zuocha)>20):
        if (abs(rline - rline2 ) <= 8):
            huang_l_flag = 1
            huang_tag = 1  # 设置全局环岛标志
            huang_l_zt =1
        
        
    elif  (road<=-3900 and huang_l_zt ==1) :
        anx=15
       	huang_l_zt =2
       	Imu.Yaw=-0
       	road=0
       	
    elif ( abs(circle)>=165 and 	huang_l_zt ==2 ):
        huang_l_zt =3
    elif ( lline >= 35 and rline <= 105 and lline2 >= 30 and rline2 <= 105  and 	huang_l_zt==3):
        
        huang_tag = 0
        huang_l_zt =0
        huang_l_flag =0
        Imu.Yaw = 0
        circle = 0
        fenmingqi.low()
# ################################################################################################right
    if (huang_tag == 0 and  huang_l_zt ==0 and
        huang_r_zt ==0 and
        rline >= 108 and 57>=lline>= 28 and
         flag_shizi1 != 1 and zuocha2>20):
        if (abs(lline - lline2 ) <= 8):
            huang_r_flag = 1
            huang_tag = 1  # 设置全局环岛标志
            huang_r_zt =1

        
    elif  (road<=-3950 and huang_r_zt ==1) :
        bnx=123
       	huang_r_zt =2
       	Imu.Yaw=-0
       	road=0

       	
    elif ( abs(circle)>=165 and 	huang_r_zt ==2 ):
        huang_r_zt =3
    elif ( rline <= 115 and lline >= 20 and rline2 <= 95 and lline2 >= 20  and 	huang_r_zt==3):
        
        huang_tag = 0
        huang_r_zt =0
        huang_r_flag =0
        Imu.Yaw = 0
        circle = 0
        fenmingqi.low()
# ##################################################################################################
#     elif lline >= 35 and rline <= 105 and lline2 >= 30 and rline2 <= 105 and huang_l_zt==1:
#         huang_tag == 0
#         huang_l_zt =0
#         anx=0
#         bnx=0
#         road2=road
#         road = 0
#         fenmingqi.low()
    # 右环岛触发条件（后CCD右侧线快速右移）    
#     elif (huang_tag == 0 and 
#           rline2 < 60 and 
#           rline >= 100 and 
#           (rline_last - lline_last) < 60 and 
#           (rline2 - lline2) < 60):
#         huang_r_flag = 1
#         huang_tag = 1
        
#     # ---------------------- 左环岛处理 ----------------------
#############################################################################left
    if huang_l_flag == 1:
        # 状态机切换逻辑
        if huang_l_zt == 1:
            road += (encoder_l.get()+encoder_r.get())/2
#             zhong2 = 64
            anx=rline2-55
            lline2=anx
            
        elif huang_l_zt ==2:
            fenmingqi.high()
            circle += Imu.gyro_z
            if rline2>70:
                rline2 = 70
#             if(rline2>90):
#                 rline2=90
            lline2=anx
        elif huang_l_zt ==3:
            fenmingqi.high()
            anx=rline2-55
            lline2=anx
            fix =0
###############################################################################right
    if huang_r_flag == 1:
        # 状态机切换逻辑
        if huang_r_zt == 1:
            fenmingqi.high()
            road += (encoder_l.get()+encoder_r.get())/2
#             zhong2 = 64
            bnx=lline2+55
            rline2=bnx
        elif huang_r_zt ==2:
            fenmingqi.low()
            circle += Imu.gyro_z
            if lline2<40:
                lline2 = 40
#             if(lline2<30):
#                 lline2=30
            rline2=bnx
        elif huang_r_zt ==3:
            fenmingqi.high()
            bnx=lline2+55
            rline2=bnx
            fix = 0
#             road += (Encoders.KAL_templ_pluse+Encoders.KAL_tempr_pluse)/2
#         elif  huang_l_zt == 2:
#            
#             rline2=bnx
#             lline2=anx   
        
           # 记录进入时的左侧线位置
    if lline2 <= 5 and rline2 >= 115 and  flag_shizi1 ==0  :
        
    #             flag_stute1 = 0
        flag_shizi1 = 1
        stop_flag +=1
    if flag_shizi1 == 1:
        fenmingqi.high()
        zhong2 = 64
    if lline2 >= 5 and rline2 <= 115 :
        flag_shizi1 = 0
#         if lline2<=20:
#             lline2 = 20
#         if rline2>=100:
#             rline2 = 100
        fenmingqi.low()
        
    ########art###############        
    if huang_tag==0 and flag_shizi1 ==0 and id_number==1:
        art1=1
    if huang_tag==0 and flag_shizi1 ==0 and id_number==2:
        art2=1
    if huang_tag==0 and flag_shizi1 ==0 and id_number==2:
        art3=1
#     if huang_tag==0 and flag_shizi1 ==0 and id_number==3:
#         art3=1
    if art1==1:
        motor_l.duty(0)
        motor_r.duty(0)
        pit1.stop()
    
    if art2==1:
        bizhan_l_flag=1
    if jifen<=-250:
        bizhan_l_flag=2
    if  bizhan_l_flag==1:
         jifen += (encoder_l.get()+encoder_r.get())/2
         lline2=rline2-20
         
    elif bizhan_l_flag==2:
         jifen=0
         bizhan_l_flag = 0
         art2=0
         
         
         
         
    if art3==1:
        bizhan_r_flag=1
    if jifen<=-250:
        bizhan_r_flag=2
    if  bizhan_r_flag==1:
         jifen += (encoder_l.get()+encoder_r.get())/2
         rline2=lline2+20
         
    elif bizhan_r_flag==2:
         jifen=0
         bizhan_r_flag = 0
         art3=0            
        
        
    ########art###############        
        
        
    if flag_shizi1 !=1:
        zhong2=int((lline2+rline2)/2)
#     zhong2=int((lline2+rline2)/2)
    zhong=int((lline+rline)/2)
    zuocha=lline2-lline2_last
    zuocha2=rline2-rline2_last
    lline2_last=lline2
    rline2_last=rline2
    lline_last=lline
    rline_last=rline
    cha=lline-rline
    cha2=lline2-rline2           
#     print(zhong2,lline2)      
    turn=control_turn(zhong2)
      
    ########斑马线###############

        
#     for i in range(44, 84):
#         if (abs(ccd_data2[i] - ccd_data2[i + 3]) >= 33):
#             zebra += 1
#         if (abs(ccd_data1[i] - ccd_data1[i + 3]) >= 20):
#             zebra2 += 1
#             
#     if zebra >= 10:
#         banma_flag = 1
#     if zebra2 >= 8:
#         banma_slow = 1   
#     if zebra <10:
#         banma_flag = 0
#         zebra = 0
#         zebra2 = 0
#         banma_slow = 0
#     if banma_slow == 1 :
#         med_speed1 = 30
#         
#     if banma_flag == 1 :
#         if stop_flag> 3:
#             med_speed1 = 30
#             fenmingqi.high()
#             time.sleep_ms(300)
#             fenmingqi.low()
#             motor_l.duty(0)
#             motor_r.duty(0)
#     #             ticker_flag = False
#             pit1.stop()
#         if stop_flag<= 3:
#             banma_flag = 0
# #         
#     zuocha2=lline2-lline2_last
#     zuocha2=lline-lline_last
#     if(zhong>97):
#         zhong=97
#     if(zhong<30):
#         zhong=30
#     if(zhong2>97):
#         zhong2=97
#     if(zhong2<30):
#         zhong2=30
        # 通过 wave 接口显示数据波形 (x,y,width,high,data,data_max)
        # x - 起始显示 X 坐标
        # y - 起始显示 Y 坐标
        # width - 数据显示宽度 等同于数据个数
        # high - 数据显示高度
        # data - 数据对象 这里基本仅适配 TSL1401 的 get 接口返回的数据对象
        # max - 数据最大值 TSL1401 的数据范围默认 0-255 这个参数可以不填默认 255
#     lcd.line(zhong,40,zhong,120,color=0xF000,thick=1)
#     lcd.line(lline,40,lline,120,color=0x07E0,thick=1)
#     lcd.line(rline,40,rline,120,color=0x07E0,thick=1)
#     print(ccd_data1)
    
  
        
# def control_turn():
#     errt=zhong-64
#     turn_kp=a*errt*errt+Kpc
#     turn_out=pid_turn(64, zhong, turn_kp, turn_ki, turn_kd)
#    
#     print(turn_out)
#     return turn_out
# control_turn()


def time_pit_handler(time):
    global ticker_flag,ticker_count,speed_1,angle_1,motor1,motor2 # 需要注意的是这里得使用 global 修饰全局属性
    ticker_flag = True
    ticker_count = (ticker_count + 1) if (ticker_count < 10) else (1)
    if  ticker_count % 1 == 0:
           
        Imu963()
        motor1=angle_speed1(angle_1,Imu.gyro_x)-turn
        motor2=angle_speed1(angle_1,Imu.gyro_x)+turn
        motor1=limit(motor1,-4000,4000)
        motor2=limit(motor2,-4000,4000)
        motor_l.duty(-motor1)
        motor_r.duty(-motor2)
#             print(motor1)
    if  ticker_count % 5 == 0:
        angle_1=angle(med_roll_angle-speed_1,Imu.Pitch)
#             print(Imu.Pitch)
        #         angle(med_roll_angle-speed_1,Imu.Roll)
    if  ticker_count % 10 == 0:
        encl_data = encoder_l.get()
        encr_data = encoder_r.get()
        # speed_1=pid_position_3(-med_speed,(encl_data+encr_data)/2,speed_Kp,speed_Ki,speed_Kd)
        speed_1=speed(-med_speed,(Encoders.KAL_templ_pluse + Encoders.KAL_tempr_pluse) /2)
def time_pit2_handler(time):
    global ticker_flag2,ticker_count2 # 需要注意的是这里得使用 global 修饰全局属性
    ticker_flag2 = True  # 否则它会新建一个局部变量
    global KAL_templ_pluse
    global KAL_tempr_pluse
#     print(10)
    Encoders.KAL_templ_pluse=KalmanFilter(encoder_l.get())
    Encoders.KAL_tempr_pluse=KalmanFilter2(encoder_r.get()) 
#     Angle_Cycle()
    
    ccd_data1 = ccd.get(0)
    ccd_data2 = ccd.get(1)

pit1 = ticker(1)
pit2 = ticker(2)
pit3 = ticker(3)
# 通过 capture 接口更新数据 但在这个例程中被 ticker 模块接管了
# imu.capture()
# 关联采集接口 最少一个 最多八个 (imu, ccd, key...)
# 可关联 smartcar 的 ADC_Group_x 与 encoder_x
# 可关联 seekfree 的  IMU963RA, IMU963RA, KEY_HANDLER 和 TSL1401
pit1.capture_list(imu)

# pit1.capture_list(ccd1,ccd2)
pit2.capture_list(ccd)
pit3.capture_list(encoder_l, encoder_r)
# 关联 Python 回调函数
pit1.callback(time_pit_handler)
pit2.callback(time_pit2_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
Imu963ra_Init()
pit1.start(1)
pit2.start(6)
pit3.start(10)

###########################################初始化###########################################


###########################################################################################
# wireless.send_oscilloscope( data_wave[0],data_wave[1],data_wave[2],data_wave[3], data_wave[4],data_wave[5],data_wave[6],data_wave[7])
# data_wave[0]=rline
# data_wave[1]=lline
# data_wave[2]=rline2
# data_wave[3]=lline2
while True:
    distance ,id_number = parse_data()
    print(id_number)
    
#     data_wave[0]=rline
#     data_wave[1]=lline
#     data_wave[2]=rline2
#     data_wave[3]=lline2
#     data_wave[4]= flag_shizi1 
#     wireless.send_oscilloscope( data_wave[0],data_wave[1],data_wave[2],data_wave[3], data_wave[4],data_wave[5],data_wave[6],data_wave[7])
# OpenART检测处理
    process_art_detection()
    handle_special_track()
    
    if(abs(zhong - 64 )>3 and abs(zhong - 64 )<=10):
        med_speed = med_speed1 - 35
#     if(huang_l_zt != 0):
#         med_speed = med_speed1 - 38
#     if(huang_r_zt != 0):
#         med_speed = med_speed1 - 38
    if(abs(zhong - 64 )>10):
        med_speed = med_speed1 - 30
#     if(abs(zhong2 - 64 )>3 and abs(zhong2 - 64 )<=10):
#         med_speed = med_speed1 - 38
#     if(turn>400):
#         med_speed = med_speed1 - 35
#     if(abs(zhong2 - 64 )>10):
#         med_speed = med_speed1 - 40
    if(abs(zhong - 64 )<=3):
        if(abs(zhong2 - 64 )<=3):
            med_speed = med_speed1 - 25
        if(abs(zhong2 - 64 )>3):
            med_speed = med_speed1 - 35
#     print(zhong2) 
    if ticker_flag:
        ccd_data1 = ccd.get(0)
        ccd_data2 = ccd.get(1)  
#         find_ccd_zhongzhi()
        ccd_ips()
        
        
        ticker_flag = False
    ips200_display()

            
        
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    

    gc.collect()













 


















