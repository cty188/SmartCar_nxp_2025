class PIDController:
    PID_POSITION = 0
    PID_DELTA = 1
    
    def __init__(self, mode, Kp, Ki, Kd, max_out, max_iout):
        """
        PID控制器初始化 - 每个实例独立维护状态
        
        参数:
        mode - PID模式: PID_POSITION(位置式) 或 PID_DELTA(增量式)
        Kp - 比例系数
        Ki - 积分系数
        Kd - 微分系数
        max_out - 最大输出限制
        max_iout - 积分项最大限制
        """
        # 控制参数
        self.mode = mode
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_out = max_out
        self.max_iout = max_iout
        
        # 历史状态数据 - 每个实例独立
        self.Dbuf = [0.0, 0.0, 0.0]  # 微分项历史数据 [当前, 前一次, 前两次]
        self.error = [0.0, 0.0, 0.0] # 误差历史数据 [当前, 前一次, 前两次]
        
        # 中间计算结果 - 每个实例独立
        self.Pout = 0.0
        self.Iout = 0.0
        self.Dout = 0.0
        self.out = 0.0
        
        # 输入输出数据 - 每个实例独立
        self.set = 0.0  # 目标值
        self.fdb = 0.0  # 反馈值

    def _limit_max(self, value : f32, max_val : f32) -> f32:
        """限制值在[-max_val, max_val]范围内"""
        if value > max_val:
            return max_val
        if value < -max_val:
            return -max_val
        return value

    def compute(self, feedback : f32, setpoint : f32) -> f32:
        """
        PID计算函数 - 独立执行，不影响其他PID实例
        
        参数:
        feedback - 当前反馈值
        setpoint - 目标设定值
        
        返回:
        PID控制输出
        """
        # 保存当前状态（仅用于显示，不影响计算）
        self.fdb = feedback
        self.set = setpoint
        
        # 更新误差历史（当前误差前移）
        self.error[2] = self.error[1]
        self.error[1] = self.error[0]
        self.error[0] = setpoint - feedback
        
        if self.mode == self.PID_POSITION:
            # === 位置式PID ===
            # 比例项
            self.Pout = self.Kp * self.error[0]
            
            # 积分项（累积并限制）
            self.Iout += self.Ki * self.error[0]
            self.Iout = self._limit_max(self.Iout, self.max_iout)
            
            # 微分项（差分计算）
            self.Dbuf[2] = self.Dbuf[1]
            self.Dbuf[1] = self.Dbuf[0]
            self.Dbuf[0] = self.error[0] - self.error[1]
            self.Dout = self.Kd * self.Dbuf[0]
            
            # 总输出 = P + I + D
            self.out = self.Pout + self.Iout + self.Dout
            
        elif self.mode == self.PID_DELTA:
            # === 增量式PID ===
            # 比例项
            self.Pout = self.Kp * (self.error[0] - self.error[1])
            
            # 积分项（当前误差积分）
            self.Iout = self.Ki * self.error[0]
            
            # 微分项（二阶差分）
            self.Dbuf[2] = self.Dbuf[1]
            self.Dbuf[1] = self.Dbuf[0]
            self.Dbuf[0] = self.error[0] - 2.0 * self.error[1] + self.error[2]
            self.Dout = self.Kd * self.Dbuf[0]
            
            # 总输出 = 上次输出 + Δ(P+I+D)
            self.out += self.Pout + self.Iout + self.Dout
        
        # 应用输出限制
        self.out = self._limit_max(self.out, self.max_out)
        return self.out

    def reset(self):
        """
        重置控制器状态 - 仅重置当前实例状态
        清除历史数据和中间计算结果
        """
        self.Dbuf = [0.0, 0.0, 0.0]
        self.error = [0.0, 0.0, 0.0]
        self.Pout = 0.0
        self.Iout = 0.0
        self.Dout = 0.0
        self.out = 0.0
        self.set = 0.0
        self.fdb = 0.0

    def get_params(self):
        """
        获取当前控制器参数
        用于调试或参数显示
        """
        return {
            'mode': self.mode,
            'Kp': self.Kp,
            'Ki': self.Ki,
            'Kd': self.Kd,
            'max_out': self.max_out,
            'max_iout': self.max_iout,
            'current_out': self.out
        }

    def get_state(self):
        """
        获取当前控制器状态
        用于调试或状态监控
        """
        return {
            'error': self.error.copy(),
            'Pout': self.Pout,
            'Iout': self.Iout,
            'Dout': self.Dout,
            'setpoint': self.set,
            'feedback': self.fdb,
            'output': self.out
        }
# PID控制器类 - 支持位置式和增量式PID