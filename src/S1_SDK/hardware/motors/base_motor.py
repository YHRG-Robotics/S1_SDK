from abc import ABC, abstractmethod
from typing import List
from dataclasses import dataclass


class MotorStrategy(ABC):
    """控制策略抽象基类"""
    @abstractmethod
    def enable(self):
        pass
    @abstractmethod
    def disable(self):
        pass
    @abstractmethod
    def init_motor(self):
        pass
   
    def control_pos_vel(self,pos,vel):
        "单位为°"
        pass
    @abstractmethod
    def control_foc(self,tau):
        pass
    @abstractmethod
    def set_zero_position(self):
        # 设置电机零位
        pass 
    @abstractmethod
    def refresh_motor_status(self):
        # 刷新电机状态
        pass
    @abstractmethod
    def switchmode(self,mode):
        pass
    @abstractmethod
    def proccess(self,mode):
        pass
    @abstractmethod
    def control_foc(self,mode):
        pass