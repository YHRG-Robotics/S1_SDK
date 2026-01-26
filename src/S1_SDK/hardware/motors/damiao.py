import struct
# import serial
import math
import can
import time
import os
import sys
import numpy as np
root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_dir)
from S1_SDK.hardware.motors.base_motor import MotorStrategy 
from S1_SDK.hardware.com.base_com import ReturnFrame
class Damiao(MotorStrategy):
    def __init__(self,id):
        self.id = id
        self.pos = 0
        self.vel = 0
        self.tau= 0
        self.temperture_mos = 0
        self.temperture = 0
        self.mode = 0
        self.limit_param_4310 = [12.5, 30, 10]
        self.limit_param_4340 = [12.5, 8, 28]
        self.status = 0x00
        self.MIT_param = [[54,5],[84,4],[84,4],[10,1],[10,1],[5,0.1],[5,0.1]]
    #对外接口部分
    def init_motor(self):
        return_data = []
        return_data.append(self.disable())
        return_data.append(self.switchmode(1))
        # time.sleep(0.01)
        return return_data

    
    def disable(self):
        return ReturnFrame(self.id,[0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfd])
        # 失能电机
    def enable(self):
        # 使能电机
        return ReturnFrame(self.id,[0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc])
    def set_zero_position(self):
        # 设置电机零位
        return ReturnFrame(self.id,[0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfe])
    def refresh_motor_status(self):
        # 刷新电机状态
        return ReturnFrame(0x7ff,[self.id,0x00,0xcc,0x00,0x00,0x00,0x00,0x00])
    def switchmode(self,mode):
        # print(f"switch mode to {mode},id:{self.id},mode:{self.mode}")
        if mode >2 or mode <=0:
            return None
        if self.mode != mode:
            self.mode = mode
            # print(f"switch mode to {mode}")
            return ReturnFrame(0x7ff,[self.id,0x00,0x55,0x0a,mode,0x00,0x00,0x00])
    def control_pos_vel(self,pos,vel):
        if self.mode != 2:
            return None
        return self.control_cmd(self.id,pos,vel)
        # return True
    def control_foc(self,tau):
        if self.mode != 1:
            return None
        return self.control_MIT_cmd(self.id,0,0,0,0,tau)
        # return True
    def control_pos(self,pos,tau):
        if self.mode != 1:
            return None
        return self.control_MIT_cmd(self.id,self.MIT_param[self.id-1][0],self.MIT_param[self.id-1][1],pos,0,tau)
    def control_MIT_cmd(self,motor_id,kp: float, kd: float, q: float, dq: float, tau: float):
        # 位置速度控制
        kp_uint = self.__float_to_uint(kp, 0, 500, 12)
        kd_uint = self.__float_to_uint(kd, 0, 5, 12)
        
        if motor_id <= 3:
            Q_MAX = self.limit_param_4340[0]
            DQ_MAX = self.limit_param_4340[1]
            TAU_MAX = self.limit_param_4340[2]
        else:
            Q_MAX = self.limit_param_4310[0]
            DQ_MAX = self.limit_param_4310[1]
            TAU_MAX = self.limit_param_4310[2]
        q_uint = self.__float_to_uint(q, -Q_MAX, Q_MAX, 16)
        dq_uint = self.__float_to_uint(dq, -DQ_MAX, DQ_MAX, 12)
        tau_uint = self.__float_to_uint(tau, -TAU_MAX, TAU_MAX, 12)
        data_buf = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        data_buf[0] = (q_uint >> 8) & 0xff
        data_buf[1] = q_uint & 0xff
        data_buf[2] = dq_uint >> 4
        data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf)
        data_buf[4] = kp_uint & 0xff
        data_buf[5] = kd_uint >> 4
        data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf)
        data_buf[7] = tau_uint & 0xff
        return ReturnFrame(motor_id,data_buf)
    def control_cmd(self,motor_id,P_desired,V_desired):
        # 位置速度控制
        motor_id = motor_id + 0x100
        data_buf = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        P_desired_uint8s = self.__float_to_uint8s(P_desired)
        V_desired_uint8s = self.__float_to_uint8s(V_desired)
        data_buf[0:4] = P_desired_uint8s
        data_buf[4:8] = V_desired_uint8s
        return ReturnFrame(motor_id,data_buf)
    
    def proccess(self,frame):
        id = frame.id
        # frame_bytes = bytes(frame.data)
        # print(frame_bytes.hex(" "))
        # print(id,self.id)
        if id <= 3:
            Q_MAX = self.limit_param_4340[0] 
            DQ_MAX = self.limit_param_4340[1]
            TAU_MAX = self.limit_param_4340[2]
        else:
            Q_MAX = self.limit_param_4310[0] 
            DQ_MAX = self.limit_param_4310[1]
            TAU_MAX = self.limit_param_4310[2]
        self.status = (frame.data[0] & 0xf0)>>4
        q_uint = np.uint16((np.uint16(frame.data[1]) << 8) | frame.data[2])
        dq_uint = np.uint16((np.uint16(frame.data[3]) << 4) | (frame.data[4] >> 4))
        tau_uint = np.uint16(((frame.data[4] & 0xf) << 8) | frame.data[5])
        self.pos = self.__uint_to_float(q_uint, -Q_MAX, Q_MAX, 16)
        self.spd = self.__uint_to_float(dq_uint, -DQ_MAX, DQ_MAX, 12)
        self.tau = self.__uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)
        self.temperture_mos= frame.data[6]
        self.temperture = frame.data[7]
        # print(pos)


    def __LIMIT_MIN_MAX(self, x, min, max):
        if x <= min:
            x = min
        elif x > max:
            x = max

    def __float_to_uint(self, x: float, x_min: float, x_max: float, bits):
        self.__LIMIT_MIN_MAX(x, x_min, x_max)
        span = x_max - x_min
        data_norm = (x - x_min) / span
        return np.uint16(data_norm * ((1 << bits) - 1))
    def __uint_to_float(self,x: np.uint16, min: float, max: float, bits):
        span = max - min
        data_norm = float(x) / ((1 << bits) - 1)
        temp = data_norm * span + min
        return np.float32(temp)
    
    def __float_to_uint8s(self,value):
        # Pack the float into 4 bytes
        packed = struct.pack('f', value)
        # Unpack the bytes into four uint8 values
        return struct.unpack('4B', packed)
# -------------------------
# 使用示例
# -------------------------
def read_all(bus):
    responses = []
    start_time = time.time()
    
    # 收集所有可用消息
    while time.time() - start_time < 0.002:  # 最多等待2ms
        try:
            response = bus.recv(0.001)  # 超时1ms
            # response.arbitration_id
            if response:
                responses.append(response)
        except:
            break
    return responses
if __name__ == "__main__":
    bus = can.interface.Bus(interface='socketcan', channel='motor', bitrate=1000000)
    motor_list = [1,2,3,4,5]
    
    
    motor = [0] * len(motor_list)
    
    for i in range(len(motor_list)):
        motor[i] = Damiao(bus,motor_list[i])
        motor[i].init_motor()
        time.sleep(0.1)
        # motor[i].disable()
    pos  = [0,0,0,0,0,0]
    try:
        while True:
            for i in range(len(motor_list)):
                motor[i].refresh_motor_status()
            frame = read_all(bus)
            for f in frame:
                id = f.arbitration_id & 0x0f
                print(id)
                motor[id-1].proccess(f)
                pos[id-1] = motor[id-1].pos
            print(pos)
            time.sleep(0.01)
    except KeyboardInterrupt:
        for i in range(6):
            motor[i].disable()
