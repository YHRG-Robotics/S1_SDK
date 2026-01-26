# import can
import struct
import time
import numpy as np
import os
import sys
import math
from dataclasses import dataclass
root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_dir)
from typing import Dict
from S1_SDK.hardware.motors.damiao import Damiao
from S1_SDK.hardware.motors.base_motor import MotorStrategy
from S1_SDK.hardware.com.can_type import CanType
from S1_SDK.hardware.com.uart_type import UartType
from S1_SDK.hardware.com.base_com import ComStrategy,ReturnFrame
S1: Dict[int, MotorStrategy] = {
    1: Damiao,
    2: Damiao,
    3: Damiao,
    4: Damiao,
    5: Damiao,
    6: Damiao,
    7: Damiao,
    8: Damiao,
}
COMM_TYPE :Dict[str,ComStrategy] = {
    "V1": CanType,
    "V2": UartType,
}
# 8——超压；
# 9——欠压；
# A——过电流；
# B——MOS过温；
# C——电机线圈过温；
# D——通讯丢失；
# E——过载；
Error_Type = {
    0x08: "超压",
    0x09: "欠压",
    0x0A: "过流",
    0x0B: "MOS过温",
    0x0C: "电机线圈过温",
    0x0D: "通讯丢失",
    0x0E: "过载",
}
# from S1_SDK.hardware.kdl import KDLRobotSolver
@dataclass
class Motor_Status:
    id: int
    mode: int
class Motor_Pro:
    def __init__(self, bus:str,version:str="S1",end_effector:str="None",sdk_version:str="V1"):
        self.com_port = bus
        try:
            self.bus = COMM_TYPE[sdk_version](bus)
        except:
            # raise ValueError(f"版本错误(version error){sdk_version}")
            raise
            # sys.exit("通信错误(bus error)")
        self.gipper_need = False
        self.hand_id = 0 #0x8E为左手，10E为右手
        

        res = self.ping_motor(end_effector) # 检测总线电机

        if res is None:
            # raise
            sys.exit("电机缺失(motor not found)")
        motor_id = []
        for motor in res:
            motor_id.append(motor.id)
        self.motor_id = motor_id
        self.send_cmd_flag = [0]*len(motor_id)
        self.motors = [None]*len(motor_id)
        self.online_motors = [0] * len(motor_id)
        self.send_cmd_timestamp = [0.0]*len(motor_id)
        self.recv_cmd_timestamp = [0.0]*len(motor_id)
        self.system_delay = [0.0]*len(motor_id)

        #初始化电机
        ver = None
        if version == 'S1':
            ver = S1
        else:
            raise ValueError(f"版本错误(version error){version}")

        for i in range(len(motor_id)):
            motor_class = ver.get(self.motor_id[i])
            if motor_class is not None:
                self.motors[i] = motor_class(self.motor_id[i])
                res = self.motors[i].init_motor()
                self.send_commands(res)
        self.__read_all_messages()
        self.pos = np.zeros(len(motor_id))
        self.spd = np.zeros(len(motor_id))
        self.tau = np.zeros(len(motor_id))
        self.temperture_rot = np.zeros(len(motor_id)) 

        self.pid_param = [[2.0,0.05,0.1],[20.0,0.15,1.1],[2.0,0.05,0.1],[10.0,0.05,0.1],[2.0,0.05,0.1],[2.0,0.05,0.1],[2,0.05,0.1]]
        self.error = np.zeros(len(motor_id))
        self.last_error = np.zeros(len(motor_id))
        self.fix_tau = np.zeros(len(motor_id))
        self.sum_error = np.zeros(len(motor_id))

    
    def enable(self, motor_id):
        # 使能电机
        res = []
        if motor_id == 8:
            res = self.motors[motor_id-2].enable()
        else:
            res = self.motors[motor_id-1].enable()
        self.send_commands(res)

    def disable(self, motor_id):
        # 失能电机
        res = []

        if motor_id > 6:
            res = self.motors[6].disable()
        else:
            res = self.motors[motor_id-1].disable()
        self.send_commands(res)

    
    def disable_all(self):
        # 失能所有电机
        res = []
        for i in range(len(self.motor_id)):
            res.append(self.motors[i].disable())
            # print(res)
        self.send_commands(res)
        #     time.sleep(0.1)
    def enable_all(self):
        # 使能所有电机
        res = []
        for i in range(len(self.motor_id)):
            res.append(self.motors[i].enable())
        self.send_commands(res)

    def set_zero_position(self, motor_id):
        # 设置电机零位
        res = []
        if motor_id in self.motor_id:
            index = self.motor_id.index(motor_id)
            res.append(self.motors[index].set_zero_position())
        else:
            raise ValueError(f"电机ID {motor_id} 不存在")
        # print(res)
        self.send_commands(res)
    def set_zero_position_all(self):
        # 设置所有电机零位
        res = []
        for i in range(len(self.motor_id)):
            res.append(self.motors[i].set_zero_position())
        self.send_commands(res)
    def set_end_zero_position(self):
        # 设置末端零位
        res = []
        if self.gipper_need:
            res.append(self.motors[-1].set_zero_position())
            self.send_commands(res)
    def refresh_motor_status(self):
        # 刷新电机状态
        res = []
        for i in range(len(self.motor_id)):
            if self.send_cmd_flag[i] == 1:
                self.send_cmd_flag[i] == 0
                continue
            self.send_cmd_timestamp[i] = time.time()
            res.append(self.motors[i].refresh_motor_status())
        self.send_commands(res)
        self.__read_motor_status_batch()
        #计算控制延迟
        for i in range(len(self.motor_id)):
            self.system_delay[i] = self.recv_cmd_timestamp[i] - self.send_cmd_timestamp[i]
            if self.system_delay[i] < -1.0:
                print(f"{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.recv_cmd_timestamp[i]))} 端口 {self.com_port} 电机 {self.motor_id[i]} 掉线")
            

    def switchmode(self,index,mode):
        res = []
        res.append(self.motors[index].switchmode(mode))
        self.send_commands(res)
    def control_Pos_Vel(self,pos):
        
        res = []
        # print(pos)
        for i in range(6):
            if self.motors[i].mode != 2:
                self.switchmode(i,2) 
                # print(f"切换到位置速度模式 {self.motor_id[i]}")
                continue
            else:
                # print(f"执行位置速度控制 {self.motor_id[i]}")
                self.send_cmd_flag[i] = 1
                self.send_cmd_timestamp[i] = time.time()
                res.append(self.motors[i].control_pos_vel(pos[i],20))
        self.send_commands(res)
        
    def control_foc(self,tau):
        
        res = []
        for i in range(len(tau)):
            if self.motors[i].mode != 1:
                self.switchmode(i,1)
                continue
            self.send_cmd_flag[i] = 1
            self.send_cmd_timestamp[i] = time.time()
            res.append(self.motors[i].control_foc(tau[i]))
        self.send_commands(res)

    def control_pos(self,pos,tau):
        
        res = []
        # print(len(pos),len(tau))
        self.refresh_motor_status()
        self.pid_cal(pos)
        for i in range(6):
            if self.motors[i].mode != 1:
                self.switchmode(i,1)
                continue
            self.send_cmd_flag[i] = 1
            self.send_cmd_timestamp[i] = time.time()
            res.append(self.motors[i].control_pos(pos[i],tau[i]+self.fix_tau[i]))
        self.send_commands(res)
    def control_gripper(self,pos,tau):

        res = []
        if self.gipper_need:
            if self.motors[-1].mode != 1:
                self.switchmode(6,1)
            temp = pos - self.pos[-1]
            
            if temp>0.5:
                self.send_cmd_flag[6] = 1
                self.send_cmd_timestamp[6] = time.time()
                res.append(self.motors[-1].control_pos(pos,tau))
            else:
                self.send_cmd_flag[6] = 1
                self.send_cmd_timestamp[6] = time.time()
                res.append(self.motors[-1].control_pos(pos,0))
        self.send_commands(res)
    def control_hand(self,pos):
        data = [0,0,0,0,0,0]
        if pos > 0.0:
            pos = 0.0
        if pos < -2.0:
            pos = -2.0
        temp = int(-(pos / 2.0) *1000)
        # temp = temp -500
        temp = 1000 - temp
        if temp>1000:
            temp = 1000
        if temp < 600 and temp > 300:
            data[1] = 1000
            data[0] = int((temp-300)*0.66)
            data[2] = int((temp-300)*0.66)
        elif temp >= 600 and temp<=800:
            data[1] = 1000
            data[0] = 200
            data[2] = 200
            for i in range(len(data)):
                if i == 1 or i == 0 or i == 2:
                    continue
                data[i] = (temp - 600) 
        elif temp>800:
            data[1] = 1000 - (temp - 800)* 1.2
            data[0] = (temp -800) * 1.9 + 200
            data[2] = (temp -800) * 1.9 + 200
            data[3] = int((temp - 800) * 4)+120
            data[4] = int((temp - 800) * 4)+120
            data[5] = int((temp - 800) * 4)+120
        elif temp<=300 and temp > 200:
            data[1] = int((temp-200)*10)
            data[0] = 0
            data[2] = 0
        elif temp < 140 and temp > 20:
            data[0] = int(800 - (temp - 20)*6.6)
            data[1] = int(400 - (temp - 20)*3.3)
            data[2] = 0
            data[3] = int(1000 - (temp - 20)*8.3)
            data[4] = int(1000- (temp - 20)*8.3)
            data[5] = int(1000- (temp - 20)*8.3)
        elif temp < 20:
            data[0] = 800
            data[1] = 400
            data[2] = 0
            data[3] = 1000
            data[4] = 1000
            data[5] = 1000
        for i in range(len(data)):
            data[i] = int(data[i] / 10)
        frame_data = [0] * 8
        frame_data[2:] = data
        frame = ReturnFrame(self.hand_id,data)
        frame.data[2:] = data
        res = []
        res.append(frame)
        self.send_commands(res)
    def control_teach(self,tau):
        res = []
        if self.gipper_need:
            if self.motors[-1].mode != 1:
                self.switchmode(6,1)
            else:
                tau = tau/math.cos(math.fabs(self.pos[-1] + math.degrees(30))) 
                if self.spd[-1]>0.5:
                    tau = -tau/3
                if self.pos[-1] > -1.0:
                    res.append(self.motors[-1].control_foc(tau))
                else:
                    res.append(self.motors[-1].control_foc(0))
                self.send_commands(res)
        else:
            return False
    def send_commands(self,res):
        if res is None:
            return False
        if type(res) == ReturnFrame:
            if res.id == 0x7ff and res.data[2] == 0x55:
                    time.sleep(0.01)
            self.__write_data(res.id,res.data)
        else:
            for frame in res:
                if frame is None:
                    continue
                self.__write_data(frame.id,frame.data)
                if frame.id == 0x7ff and frame.data[2] == 0x55:
                    time.sleep(0.01)
                time.sleep(0.0001)
    def pid_cal(self,tar_pos):
        for i in range(6):
            self.error[i] = tar_pos[i] - self.pos[i]
            if abs(self.error[i]) > 0.02:
                self.sum_error[i] += self.error[i]
            self.fix_tau[i] = self.pid_param[i][0]*self.error[i] + self.pid_param[i][1]*self.sum_error[i] - self.pid_param[i][2]*self.last_error[i]
            self.last_error[i] = self.error[i]
        return self.fix_tau


    def __read_motor_status_batch(self):
        responses = self.__read_all_messages()
        # 处理所有收集到的消息
        for response in responses:
            self.__process_motor_status(response)

    def __process_motor_status(self, response):
        """处理单个电机状态消息"""
        id = response.id & 0x0f
        if id in self.motor_id:
            index = self.motor_id.index(id)
            self.recv_cmd_timestamp[index] = time.time()   
            self.motors[index].proccess(response)
            self.pos[index] = self.motors[index].pos
            self.tau[index] = self.motors[index].tau
            self.spd[index] = self.motors[index].spd
            self.temperture_rot[index] = self.motors[index].temperture
    def __write_data(self,id,data):
        # 通信底层
        self.bus.send(id,data)
    def get_fix_tau(self):
        return self.fix_tau.copy()
    def ping_motor(self,end_effector):
        start_frame = ReturnFrame(0xfe,[0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]) #启用板载passby
        self.send_commands(start_frame)
        modes = []
        ids = []
        motor_status = []
        truly_id = [1,2,3,4,5,6]
        if end_effector == "teach":
            truly_id.append(8)
            self.gipper_need = True
        elif end_effector == "gripper":
            truly_id.append(7)
            self.gipper_need = True
        elif end_effector == "left_hand":
            self.hand_id = 0x8E
        elif end_effector == "right_hand":
            self.hand_id = 0x10E
        res = []
        time.sleep(0.01)
        for i in range(10):
            res = (ReturnFrame(0x7ff,[i+1,0x00,0x33,0x0a,0x00,0x00,0x00,0x00]))
            self.send_commands(res)
            time.sleep(0.01)
        responses = self.__read_all_messages()
        for response in responses:
            id = response.id & 0x0f
            if response.data[2] == 0x33 and response.data[3] == 0x0a:
                modes.append(response.data[4])
                ids.append(id)
        for i in range(len(ids)):
            if end_effector == "None" and (ids[i] == 8 or ids[i] == 7):
                continue
            motor_status.append(Motor_Status(id = ids[i],mode = modes[i]))
        for motor in motor_status:
            if motor.id in truly_id:
                truly_id.remove(motor.id)
        if len(truly_id) != 0:
            print("未检测到机械臂")
            print("未检测到电机ID:",truly_id,"机械臂故障")
            return None
        return motor_status
    def __read_all_messages(self):
        responses = []
        while True:
            response = self.bus.recv(0)  # 不等待，立即返回或 None
            if response is None:
                # print("recv None")
                break  # 一旦没消息，立刻跳出
                
            responses.append(response)
            # print("res:",responses)
        
        return responses
    def close(self):
        target_positions = np.zeros(len(self.motor_id))
        
        
        time.sleep(0.1)
        self.control_Pos_Vel(target_positions)
        self.control_Pos_Vel(target_positions)
        self.control_Pos_Vel(target_positions)
        self.control_Pos_Vel(target_positions)
        time.sleep(2)
        self.disable_all()
        self.disable_all()
        self.disable_all()
        end_frame = ReturnFrame(0xfd,[0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
        self.send_commands(end_frame)
        self.bus.close()
            # print(i)

if __name__ == "__main__":
    motor = Motor_Pro("COM18")
    tar_pos = np.zeros(6)
    for i in range(6):
        if motor.motors[i].mode != 2:
            motor.switchmode(i,2)
    motor.enable_all()
    try:
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        motor.close()
