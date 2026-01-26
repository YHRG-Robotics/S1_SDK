import sys
import os

# 添加项目根目录到 Python 路径
install_kdl = True
from S1_SDK.hardware.motor import Motor_Pro
from S1_SDK.hardware.mujoco_sim import Mujoco
try:
    from S1_SDK.hardware.kdl_solver import KDLRobotSolver
except ImportError:
    pass
    install_kdl = False
from S1_SDK.hardware.collision import CollisionChecker
from S1_SDK.hardware.gravity import ArmGravityCalculator
# import numpy as math
from enum import IntEnum
import time 
import math
from typing import Dict
from S1_SDK.base_mode import ControlStrategy
from S1_SDK.only_real import OnlyRealStrategy
from S1_SDK.only_sim import OnlySimStrategy
from S1_SDK.real_control_sim import RealControlSimStrategy
# from smotor_master import SmotorMaster
##控制模式枚举
class control_mode(IntEnum):
    only_sim = 0
    only_real = 1
    real_control_sim = 3
# 策略映射
STRATEGY_MAP: Dict[control_mode, ControlStrategy] = {
    control_mode.only_sim: OnlySimStrategy(),
    control_mode.only_real: OnlyRealStrategy(),
    control_mode.real_control_sim: RealControlSimStrategy(),
}
end_checker = ["None","gripper","teach"]
range_checker = [[math.radians(-170),math.radians(170)],
                  [math.radians(0),math.radians(180)],
                  [math.radians(0),math.radians(170)],
                  [math.radians(-90),math.radians(87)],
                  [math.radians(-90),math.radians(90)],
                  [math.radians(-90),math.radians(90)],
                  [math.radians(-100),math.radians(100)],
                  ]
def clamp(value, checker):
    # for i in range(len(value)):
    if value < checker[0]:
        value = checker[0]
    elif value > checker[1]:
        value = checker[1]
    return value
class S1_arm:
    def __init__(self,mode:control_mode,dev:str="can0",end_effector:str="None",check_collision:bool=True,arm_version:str="V2"):
        """
        初始化S1_arm类
        :param mode: 控制模式, 可选值为control_mode枚举中的值
        :param dev: 电机通信设备, 默认值为"can0"
        :param end_effector: 末端类型, 可选值为"None","gripper","teach"
        :param check_collision: 是否检查碰撞, 默认值为True
        :param arm_version: 机械臂版本, 默认值为"V2"
        """
        if end_effector not in end_checker:
            sys.exit(f"末端执行器错误,只能为{end_checker},当前为{end_effector}")
        self.end_effector = end_effector
        self.gripper_need = False
        if end_effector != "None":
            self.gripper_need = True
        self.motor = None
        self.sim = None
        if install_kdl:
            self.kdl_solver = KDLRobotSolver()
        if check_collision:
            self.collision_checker = CollisionChecker(end_effector)
        else:
            self.collision_checker = None
        self.gravity_calculator = ArmGravityCalculator(end_effector)
        self.__init_arm(dev,mode,arm_version)
        if self.strategy.needs_motor():
            self.motors = len(self.motor.motors)
        else:
            self.motors = 7
        

    ###控制块###
    def joint_control_mit(self,pos = None,fix_tau:list=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]):
        """
        MIT关节控制,控制六个关节
        :param pos: 六个关节的位置,列表形式,每个元素为一个关节的位置
        """
        if len(pos) != 6 or pos == None:
            return False
        for i in range(len(pos)):
            pos[i] = clamp(pos[i],range_checker[i])
        if self.collision_checker is None:
            pass
        else:
            if self.collision_checker.check_collision(pos):
                print(f"time:{time.time()} 碰撞检测到, 控制被阻止")
                return False
        tau = self.gravity(return_tau=True)
        if self.motor is None:
            return self.strategy.joint_control(self, pos)       
        return_state = self.motor.control_pos(pos,tau)
        return return_state

    def joint_control(self,pos = None):
        """
        关节控制,控制六个关节
        :param pos: 六个关节的位置,列表形式,每个元素为一个关节的位置
        """
        if pos is None or len(pos) != 6:
            return False
        # for i in range(len(pos)):
        #     pos[i] = clamp(pos[i],range_checker[i])
        if self.collision_checker is None:
            pass
        else:
            if self.collision_checker.check_collision(pos):
                print(f"time:{time.time()} 碰撞检测到, 控制被阻止")
                return False
        return self.strategy.joint_control(self, pos)       
    def control_gripper(self,pos,force):
        return self.motor.control_gripper(pos,force)
    def control_teach(self,tau):
        """
        示教控制,控制六个关节
        :param tau: 六个关节的力矩,列表形式,每个元素为一个关节的力矩
        """
        # print(tau)
        return self.motor.control_teach(tau)
    def enable(self):
        """
        使能电机
        """
        if self.strategy.needs_motor():
            self.motor.enable_all()
    def disable(self):
        """
        失能电机
        """
        if self.strategy.needs_motor():
            self.motor.disable_all()
    def refresh(self):
        """
        刷新仿真,更新当前状态
        """
        self.strategy.refresh(self)

    def set_zero_position(self):
        """
        设置所有电机的零位
        """
        self.motor.set_zero_position_all()

    def set_end_zero_position(self):
        """
        设置末端零位
        """
        if self.strategy.needs_motor():
            self.motor.set_end_zero_position()
            # time.sleep(0.1)

    def get_fk_quat(self, joint_values):
        """
        获取正运动学结果
        :param joint_values: 关节角列表 [q1, q2, ..., q6]
        :return: 末端位姿 (x, y, z, qx, qy, qz, qw)
        """
        if install_kdl:
            return self.kdl_solver.fk_quat(joint_values)
        else:
            print("未安装KDL求解器，无法获取正运动学结果")
            return None
    def get_fk_euler(self, joint_values):
        """
        获取正运动学结果
        :param joint_values: 关节角列表 [q1, q2, ..., q6]
        :return: 末端位姿 (x, y, z, r, p, y)
        """
        if install_kdl:
            return self.kdl_solver.fk_euler(joint_values)
        else:
            print("未安装KDL求解器，无法获取正运动学结果")
            return None
    def get_ik_quat(self, target_pose, q_init=None):
        """
        获取逆运动学结果
        :param target_pose: 目标末端位姿 (x, y, z, qx, qy, qz, qw)
        :param q_init: 初始关节角列表 [q1, q2, ..., q6], 默认None
        :return: 关节角列表 [q1, q2, ..., q6] 或 None (如果失败)
        """
        if install_kdl:
            ik_solution = self.kdl_solver.ik_quat(target_pose, q_init)
            if ik_solution is None:
                # print("IK 求解失败，无法控制末端")
                return None
            # for i in range(len(ik_solution)):
            #     if ik_solution[i] < range_checker[i][0]:
            #         print("IK 求解失败，输出关节角超出范围")
            #         return None
            #     elif ik_solution[i] > range_checker[i][1]:
            #         print("IK 求解失败，输出关节角超出范围")
            #         return None
            return ik_solution
        else:
            print("未安装KDL求解器，无法获取逆运动学结果")
            return None
    def get_ik_euler(self, target_pose, q_init=None):
        """
        获取逆运动学结果
        :param target_pose: 目标末端位姿 (x, y, z, r, p, y)
        :param q_init: 初始关节角列表 [q1, q2, ..., q6], 默认None
        :return: 关节角列表 [q1, q2, ..., q6] 或 None (如果失败)
        """
        if install_kdl:
            ik_solution = self.kdl_solver.ik_euler(target_pose, q_init)
            if ik_solution is None:
                return None
            # for i in range(len(ik_solution)):
            #     if ik_solution[i] < range_checker[i][0]:
            #         print("IK 求解失败，输出关节角超出范围")
            #         return None
            #     elif ik_solution[i] > range_checker[i][1]:
            #         print("IK 求解失败，输出关节角超出范围")
            #         return None
            return ik_solution
        else:
            print("未安装KDL求解器，无法获取逆运动学结果")
            return None
        
        
    def end_effector_control(self,pos):
        """
        末端执行器控制: 控制末端执行器的位置
        :param pos: 末端执行器的位置, 列表形式,每个元素为一个坐标或完整位姿 [x,y,z,qx,qy,qz,qw]
        """
        if install_kdl:
            ik_solution  = None
            if len(pos) == 3:
                x, y, z = pos
                qx, qy, qz, qw = 0, 0, 0, 1.0  # 默认朝向
                target_pose = (x, y, z, qx, qy, qz, qw)
            elif len(pos) == 6:
                target_pose = tuple(pos)
                ik_solution = self.get_ik_euler(target_pose)
            elif len(pos) == 7:
                target_pose = tuple(pos)
                ik_solution = self.get_ik_quat(target_pose)
            else:
                raise ValueError("pos 必须为长度 3 或 7 的列表或元组")
            
            
            if ik_solution is None:
                # print("IK 求解失败，无法控制末端")
                return
            print(f"ik_solution: {[f'{x:.2f}' for x in ik_solution]}")
                    
            self.joint_control(ik_solution)
        else:
            print("未安装KDL求解器，无法控制末端")
            return False



    def gravity(self,return_tau=False):
        """
        :param return_tau: 是否返回重力补偿力矩,默认False
        :return: 如果return_tau为True,返回当前位置下重力补偿应输出力矩列表,否则直接控制电机
        """
        qpos = self.get_pos()
        # qpos[2] = qpos[2]/4
        tau = [0.0]*6
        tau = self.gravity_calculator.get_tau_numeric(qpos[:5])
        if return_tau:
            return tau
        self.motor.control_foc(tau[:6])
        
    #####控制块END###

    def check_collision(self, qpos, gripper=False):
        """
        检查给定关节角度下是否发生自碰撞
        :param qpos: 关节角度列表,长度为6
        :param gripper: 是否检查夹爪,默认不带
        :return: 如果发生碰撞返回True,否则返回False
        """
        if gripper is None:
            gripper = self.gripper
        return self.collision_checker.check_collision(qpos, gripper)

    ######状态块#####
    def get_pos(self):
        """
        获取当前关节位置
        :return: 关节位置列表,每个元素为一个关节的位置
        """
        if self.strategy.needs_motor():
            return self.motor.pos
        elif self.strategy.needs_sim():
            return self.sim.data.qpos
    def get_vel(self):
        """
        获取当前关节速度
        :return: 关节速度列表,每个元素为一个关节的速度
        """
        if self.strategy.needs_motor():
            return self.motor.spd
        else:
            return [0.0]*len(self.motor_list)
    def get_tau(self):
        """
        获取当前关节力矩
        :return: 关节力矩列表,每个元素为一个关节的力矩
        """
        if self.strategy.needs_motor():
            return self.motor.tau
        else:
            return [0.0]*6
    def get_temp(self):
        """
        获取当前关节温度
        :return: 关节温度列表,每个元素为一个关节的温度
        """
        if self.strategy.needs_motor():
            return self.motor.temperture_rot
        else:
            return [0.0]*len(self.motor_list)
    #####状态块END###
        
    def close(self):
        """
        关闭电机
        """
        if self.strategy.needs_motor():
            self.motor.close()
    def __init_arm(self,dev,mode,arm_version):
        self.strategy = STRATEGY_MAP.get(mode)
        if not self.strategy:
            raise ValueError(f"模式错误(mode error){mode}")

        if self.strategy.needs_motor():
            self.motor = Motor_Pro(dev,"S1",end_effector=self.end_effector,sdk_version=arm_version)
        if self.strategy.needs_sim():
            script_dir = os.path.dirname(os.path.abspath(__file__))
            if self.end_effector == "None":
                self.sim = Mujoco(os.path.join(script_dir,'resource/meshes/gripper_less.xml'))
            elif self.end_effector == "gripper" or self.end_effector == "teach":
                self.sim = Mujoco(os.path.join(script_dir,'resource/meshes/gripper.xml'))
            

   