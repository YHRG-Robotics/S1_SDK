import sympy as sp
import math
import numpy as np
from dataclasses import dataclass
@dataclass
class mass_point:
    mass: float
    x: float
    y: float
def rotate(vector, angle):
    R = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle),  np.cos(angle)]
    ])
    v = np.array(vector).reshape(2, 1)  # 列向量 (2,1)
    v_rot = R @ v                       # 矩阵乘法
    return v_rot.flatten()              # 转回一维数组
class ArmGravityCalculator:
    def __init__(self,gripper:str="None"):
        # 定义符号变量
        self.q1, self.q2, self.q3, self.q4, self.q5 = sp.symbols('q1 q2 q3 q4 q5')
        self.q_syms = [self.q1, self.q2, self.q3, self.q4, self.q5]

        self.gripper_status = False
        self.teach_gripper_status = False

        if gripper == "gripper":
            self.tau = [
                sp.S(0),  # tau1
                # tau2
                + 1.11408743951156*sp.sin(self.q2 - self.q3)
                + 0.398214339527504*sp.sin(self.q2 - self.q3 + self.q4)
                - 8.06377168222792*sp.cos(self.q2)
                + 0.0146842753117525*sp.cos(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                + 4.06826214177259*sp.cos(self.q2 - self.q3)
                + 1.07487101378681*sp.cos(self.q2 - self.q3 + self.q4),
                # tau3
                - 4.06826214177259*sp.sin(self.q2)*sp.sin(self.q3)
                - 1.11408743951156*sp.sin(self.q2)*sp.cos(self.q3)
                + 1.11408743951156*sp.sin(self.q3)*sp.cos(self.q2)
                - 0.398214339527504*sp.sin(self.q2 - self.q3 + self.q4)
                - 4.06826214177259*sp.cos(self.q2)*sp.cos(self.q3)
                - 0.0146842753117525*sp.cos(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                - 1.07487101378681*sp.cos(self.q2 - self.q3 + self.q4),
                # tau4
                - 1.15980617183329e-4*sp.sin(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                + 0.459596402817037*sp.sin(self.q2 - self.q3 + self.q4)
                + 2.93797117225226e-2*sp.cos(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                + 1.64777501378681*sp.cos(self.q2 - self.q3 + self.q4),

                # tau5
                - 5.79900955813156e-5*sp.sin(self.q2 - self.q3 + self.q4 - self.q5)
                - 5.79905216020131e-5*sp.sin(self.q2 - self.q3 + self.q4 + self.q5)
                - 1.46898019024078e-2*sp.cos(self.q2 - self.q3 + self.q4 - self.q5)
                + 1.46899098201148e-2*sp.cos(self.q2 - self.q3 + self.q4 + self.q5)
            ]
            self.gripper_status = True
        elif gripper == "teach":
            self.tau = [
                sp.S(0),  # tau1

                # tau2
                - 1.77008636070792e-16*sp.sin(self.q2)
                - 5.39383550138731e-8*sp.sin(self.q5)*sp.sin(self.q2 - self.q3 + self.q4)
                - 5.79682785737267e-5*sp.sin(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                + 1.18051230954933*sp.sin(self.q2 - self.q3)
                - 2.12929427959102e-10*sp.sin(self.q2 - self.q3 + self.q4)*sp.cos(self.q5)
                + 0.398214339527504*sp.sin(self.q2 - self.q3 + self.q4)
                - 8.21428118673767*sp.cos(self.q2)
                + 0.0146842753117525*sp.cos(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                + 4.25650991180939*sp.cos(self.q2 - self.q3)
                + 1.07487101378681*sp.cos(self.q2 - self.q3 + self.q4),

                # tau3
                + 5.39383550138731e-8*sp.sin(self.q5)*sp.sin(self.q2 - self.q3 + self.q4)
                + 5.79682785737267e-5*sp.sin(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                - 1.18051230954933*sp.sin(self.q2 - self.q3)
                + 2.12929427959102e-10*sp.sin(self.q2 - self.q3 + self.q4)*sp.cos(self.q5)
                - 0.398214339527504*sp.sin(self.q2 - self.q3 + self.q4)
                - 0.0146842753117525*sp.cos(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                - 4.25650991180939*sp.cos(self.q2 - self.q3)
                - 1.07487101378681*sp.cos(self.q2 - self.q3 + self.q4),

                # tau4
                - 5.39383550138731e-8*sp.sin(self.q5)*sp.sin(self.q2 - self.q3 + self.q4)
                - 5.79682785737267e-5*sp.sin(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                - 2.12929427959102e-10*sp.sin(self.q2 - self.q3 + self.q4)*sp.cos(self.q5)
                + 0.398214339527504*sp.sin(self.q2 - self.q3 + self.q4)
                + 0.0146842753117525*sp.cos(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                + 1.07487101378681*sp.cos(self.q2 - self.q3 + self.q4),

                # tau5
                + 2.02395162883661e-19*sp.sin(self.q5)
                - 2.89840328221494e-5*sp.sin(self.q2 - self.q3 + self.q4 - self.q5)
                - 2.89842457515773e-5*sp.sin(self.q2 - self.q3 + self.q4 + self.q5)
                - 5.1269873225073e-17*sp.cos(self.q5)
                - 0.00734211068669876*sp.cos(self.q2 - self.q3 + self.q4 - self.q5)
                + 0.00734216462505377*sp.cos(self.q2 - self.q3 + self.q4 + self.q5)
                - 5.0598976580993e-20*sp.cos(self.q2 + self.q3 - self.q4 - self.q5)
                - 5.05986048615204e-20*sp.cos(self.q2 + self.q3 - self.q4 + self.q5)
            ]
            self.teach_gripper_status = True
        else:
            # 无夹爪，测试通过
            self.tau = [
                sp.S(0),
                + 0.863718101066337*sp.sin(self.q2 - self.q3)
                + 0.253652183280507*sp.sin(self.q2 - self.q3 + self.q4)
                - 6.44883036135267*sp.cos(self.q2) + 0.00824244017278482*sp.cos(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                + 3.00315462962467*sp.cos(self.q2 - self.q3) + 0.648271075258975*sp.cos(self.q2 - self.q3 + self.q4),

                -3.00315462962467*sp.sin(self.q2)*sp.sin(self.q3) 
                - 0.863718101066337*sp.sin(self.q2 - self.q3)
                - 0.253652183280507*sp.sin(self.q2 - self.q3 + self.q4)
                - 3.00315462962467*sp.cos(self.q2)*sp.cos(self.q3) - 0.00824244017278482*sp.cos(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                - 0.648271075258975*sp.cos(self.q2 - self.q3 + self.q4),

                + 0.253652183280507*sp.sin(self.q2 - self.q3 + self.q4)
                + 0.00824244017278482*sp.cos(self.q5)*sp.cos(self.q2 - self.q3 + self.q4) + 0.648271075258975*sp.cos(self.q2 - self.q3 + self.q4),

                - 0.00412120494830576*sp.cos(self.q2 - self.q3 + self.q4 - self.q5) + 0.00412123522447906*sp.cos(self.q2 - self.q3 + self.q4 + self.q5)
            
            ]
            self.gripper_status = False
            self.teach_gripper_status = False
    
    def get_tau_numeric(self, joint_values_list):
        """
        计算给定关节角度下的力矩值
        参数:
        joint_values_list: 列表，包含5个关节角度的弧度值 [q1, q2, q3, q4, q5]
        返回:
        tau_numeric: 列表，包含各关节的力矩值
        """
        # 将列表转换为字典
        joint_values_rad = {
            self.q1: joint_values_list[0],
            self.q2: joint_values_list[1],
            self.q3: joint_values_list[2],
            self.q4: joint_values_list[3],
            self.q5: joint_values_list[4]
        }
        tau_numeric = [float(t.evalf(subs=joint_values_rad)) for t in self.tau]
        tau_numeric[1] = -tau_numeric[1]
        tau_numeric[2] = -tau_numeric[2]
        qpos = joint_values_list
        if self.gripper_status:

            link2 = mass_point(0.7,0.18,0.054)    
            link3 = mass_point(0.7,0.15,0.045)
            pos = rotate([link3.x,link3.y],-(qpos[3] + qpos[1] - qpos[2]))

            pos2 = rotate([0.22,0.06],qpos[2] - qpos[1])
            # print(pos2[0]*100,pos2[1]*100)
            link3.x = pos[0] + pos2[0]
            link3.y = pos[1] + pos2[1]
            pos = rotate([link2.x,link2.y],qpos[2] - qpos[1])
            link2.x = pos[0]
            link2.y = pos[1]
            mix_point = mass_point(link2.mass+link3.mass,0,0)
            mix_point.x = (link2.mass*link2.x + link3.mass*link3.x)/(link2.mass+link3.mass)
            mix_point.y = (link2.mass*link2.y + link3.mass*link3.y)/(link2.mass+link3.mass)
            tau_numeric[2] = mix_point.x*mix_point.mass*14
            link1 = mass_point(0.85,-0.18,0.0)
            # print(mix_point.x,mix_point.y)
            pos = rotate([-0.3,0],-qpos[1])
            # print(pos[0],pos[1])
            mix_point.x = mix_point.x + pos[0]
            mix_point.y = mix_point.y + pos[1]
            pos_1 = rotate([link1.x,link1.y],-qpos[1])
            link1.x = pos_1[0]
            link1.y = pos_1[1]
            # print(link1.x,link1.y)
            mix_point_1 = mass_point(mix_point.mass + link1.mass,0,0)
            mix_point_1.x = (mix_point.mass*mix_point.x + link1.mass*link1.x)/(mix_point.mass+link1.mass)
            mix_point_1.y = (mix_point.mass*mix_point.y + link1.mass*link1.y)/(mix_point.mass+link1.mass)
            tau_numeric[1] = -mix_point_1.x * mix_point_1.mass * 14.5
            
            tau_numeric[3] = -(1.15+0.6*math.cos(qpos[4]))* math.cos(qpos[3] + qpos[1] - qpos[2]- 0.16508) 
            tau_numeric[4] = (( math.sin(qpos[3] + qpos[1] - qpos[2])) * 0.6 *(math.sin(qpos[4])) ) 
            # return tau_numeric + [0.0] + [0.0]  
            # tau_numeric[3] = -(1.1+0.5*math.cos(joint_values_list[4]))* math.cos(joint_values_list[3] + joint_values_list[1] - joint_values_list[2]- math.radians(13))
            # tau_numeric[4] = (( math.sin(joint_values_list[3] + joint_values_list[1] - joint_values_list[2])) * 0.6 *(math.sin(joint_values_list[4])) )
            return tau_numeric + [0.0] + [0.0]
        elif self.teach_gripper_status:
            link2 = mass_point(0.75,0.20,0.055)    
            link3 = mass_point(0.80,0.14,0.03)
            pos = rotate([link3.x,link3.y],-(qpos[3] + qpos[1] - qpos[2]))

            pos2 = rotate([0.22,0.06],qpos[2] - qpos[1])
            # print(pos2[0]*100,pos2[1]*100)
            link3.x = pos[0] + pos2[0]
            link3.y = pos[1] + pos2[1]
            pos = rotate([link2.x,link2.y],qpos[2] - qpos[1])
            link2.x = pos[0]
            link2.y = pos[1]
            mix_point = mass_point(link2.mass+link3.mass,0,0)
            mix_point.x = (link2.mass*link2.x + link3.mass*link3.x)/(link2.mass+link3.mass)
            mix_point.y = (link2.mass*link2.y + link3.mass*link3.y)/(link2.mass+link3.mass)
            tau_numeric[2] = mix_point.x*mix_point.mass*12.5
            link1 = mass_point(0.85,-0.18,0.0)
            # print(mix_point.x,mix_point.y)
            pos = rotate([-0.3,0],-qpos[1])
            # print(pos[0],pos[1])
            mix_point.x = mix_point.x + pos[0]
            mix_point.y = mix_point.y + pos[1]
            pos_1 = rotate([link1.x,link1.y],-qpos[1])
            link1.x = pos_1[0]
            link1.y = pos_1[1]
            # print(link1.x,link1.y)
            mix_point_1 = mass_point(mix_point.mass + link1.mass,0,0)
            mix_point_1.x = (mix_point.mass*mix_point.x + link1.mass*link1.x)/(mix_point.mass+link1.mass)
            mix_point_1.y = (mix_point.mass*mix_point.y + link1.mass*link1.y)/(mix_point.mass+link1.mass)
            tau_numeric[1] = -mix_point_1.x * mix_point_1.mass * 13.5
            
            tau_numeric[3] = -(1.15+0.6*math.cos(qpos[4]))* math.cos(qpos[3] + qpos[1] - qpos[2]- math.radians(8)) 
            tau_numeric[4] = (( math.sin(qpos[3] + qpos[1] - qpos[2])) * 0.6 *(math.sin(qpos[4])) ) 
            # return tau_numeric + [0.0] + [0.0]  
            # tau_numeric[3] = -(1.1+0.5*math.cos(joint_values_list[4]))* math.cos(joint_values_list[3] + joint_values_list[1] - joint_values_list[2]- math.radians(13))
            # tau_numeric[4] = (( math.sin(joint_values_list[3] + joint_values_list[1] - joint_values_list[2])) * 0.6 *(math.sin(joint_values_list[4])) )
            return tau_numeric + [0.0] + [0.0]
        else:
            qpos = joint_values_list
            tau_numeric[3] = -0.8 * math.cos(qpos[3] + qpos[1] - qpos[2] - math.radians(18))
            return tau_numeric + [0.0]
    
class ArmGravityCalculatorPro:
    def __init__(self, gripper=False):
        # 定义符号变量
        self.q1, self.q2, self.q3, self.q4, self.q5 = sp.symbols('q1 q2 q3 q4 q5')
        self.q_syms = [self.q1, self.q2, self.q3, self.q4, self.q5]

        # 根据是否有夹爪选择参数组
        if gripper:
            # 带夹爪，测试通过
            self.tau = [
                sp.S(0),  # tau1

                # tau2
                -2.04463729169328e-16*sp.sin(self.q2)
                - 5.39383550138731e-8*sp.sin(self.q5)*sp.sin(self.q2 - self.q3 + self.q4)
                - 5.79682785737267e-5*sp.sin(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                + 0.78746089095303*sp.sin(self.q2 - self.q3)
                - 2.12929427959102e-10*sp.sin(self.q2 - self.q3 + self.q4)*sp.cos(self.q5)
                + 0.241341072298906*sp.sin(self.q2 - self.q3 + self.q4)
                - 5.68339811757021*sp.cos(self.q2)
                + 0.0146842753117525*sp.cos(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                + 2.91320920873086*sp.cos(self.q2 - self.q3)
                + 0.840919095749696*sp.cos(self.q2 - self.q3 + self.q4),

                # tau3
                -0.787460890953031*sp.sin(self.q2)*sp.cos(self.q3)
                + 0.78746089095303*sp.sin(self.q3)*sp.cos(self.q2)
                + 5.39383550138731e-8*sp.sin(self.q5)*sp.sin(self.q2 - self.q3 + self.q4)
                + 5.79682785737267e-5*sp.sin(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                + 2.12929427959102e-10*sp.sin(self.q2 - self.q3 + self.q4)*sp.cos(self.q5)
                - 0.241341072298906*sp.sin(self.q2 - self.q3 + self.q4)
                - 0.0146842753117525*sp.cos(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                - 2.91320920873086*sp.cos(self.q2 - self.q3)
                - 0.840919095749696*sp.cos(self.q2 - self.q3 + self.q4),

                # tau4
                - 5.39383550138731e-8*sp.sin(self.q5)*sp.sin(self.q2 - self.q3 + self.q4)
                - 5.79682785737267e-5*sp.sin(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                - 2.12929427959102e-10*sp.sin(self.q2 - self.q3 + self.q4)*sp.cos(self.q5)
                + 0.241341072298906*sp.sin(self.q2 - self.q3 + self.q4)
                + 0.0146842753117525*sp.cos(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                + 0.840919095749696*sp.cos(self.q2 - self.q3 + self.q4),

                # tau5
                2.02395162883661e-19*sp.sin(self.q5)
                - 2.89840328221494e-5*sp.sin(self.q2 - self.q3 + self.q4 - self.q5)
                - 2.89842457515773e-5*sp.sin(self.q2 - self.q3 + self.q4 + self.q5)
                - 5.1269873225073e-17*sp.cos(self.q5)
                - 0.00734211068669876*sp.cos(self.q2 - self.q3 + self.q4 - self.q5)
                + 0.00734216462505377*sp.cos(self.q2 - self.q3 + self.q4 + self.q5)
                - 5.0598976580993e-20*sp.cos(self.q2 + self.q3 - self.q4 - self.q5)
                - 5.05986048615204e-20*sp.cos(self.q2 + self.q3 - self.q4 + self.q5)
            ]
        else:
            # 不带夹爪，测试通过
            self.tau = [
                sp.S(0),  # tau1

                + 0.496692490953039*sp.sin(self.q2 - self.q3)
                + 0.197616862832389*sp.sin(self.q2 - self.q3 + self.q4)
                - 4.09219069167952*sp.cos(self.q2)
                + 0.00421629321093*sp.cos(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                + 1.79095965240075*sp.cos(self.q2 - self.q3)
                + 0.432823095749696*sp.cos(self.q2 - self.q3 + self.q4),  # tau2

                -0.496692490953039*sp.sin(self.q2)*sp.cos(self.q3)
                + 0.496692490953039*sp.sin(self.q3)*sp.cos(self.q2)
                - 0.197616862832389*sp.sin(self.q2 - self.q3 + self.q4)
                - 0.00421629321093*sp.cos(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                - 1.79095965240075*sp.cos(self.q2 - self.q3)
                - 0.432823095749696*sp.cos(self.q2 - self.q3 + self.q4),  # tau3

                + 0.197616862832389*sp.sin(self.q2 - self.q3 + self.q4)
                + 0.00421629321093*sp.cos(self.q5)*sp.cos(self.q2 - self.q3 + self.q4)
                + 0.432823095749696*sp.cos(self.q2 - self.q3 + self.q4),  # tau4

                - 0.00210813886181013*sp.cos(self.q2 - self.q3 + self.q4 - self.q5)
                + 0.00210815434911987*sp.cos(self.q2 - self.q3 + self.q4 + self.q5)  # tau5
            ]

    def get_tau_numeric(self, joint_values_list):
        """
        计算给定关节角度下的力矩值
        参数:
        joint_values_list: 列表，包含5个关节角度的弧度值 [q1, q2, q3, q4, q5]
        返回:
        tau_numeric: 列表，包含各关节的力矩值
        """
        # 将列表转换为字典
        joint_values_rad = {
            self.q1: joint_values_list[0],
            self.q2: joint_values_list[1],
            self.q3: joint_values_list[2],
            self.q4: joint_values_list[3],
            self.q5: joint_values_list[4]
        }
        tau_numeric = [float(t.evalf(subs=joint_values_rad)) for t in self.tau]

        tau_numeric[1] = -tau_numeric[1]
        tau_numeric[2] = -tau_numeric[2]
        tau_numeric[3] = -tau_numeric[3]
        return tau_numeric
