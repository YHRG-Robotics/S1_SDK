from urdfpy import URDF
import PyKDL
import numpy as np
import os
import sys
root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_dir)
from S1_SDK.hardware.mujoco_sim import Mujoco
import tempfile

np.float = float  # 修复 numpy float 弃用

class KDLRobotSolver:
    def __init__(self, base_link="base_link", ee_link="6_Link"):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # URDF 文件路径
        urdf_path = os.path.join(script_dir, "../resource/meshes/DM13urdf.urdf")

        # Meshes 目录
        mesh_dir = os.path.join(script_dir, "../resource/meshes")

        # 读取 URDF
        with open(urdf_path, "r", encoding='utf-8') as f:
            content = f.read()

        # 替换 package:// 路径为真实绝对路径
        content = content.replace(
            "package://DM13urdf/meshes/",
            mesh_dir + "/"
        )

        # 写入临时文件，再用 URDF.load 加载
        
        tmp_file = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
        tmp_file.write(content)
        tmp_file.close()
        self.robot = URDF.load(tmp_file.name)

        self.base_link = base_link
        self.ee_link = ee_link

        # 构建 KDL Chain
        self.kdl_chain = self._urdf_to_kdl_chain()
        self.n_joints = self.kdl_chain.getNrOfJoints()
        self.joint_limits_min = PyKDL.JntArray(self.n_joints)
        self.joint_limits_max = PyKDL.JntArray(self.n_joints)

        # 示例：从 URDF 或手动设置
        # 注意：顺序必须与 KDL Chain 中的活动关节一致！
        # print(f"KDL Chain 构建完成, 关节数: {self.n_joints}")
        # self._print_chain_info()

        # 初始化 FK 和 IK 求解器
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kdl_chain)
        self.ik_solver = PyKDL.ChainIkSolverPos_LMA(self.kdl_chain)

    def _urdf_to_kdl_chain(self):
        chain = PyKDL.Chain()
        processed_links = set()

        def add_children(parent_name):
            if parent_name in processed_links:
                return
            processed_links.add(parent_name)

            for joint in self.robot.joints:
                if joint.parent != parent_name:
                    continue

                child_name = joint.child

                # origin
                if joint.origin is not None:
                    T = joint.origin
                    xyz = T[:3, 3]
                    position = PyKDL.Vector(*xyz)
                    R_mat = T[:3, :3]
                    rotation = PyKDL.Rotation(
                        R_mat[0,0], R_mat[0,1], R_mat[0,2],
                        R_mat[1,0], R_mat[1,1], R_mat[1,2],
                        R_mat[2,0], R_mat[2,1], R_mat[2,2]
                    )
                else:
                    position = PyKDL.Vector(0,0,0)
                    rotation = PyKDL.Rotation.Identity()

                # joint
                if joint.joint_type == 'fixed':
                    kdl_joint = PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)
                elif joint.joint_type == 'revolute':
                    axis_local = PyKDL.Vector(*joint.axis)
                    axis_global = rotation * axis_local
                    kdl_joint = PyKDL.Joint(joint.name, position, axis_global, PyKDL.Joint.RotAxis)
                else:
                    continue

                frame = PyKDL.Frame(rotation, position)
                segment = PyKDL.Segment(joint.name, kdl_joint, frame)
                chain.addSegment(segment)

                add_children(child_name)

        add_children(self.base_link)
        return chain

    def _print_chain_info(self):
        print("\n=== KDL Chain 详细信息 ===")
        for i in range(self.kdl_chain.getNrOfSegments()):
            seg = self.kdl_chain.getSegment(i)
            joint = seg.getJoint()
            print(f"Segment {i}: {seg.getName()}")
            print(f"  Joint: {joint.getName()}, Type: {joint.getTypeName()}")
            if joint.getType() != PyKDL.Joint.Fixed:
                axis = joint.JointAxis()
                print(f"  Joint Axis: ({axis.x():.3f}, {axis.y():.3f}, {axis.z():.3f})")

    def fk_quat(self, joint_values):
        """正运动学：输入关节角 -> 输出 (x,y,z,qx,qy,qz,qw)"""
        q = PyKDL.JntArray(self.n_joints)
        for i in range(min(self.n_joints, len(joint_values))):
            q[i] = joint_values[i]

        end_frame = PyKDL.Frame()
        self.fk_solver.JntToCart(q, end_frame)  # ✅ 调用 FK solver，而不是 self.fk()

        # 转 xyz + 四元数
        x, y, z = end_frame.p[0], end_frame.p[1], end_frame.p[2]
        qx, qy, qz, qw = end_frame.M.GetQuaternion()
        return (x, y, z, qx, qy, qz, qw)
    

    def fk_euler(self, joint_values):
        """
        正运动学：输入关节角 -> 输出 (x,y,z,r,p,y)
        """
        q = PyKDL.JntArray(self.n_joints)
        for i in range(min(self.n_joints, len(joint_values))):
            q[i] = joint_values[i]

        frame = PyKDL.Frame()
        self.fk_solver.JntToCart(q, frame)

        # 位置
        x, y, z = frame.p[0], frame.p[1], frame.p[2]

        # 欧拉角
        roll, pitch, yaw = frame.M.GetRPY()

        return (x, y, z, roll-1.57, pitch, yaw+1.57)

    def ik_quat(self, target_pose, q_init=None):
        """
        逆运动学：
        输入: 目标末端位姿 (x, y, z, qx, qy, qz, qw)
        输出: 关节角列表
        """
        if q_init is None:
            q_init = [0.0] * self.n_joints

        # 拆分位姿
        x, y, z, qx, qy, qz, qw = target_pose
        rot = PyKDL.Rotation.Quaternion(qx, qy, qz, qw)
        pos = PyKDL.Vector(x, y, z)
        target_frame = PyKDL.Frame(rot, pos)

        # 初始关节角
        q_init_arr = PyKDL.JntArray(self.n_joints)
        for i in range(self.n_joints):
            q_init_arr[i] = q_init[i]

        # 输出结果
        q_out = PyKDL.JntArray(self.n_joints)
        ret = self.ik_solver.CartToJnt(q_init_arr, target_frame, q_out)

        if ret >= 0:
            return [q_out[i] for i in range(self.n_joints)]
        else:
            print("IK 求解失败")
            return None

    def ik_euler(self, target_pose_rpy,q_init=None):
        """
        逆运动学（欧拉角版本）
        输入:
            target_pose_rpy = (x, y, z, roll, pitch, yaw)
        输出:
            关节角列表
        """
        if q_init is None:
            q_init = [0.0] * self.n_joints
        # q_init[3] = 1.57
        # 拆分位置 + RPY
        x, y, z, roll, pitch, yaw = target_pose_rpy

        # 欧拉角 → Rotation
        rot = PyKDL.Rotation.RPY(roll, pitch, yaw)
        rot_temp = PyKDL.Rotation.RPY(1.57, 0,-1.57)

        # Rx = np.array([[ 1,  0,  0],
        #             [ 0,  0, -1],
        #             [ 0,  1,  0]])

        # print(rot)
        R_transfor = rot_temp* rot
        pos = PyKDL.Vector(x, y, z)
        target_frame = PyKDL.Frame(R_transfor, pos)

        # 初始关节角
        q_init_arr = PyKDL.JntArray(self.n_joints)
        for i in range(len(q_init)):
            q_init_arr[i] = q_init[i]

        # 输出
        q_out = PyKDL.JntArray(self.n_joints)
        ret = self.ik_solver.CartToJnt(q_init_arr, target_frame, q_out)

        if ret >= 0:
            return [q_out[i] for i in range(self.n_joints)]
        else:
            # print("IK 求解失败")
            return None

def main():
    urdf_path = "/home/zheke/workspace/R5/meshes/DM9_URDF.urdf"
    mujoco_xml = "/home/zheke/workspace/R5/test_mesh/mjmodel.xml"

    solver = KDLRobotSolver()

    # 测试关节角
    # joint_values = [-2.64, 0.884, 1.04, -0.136, -1.22, -0.863]
    # joint_values = [0] * 6
    joint_values = [0,0,0,0,0,0]

    # === FK ===
    pose = solver.fk_quat(joint_values)  # (x,y,z,qx,qy,qz,qw)
    pose_euler = solver.fk_euler(joint_values)
    print("\n=== FK ===")
    print("输入关节角:", joint_values)
    print("末端位姿: x={:.3f}, y={:.3f}, z={:.3f}, q=({:.3f}, {:.3f}, {:.3f}, {:.3f})".format(
        pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]))
    
    print("\n=== FK (Euler) ===")
    print("xyz = ({:.3f}, {:.3f}, {:.3f})".format(pose_euler[0], pose_euler[1], pose_euler[2]))
    print("rpy = ({:.3f}, {:.3f}, {:.3f})".format(pose_euler[3], pose_euler[4], pose_euler[5]))

    # === IK ===
    ik_test = [0,0,0.45,0, 0, 0, 1]
    ik_solution = solver.ik_quat(ik_test)  # 输入 xyz+quat
    ik_solution_euler = solver.ik_euler(pose_euler)  # 输入 xyz+rpy
    
    print("\n=== IK ===")
    if ik_solution is not None:
        print("IK 求解成功")
        print("IK 解:", ik_solution)
        print("\n=== IK (Euler) ===")
        if ik_solution_euler is not None:
            print("IK 求解成功")
            print("IK 解:", ik_solution_euler)
        else:
            print("IK 求解失败")

        # 用 IK 结果再次 FK 验证
        ik_test = [0,0,0.45,0,0,0,1]
        pose2 = solver.fk_quat(ik_solution)
        print("\n=== FK (使用 IK 结果) ===")
        print("末端位姿: x={:.3f}, y={:.3f}, z={:.3f}, q=({:.3f}, {:.3f}, {:.3f}, {:.3f})".format(
            pose2[0], pose2[1], pose2[2], pose2[3], pose2[4], pose2[5], pose2[6]))

        # 如果要在 Mujoco 里看运动
        # mujoco_sim = Mujoco(mujoco_xml)
        # while True:
        #     mujoco_sim.control(ik_solution)
        #     mujoco_sim.refresh()
    else:
        print("IK 求解失败")


if __name__ == "__main__":
    main()

