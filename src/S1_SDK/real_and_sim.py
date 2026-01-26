# 添加项目根目录到 Python 路径
from S1_SDK.base_mode import ControlStrategy
from typing import List


class RealAndSimStrategy(ControlStrategy):
    def joint_control(self, arm, pos: List[float]) -> bool:
        if pos is not None:
            arm.motor.control_Pos_Vel(pos)
            arm.sim.control(pos)
            return True
        return False
    
    def refresh(self, arm):
        arm.sim.refresh()
        arm.motor.refresh_motor_status()
    
    def needs_motor(self) -> bool:
        return True
    
    def needs_sim(self) -> bool:
        return True