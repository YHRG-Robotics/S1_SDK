from S1_SDK.base_mode import ControlStrategy
from typing import List


class RealControlSimStrategy(ControlStrategy):
    def joint_control(self, arm, pos: List[float]) -> bool:
        arm.sim.control(arm.get_pos())
        return True
    
    def refresh(self, arm):
        arm.sim.refresh()
        arm.motor.refresh_motor_status()
    
    def needs_motor(self) -> bool:
        return True
    
    def needs_sim(self) -> bool:
        return True
