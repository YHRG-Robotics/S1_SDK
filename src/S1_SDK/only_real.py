from S1_SDK.base_mode import ControlStrategy
from typing import List


class OnlyRealStrategy(ControlStrategy):
    def joint_control(self, arm, pos: List[float]) -> bool:
        if pos is not None:
            arm.motor.control_Pos_Vel(pos)
            return True
        return False
    
    def refresh(self, arm):
        arm.motor.refresh_motor_status()
    
    def needs_motor(self) -> bool:
        return True
    
    def needs_sim(self) -> bool:
        return False