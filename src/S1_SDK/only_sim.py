from S1_SDK.base_mode import ControlStrategy
from typing import List


class OnlySimStrategy(ControlStrategy):
    def joint_control(self, arm, pos: List[float]) -> bool:
        if pos is not None:
            arm.sim.control(pos)
            return True
        return False
    
    def refresh(self, arm):
        arm.sim.refresh()
    
    def needs_motor(self) -> bool:
        return False
    
    def needs_sim(self) -> bool:
        return True
