# hardware/__init__.py
from .base_mode import ControlStrategy
from .only_sim import OnlySimStrategy
from .only_real import OnlyRealStrategy
from .real_control_sim import RealControlSimStrategy
# from .smotor_master import SmotorMaster
from .S1_arm import  S1_arm,control_mode  # 假设 motor.py 中定义了这些类
__all__ = ['control_mode', 'S1_arm']
