from abc import ABC, abstractmethod
from typing import List
from dataclasses import dataclass

@dataclass
class ReturnFrame:
    """通用接口"""
    id: int
    data: List[int] = None
class ComStrategy(ABC):
    """控制策略抽象基类"""
    @abstractmethod
    def send(self):
        pass
    @abstractmethod
    def recv(self):
        pass
    @abstractmethod
    def close(self):
        pass
   