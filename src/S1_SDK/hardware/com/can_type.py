from S1_SDK.hardware.com.base_com import ComStrategy,ReturnFrame
import can
import sys
class CanType(ComStrategy):
    def __init__(self,bus):
        if sys.platform.startswith('win'):
            self.bus = can.interface.Bus(interface='pcan', channel=bus, bitrate=1000000)    
        elif sys.platform.startswith('linux'):
            self.bus = can.interface.Bus(interface='socketcan', channel=bus, bitrate=1000000)
        elif sys.platform.startswith('darwin'):
            print("这里报错正常，还未对macos进行测试")
            self.bus = can.interface.Bus(interface='socketcan', channel=bus, bitrate=1000000)
    def send(self,id,data):
        tx_message = can.Message(
        arbitration_id=id,      
        is_extended_id=False,      
        is_remote_frame=False,     
        dlc=8,                     
        data=data
        )
        self.bus.send(tx_message)
    def recv(self,timeout=0.1):
        msg = self.bus.recv(timeout)
        if msg is None:
            return None
        return ReturnFrame(msg.arbitration_id,msg.data)
    def close(self):
        self.bus.shutdown()