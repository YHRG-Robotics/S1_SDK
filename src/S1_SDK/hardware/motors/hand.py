from S1_SDK.hardware.com.can_type import CanType
bus = CanType("PCAN_USBBUS2")
class Hand:
    def __init__(self,hand_type:str):
        self.type = hand_type
        if self.type == "left":
            self.id = 0x8E
        elif self.type == "right":
            self.id = 0x10E
    def position_control(self,position:list):
        data = [0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        data[2] = position[0]
        data[3] = position[1]
        data[4] = position[2]
        data[5] = position[3]
        data[6] = position[4]
        data[7] = position[5]
        bus.send(self.id,data)
