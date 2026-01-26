from S1_SDK.hardware.com.base_com import ComStrategy,ReturnFrame
import serial
class UartType(ComStrategy):
    def __init__(self, device: str):
        """
        初始化串口
        :param device: 串口设备路径，如 'COM3' (Windows) 或 '/dev/ttyUSB0' (Linux)
        :param baudrate: 波特率，默认 2000000
        :param timeout: 读取超时时间（秒）
        """
        baudrate = 1500000
        timeout = 0.1
        self.uart_buffer = []
        self.data_buffer = []
        try:
            self.bus = serial.Serial(
                port=device,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,          # 读超时
                write_timeout=timeout     # 写超时
            )
            # print(f"UART opened: {device} @ {baudrate} bps")
        except Exception as e:
            print(f"Failed to open UART {device}: {e}")
            self.bus = None

    def send(self,id, data) -> None:
        """通过串口发送字节数据"""
        if self.bus is None or not self.bus.is_open:
            raise RuntimeError("UART not initialized or closed")
        # print(id)
        frame =[0x55,0xAA]
        if id <0x0b and id>0:
            frame.append(id)
        elif id == 0xfe:
            frame.append(0xfe)
        elif id == 0x10E:
            frame.append(0xfc)
        elif id == 0x8E:
            frame.append(0xfb)
        elif 0xff <= id and id <= 0x10a:
            frame.append(0x20|(id&0x0f))
        else:
            frame.append(0xff)
        frame.extend(data)
        self.bus.write(frame)
        self.bus.flush()  # 确保数据立即发出
    def process(self,data):
        self.uart_buffer.extend(data)
        while len(self.uart_buffer) >= 12:  # 至少一帧才处理
            # print(self.uart_buffer)
            if self.uart_buffer[0] == 0xAA and self.uart_buffer[1] == 0x55 and self.uart_buffer[11] == 0x01:
                payload = self.uart_buffer[2:12]
                self.data_buffer.append(payload)
                del self.uart_buffer[:12]
            else:
                del self.uart_buffer[0]
    def recv(self, timeout: float = 0.1) -> bytes:
        original_timeout = self.bus.timeout
        self.bus.timeout = timeout

        try:
            data = self.bus.read_all()   
            # print(data)     
            if len(data) != 0:
                self.process(data)
            if len(self.data_buffer) == 0:
                return None
            frame = self.data_buffer.pop(0)
            return ReturnFrame(frame[0]&0x0f,frame[1:])
        finally:
            self.bus.timeout = original_timeout
    def close(self):
        """关闭串口"""
        if self.bus is not None and self.bus.is_open:
            self.bus.close()
            # print("UART closed")
