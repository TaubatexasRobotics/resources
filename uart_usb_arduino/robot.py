import wpilib

class TestArduino(wpilib.TimedRobot):
    def robotInit(self) -> None:  
        self.arduino = wpilib.SerialPort(9600, wpilib.SerialPort.Port.kUSB1)
        self.timer = wpilib.Timer()
        self.timer.start()

    def robotPeriodic(self) -> None:
        if self.timer.get() > 5:
            print('Wrote to Arduino')
            self.arduino.write(b'\x12')
            self.timer.reset()
        
        if self.arduino.getBytesReceived() > 0:
            print(self.readString(self.arduino))
    
    def readString(self, port) -> str:
        sz = port.getBytesReceived()
        buf = bytearray(sz)
        sz = port.read(buf)
        return buf[:sz].decode("ascii")

if __name__ == "__main__":
    wpilib.run(TestArduino)