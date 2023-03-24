import wpilib
import ctre

class Luluzinha(wpilib.TimedRobot):
    def robotInit(self):
        self.switch=wpilib.DigitalInput(0)
        self.motor=ctre.WPI_VictorSPX(1)
    def batatinha(self,speed):
        if speed>0:
            if self.switch.get():
                self.motor.set(0)
        else:
