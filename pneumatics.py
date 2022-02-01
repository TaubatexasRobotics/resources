import wpilib
import wpilib.drive

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.compressor = wpilib.Compressor(wpilib.PneumaticsModuleType.CTREPCM)
        self.solenoid = wpilib.DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, 1, 2)
        self.stick = wpilib.Joystick(0)
    
    def teleopPeriodic(self):
        if self.stick.getRawButton(1) == True:
            self.compressor.start()
        else:
            self.compressor.stop()

		'''
        caso queria com toggle (se estiver off, acontece nada / se estiver desativado, ativa e vice-versa)
        if self.stick.getRawButton(2) == True:
            self.solenoid.toggle()
		'''
        if self.stick.getRawButton(2) == True:
            self.solenoid.set(wpilib.DoubleSolenoid.Value.kOff)
        if self.stick.getRawButton(3) == True:
            self.solenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        if self.stick.getRawButton(4) == True:
            self.solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
		
if __name__ == "__main__":
    wpilib.run(MyRobot)
