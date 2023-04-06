import wpilib
import wpilib.drive

MODULE_ID = 0


class Pneumatics(wpilib.TimedRobot):
    def robotInit(self):
        self.compressor = wpilib.Compressor(
            MODULE_ID, wpilib.PneumaticsModuleType.CTREPCM
        )
        self.solenoid = wpilib.DoubleSolenoid(
            MODULE_ID, wpilib.PneumaticsModuleType.CTREPCM, 1, 2
        )
        self.stick = wpilib.Joystick(0)

    def teleopInit(self):
        self.compressor.stop()

    def teleopPeriodic(self):
        if self.stick.getRawButton(1) == True:
            self.compressor.start()
        else:
            self.compressor.stop()

        # Caso queria com toggle (se estiver off, acontece nada / se estiver desativado, ativa e vice-versa)
        # if self.stick.getRawButton(2) == True:
        # 	self.solenoid.toggle()

        if self.stick.getRawButton(2) == True:
            self.solenoid.set(wpilib.DoubleSolenoid.Value.kOff)
        if self.stick.getRawButton(3) == True:
            self.solenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        if self.stick.getRawButton(4) == True:
            self.solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)


if __name__ == "__main__":
    wpilib.run(Pneumatics)
