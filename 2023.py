import wpilib 
import wpilib.drive 
import ctre
import rev

MODULE_ID = 0

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):

        
        self.m_left_front = rev.CANSparkMax(1, rev.CANSparkMaxLowLevel.MotorType.kBrushless) 
        self.m_left_back = rev.CANSparkMax(2,  rev.CANSparkMaxLowLevel.MotorType.kBrushless) 
        self.m_right_front = rev.CANSparkMax(3, rev.CANSparkMaxLowLevel.MotorType.kBrushless) 
        self.m_right_back = rev.CANSparkMax(4,  rev.CANSparkMaxLowLevel.MotorType.kBrushless)  

        self.m_left= wpilib.MotorControllerGroup(self.m_left_front,self.m_left_back)   
        self.m_right= wpilib.MotorControllerGroup(self.m_right_front,self.m_right_back) 

        self.p = wpilib.PneumaticsControlModule()
        self.myRobot  = wpilib.drive.DifferentialDrive(self.m_left, self.m_right) 

        self.compressor = wpilib.Compressor(MODULE_ID, wpilib.PneumaticsModuleType.CTREPCM)
        self.solenoid = wpilib.DoubleSolenoid(MODULE_ID, wpilib.PneumaticsModuleType.CTREPCM, 1, 2)
        self.stick = wpilib.Joystick(0)


    def teleopInit(self):

        self.myRobot.setSafetyEnabled(True)
        self.compressor.disable()

    def teleopPeriodic(self):

        self.myRobot.arcadeDrive(
            self.stick.getRawAxis(1),
            self.stick.getRawAxis(0)*1.15,
            True
        )

        if self.stick.getRawButton(1) == True:
            self.compressor.enableDigital()
        else:
            self.compressor.disable()

        if self.stick.getRawButton(2) == True:
            self.solenoid.set(wpilib.DoubleSolenoid.Value.kOff)
            #print("kOff")

        if self.stick.getRawButton(3) == True:
            self.solenoid.set(wpilib.DoubleSolenoid.Value.kForward)
            #print("kForward")
        
        if self.stick.getRawButton(4) == True:
            self.solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

            #print("reverteu")
            
if __name__ == "__main__":
    wpilib.run(MyRobot)
        