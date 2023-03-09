import wpilib 
import wpilib.drive 
import ctre
import rev

MODULE_ID = 0

C_LEFT_BACK = 1
C_LEFT_FRONT = 2
C_RIGHT_FRONT = 3
C_RIGHT_BACK = 4
#Angle limit
LIMIT_ANGLE_FORWARD = 0.5
LIMIT_ANGLE_BACKWARD = 0.5
#Lenght limit
LIMIT_LENGHT_FORWARD = 4.97
LIMIT_LENGHT_BACKWARD = 0


class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        #tração
        self.m_left_back = ctre.WPI_VictorSPX(C_LEFT_BACK)
        self.m_left_front = ctre.WPI_VictorSPX(C_LEFT_FRONT)
        self.m_right_front = ctre.WPI_VictorSPX(C_RIGHT_FRONT)
        self.m_right_back = ctre.WPI_VictorSPX(C_RIGHT_BACK)
        self.m_left= wpilib.MotorControllerGroup(self.m_left_front,self.m_left_back)   
        self.m_right= wpilib.MotorControllerGroup(self.m_right_front,self.m_right_back) 
        self.myRobot  = wpilib.drive.DifferentialDrive(self.m_left, self.m_right) 

        #braço e intake
        self.m_arm_angle = rev.CANSparkMax(1, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.m_arm_lenght = rev.CANSparkMax(2,  rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        #reset encoder
        self.m_arm_angle_pid = self.m_arm_angle.getPIDController()
        self.m_arm_lenght_pid = self.m_arm_angle.getPIDController()
        self.m_arm_angle_pid.setReference(0,rev.CANSparkMax.ControlType.kPosition)
        self.m_arm_lenght_pid.setReference(0,rev.CANSparkMax.ControlType.kPosition)
        #set limit
        self.m_arm_angle.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, LIMIT_ANGLE_FORWARD)
        self.m_arm_angle.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, LIMIT_ANGLE_BACKWARD)
        self.m_arm_lenght.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, LIMIT_LENGHT_FORWARD)
        self.m_arm_lenght.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, LIMIT_LENGHT_BACKWARD)
      
        #pneumática
        self.p = wpilib.PneumaticsControlModule()
        self.compressor = wpilib.Compressor(MODULE_ID, wpilib.PneumaticsModuleType.CTREPCM)
        self.solenoid = wpilib.DoubleSolenoid(MODULE_ID, wpilib.PneumaticsModuleType.CTREPCM, 1, 2)
        
        #joystick
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
        