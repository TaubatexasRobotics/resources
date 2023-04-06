import wpilib 
import wpilib.drive 
import ctre
import rev

MODULE_ID = 0

C_LEFT_BACK = 1
C_LEFT_FRONT = 2
C_RIGHT_FRONT = 3
C_RIGHT_BACK = 4

ARM_LENGHT_SPARK_ID = 50
ARM_ANGLE_SPARK_ID = 51

#Angle limit
LIMIT_ANGLE_FORWARD = 0
LIMIT_ANGLE_BACKWARD = 21.7
#Lenght limit
LIMIT_LENGHT_FORWARD = 4.97
LIMIT_LENGHT_BACKWARD = 0

ARM_ANGLE= {
    "KP" : 0.1,
    "KI" : 0.0,
    "KD" : 0.1,   
}

ARM_LENGHT= {
    "KP" : 0.1,
    "KI" : 0.0,
    "KD" : 0.1,   
}

SOLENOID_FORWARD_CHANNEL = 0
SOLENOID_REVERSE_CHANNEL = 1

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        #tração
        self.m_left_back = ctre.WPI_VictorSPX(C_LEFT_BACK)
        self.m_left_front = ctre.WPI_VictorSPX(C_LEFT_FRONT)
        self.m_right_front = ctre.WPI_VictorSPX(C_RIGHT_FRONT)
        self.m_right_back = ctre.WPI_VictorSPX(C_RIGHT_BACK)
        self.m_left= wpilib.MotorControllerGroup(self.m_left_front,self.m_left_back) 
        self.m_left.setInverted(True) 
        self.m_right= wpilib.MotorControllerGroup(self.m_right_front,self.m_right_back) 
        self.myRobot  = wpilib.drive.DifferentialDrive(self.m_left, self.m_right) 

        #braço e intake
        self.m_arm_angle = rev.CANSparkMax(ARM_ANGLE_SPARK_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.m_arm_lenght = rev.CANSparkMax(ARM_LENGHT_SPARK_ID,  rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        self.m_arm_angle_encoder = self.m_arm_angle.getEncoder()
        self.m_arm_lenght_encoder = self.m_arm_lenght.getEncoder()

        #set pid constants
        self.m_arm_angle_pid = self.m_arm_angle.getPIDController()
        self.m_arm_angle_pid.setP(ARM_ANGLE["KP"])
        self.m_arm_angle_pid.setI(ARM_ANGLE["KI"])
        self.m_arm_angle_pid.setD(ARM_ANGLE["KD"])

        self.m_arm_lenght_pid = self.m_arm_lenght.getPIDController()
        self.m_arm_lenght_pid.setP(ARM_LENGHT["KP"])
        self.m_arm_lenght_pid.setI(ARM_LENGHT["KI"])
        self.m_arm_lenght_pid.setD(ARM_LENGHT["KD"])

        #set limit
        # # self.m_arm_angle.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, LIMIT_ANGLE_FORWARD)
        # self.m_arm_angle.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, LIMIT_ANGLE_BACKWARD)
        # self.m_arm_lenght.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, LIMIT_LENGHT_FORWARD)
        # self.m_arm_lenght.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, LIMIT_LENGHT_BACKWARD)
      
        #pneumática
        self.p = wpilib.PneumaticsControlModule()
        self.compressor = wpilib.Compressor(MODULE_ID, wpilib.PneumaticsModuleType.CTREPCM)
        self.solenoid = wpilib.DoubleSolenoid(MODULE_ID, wpilib.PneumaticsModuleType.CTREPCM, SOLENOID_FORWARD_CHANNEL, SOLENOID_REVERSE_CHANNEL)
        
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
            
        if self.stick.getRawButton(3) == True:
            self.solenoid.set(wpilib.DoubleSolenoid.Value.kForward)
                    
        if self.stick.getRawButton(4) == True:
            self.solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

        if self.stick.getRawButtonPressed(5) == True:
            angle_position = self.m_arm_angle_encoder.getPosition()
            print(angle_position)
            self.m_arm_angle_pid.setReference(angle_position+1,rev.CANSparkMax.ControlType.kPosition)
        
        if self.stick.getRawButtonPressed(6) == True:
            angle_position = self.m_arm_angle_encoder.getPosition()
            self.m_arm_angle_pid.setReference(angle_position-1,rev.CANSparkMax.ControlType.kPosition)

        if self.stick.getRawButtonPressed(7) == True:
            lenght_position = self.m_arm_lenght_encoder.getPosition()
            self.m_arm_lenght_pid.setReference(lenght_position+0.5,rev.CANSparkMax.ControlType.kPosition)

        if self.stick.getRawButtonPressed(8) == True:
            lenght_position = self.m_arm_lenght_encoder.getPosition()
            self.m_arm_lenght_pid.setReference(lenght_position-0.5,rev.CANSparkMax.ControlType.kPosition)

            
if __name__ == "__main__":
    wpilib.run(MyRobot)
        
