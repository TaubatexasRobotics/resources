import wpilib 
import wpilib.drive 
import ctre
import rev
import constants

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        #tração
        self.m_left_back = ctre.WPI_VictorSPX(constants.C_LEFT_BACK)
        self.m_left_front = ctre.WPI_VictorSPX(constants.C_LEFT_FRONT)
        self.m_right_front = ctre.WPI_VictorSPX(constants.C_RIGHT_FRONT)
        self.m_right_back = ctre.WPI_VictorSPX(constants.C_RIGHT_BACK)
        self.m_left= wpilib.MotorControllerGroup(self.m_left_front,self.m_left_back) 
        self.m_left.setInverted(True) 
        self.m_right= wpilib.MotorControllerGroup(self.m_right_front,self.m_right_back) 
        self.myRobot  = wpilib.drive.DifferentialDrive(self.m_left, self.m_right) 

        #braço e intake
        self.m_arm_angle = rev.CANSparkMax(constants.ARM_ANGLE_SPARK_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.m_arm_lenght = rev.CANSparkMax(constants.ARM_LENGHT_SPARK_ID,  rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        self.m_arm_angle_encoder = self.m_arm_angle.getEncoder()
        self.m_arm_lenght_encoder = self.m_arm_lenght.getEncoder()

        #pneumática
        self.p = wpilib.PneumaticsControlModule()
        self.compressor = wpilib.Compressor(constants.MODULE_ID, wpilib.PneumaticsModuleType.CTREPCM)
        self.solenoid = wpilib.DoubleSolenoid(constants.MODULE_ID, wpilib.PneumaticsModuleType.CTREPCM, constants.SOLENOID_FORWARD_CHANNEL, constants.SOLENOID_REVERSE_CHANNEL)
        
        #joystick
        self.stick = wpilib.Joystick(0)

        #variáveis de estado
        self.solenoidActived = False

    def teleopInit(self):

        self.myRobot.setSafetyEnabled(True)
        self.compressor.disable()

    def teleopPeriodic(self):

        self.myRobot.arcadeDrive(
            self.stick.getRawAxis(constants.XBOX_L_ANALOGICO_Y),
            self.stick.getRawAxis(constants.XBOX_L_ANALOGICO_X)*1.15,
            True
        )
        self.m_arm_angle.set(self.stick.getRawAxis(constants.XBOX_R_ANALOGICO_X))


        if self.stick.getRawButton(constants.XBOX_START_BUTTON) == True:
            self.compressor.enableDigital()
        else:
            self.compressor.disable()

        if self.stick.getRawButtonPressed(constants.XBOX_A_BUTTON) == True:
            if (self.solenoidActived):
                self.solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
            else :
                self.solenoid.set(wpilib.DoubleSolenoid.Value.kForward)
            self.solenoidActived = not(self.solenoidActived)
            print("EAI")
            
        if self.stick.getRawButton(constants.XBOX_RB) == True:
            self.m_arm_lenght.set(0.1)
        elif self.stick.getRawButton(constants.XBOX_LB) == True:
            self.m_arm_lenght.set(-0.1)
        else:
            self.m_arm_lenght.set(0)
            #trocar por permanecer no lugar
        
if __name__ == "__main__":
    wpilib.run(MyRobot)