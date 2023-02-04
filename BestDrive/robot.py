import wpilib
import wpilib.drive
import ctre

C_LEFT_BACK = 11
C_LEFT_FRONT = 22
C_RIGHT_FRONT = 33
C_RIGHT_BACK = 44


class Drivetrain(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""

        # motor controllers for drivetrain
        self.m_left_back = ctre.WPI_VictorSPX(C_LEFT_BACK)
        self.m_left_front = ctre.WPI_VictorSPX(C_LEFT_FRONT)
        self.m_right_front = ctre.WPI_VictorSPX(C_RIGHT_FRONT)
        self.m_right_back = ctre.WPI_VictorSPX(C_RIGHT_BACK)

        self.m_left = wpilib.MotorControllerGroup(self.m_left_front, self.m_left_back)
        self.m_right = wpilib.MotorControllerGroup(self.m_right_front, self.m_right_back)

        # object that handles basic drive operations
        self.drivetrain = wpilib.drive.DifferentialDrive(self.m_left, self.m_right)
        self.drivetrain.setExpiration(0.1)

        # joystick 0
        self.stick = wpilib.Joystick(0)
        
        self.m_right.setInverted(True)


        #achei um video: https://www.youtube.com/watch?v=dQw4w9WgXcQ

    def teleopInit(self):
        self.drivetrain.setSafetyEnabled(True)

    def teleopPeriodic(self):
        self.l_trigger_value= self.stick.getRawAxis(2)
        self.r_trigger_value = self.stick.getRawAxis(3)
        
        self.combined_triggered_value = self.r_trigger_value - self.l_trigger_value

        self.drivetrain.arcadeDrive(
            self.combined_triggered_value, self.stick.getRawAxis(0), True        )

if __name__ == "__main__":
    wpilib.run(Drivetrain)