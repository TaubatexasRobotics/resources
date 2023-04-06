import wpilib
import wpilib.drive
import ctre
import rev

MODULE_ID = 0


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):

        self.m_left_front = ctre.WPI_VictorSPX(2)
        self.m_right_front = ctre.WPI_VictorSPX(1)
        self.m_left_back = ctre.WPI_VictorSPX(4)
        self.m_right_back = ctre.WPI_VictorSPX(3)

        self.m_left = wpilib.MotorControllerGroup(self.m_left_front, self.m_left_back)
        self.m_right = wpilib.MotorControllerGroup(
            self.m_right_front, self.m_right_back
        )

        self.myRobot = wpilib.drive.DifferentialDrive(self.m_left, self.m_right)
        self.stick = wpilib.Joystick(0)

    def teleopInit(self):

        self.myRobot.setSafetyEnabled(True)

    def teleopPeriodic(self):

        self.myRobot.arcadeDrive(self.stick)


if __name__ == "__main__":
    wpilib.run(MyRobot)
