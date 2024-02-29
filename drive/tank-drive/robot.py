import wpilib
import wpilib.drive
import phoenix5

C_LEFT_BACK = 11
C_LEFT_FRONT = 22
C_RIGHT_FRONT = 33
C_RIGHT_BACK = 44


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.front_left = phoenix5.WPI_VictorSPX(11)
        self.rear_left = phoenix5.WPI_VictorSPX(22)
        self.left = wpilib.MotorControllerGroup(self.front_left, self.rear_left)

        self.front_right = phoenix5.WPI_VictorSPX(33)
        self.rear_right = phoenix5.WPI_VictorSPX(44)
        self.right = wpilib.MotorControllerGroup(self.front_right, self.rear_right)

        self.drivetrain = wpilib.drive.DifferentialDrive(self.left, self.right)
        self.drivetrain.setExpiration(0.1)

        self.stick = wpilib.Joystick(0)
        self.stick2 = wpilib.Joystick(1)

        self.right.setInverted(True)

    def teleopInit(self):
        self.drivetrain.setSafetyEnabled(True)

    def teleopPeriodic(self):
        self.drivetrain.tankDrive(
            self.stick.getRawAxis(1), self.stick2.getRawAxis(5), True
        )


if __name__ == "__main__":
    wpilib.run(Robot)
