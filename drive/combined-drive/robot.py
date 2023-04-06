import wpilib
import wpilib.drive
import ctre


C_LEFT_BACK = 11
C_LEFT_FRONT = 22
C_RIGHT_FRONT = 33
C_RIGHT_BACK = 44

xbox = {
    "a": 1,
    "b": 2,
    "x": 3,
    "y": 4,
    "lb": 5,
    "rb": 6,
    "select": 7,
    "start": 8,
    "press_left_stick": 9,
    "press_right_stick": 10,
    "pov-up": 0,
    "pov-down": 180,
    "pov-left": 270,
    "pov-right": 90,
    "left-x-stick": 0,
    "left-y-stick": 1,
    "lt": 2,
    "rt": 3,
    "right-x-stick": 4,
    "right-y-stick": 5,
}


class Drivetrain(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""

        # motor controllers for drivetrain
        self.m_left_back = ctre.WPI_VictorSPX(C_LEFT_BACK)
        self.m_left_front = ctre.WPI_VictorSPX(C_LEFT_FRONT)
        self.m_right_front = ctre.WPI_VictorSPX(C_RIGHT_FRONT)
        self.m_right_back = ctre.WPI_VictorSPX(C_RIGHT_BACK)

        self.m_left = wpilib.MotorControllerGroup(self.m_left_front, self.m_left_back)
        self.m_right = wpilib.MotorControllerGroup(
            self.m_right_front, self.m_right_back
        )

        # object that handles basic drive operations
        self.drivetrain = wpilib.drive.DifferentialDrive(self.m_left, self.m_right)
        self.drivetrain.setExpiration(0.1)

        # joystick 0
        self.stick = wpilib.Joystick(0)

        self.m_right.setInverted(True)

        self.mode = 0

    def teleopInit(self):
        self.drivetrain.setSafetyEnabled(True)

    def teleopPeriodic(self):
        if self.stick.getRawButton(xbox["select"]) == True:
            self.mode += 1

        if self.mode == 0:
            self.drivetrain.arcadeDrive(
                self.stick.getRawAxis(xbox["left-y-stick"]),
                self.stick.getRawAxis(xbox["right-x-stick"]),
                True,
            )
        # Cheezy Drive
        elif self.mode == 1:
            self.l_trigger_value = self.stick.getRawAxis(xbox["lt"])
            self.r_trigger_value = self.stick.getRawAxis(xbox["rt"])

            self.combined_triggered_value = self.r_trigger_value - self.l_trigger_value

            # Best Drive
            self.drivetrain.arcadeDrive(
                self.combined_triggered_value,
                self.stick.getRawAxis(xbox["left-x-stick"]),
                True,
            )

        elif self.mode > 1:
            self.mode = 0


if __name__ == "__main__":
    wpilib.run(Drivetrain)
