#!/usr/bin/env python3

import wpilib
import wpilib.drive
import ctre

C_LEFT_BACK = 11
C_LEFT_FRONT = 22
C_RIGHT_FRONT = 33
C_RIGHT_BACK = 44

C_BUFFER = 1

class Drivetrain(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""

        # motor controllers for traction
        self.m_left_back = ctre.WPI_VictorSPX(C_LEFT_BACK)
        self.m_left_front = ctre.WPI_VictorSPX(C_LEFT_FRONT)
        self.m_right_front = ctre.WPI_VictorSPX(C_RIGHT_FRONT)
        self.m_right_back = ctre.WPI_VictorSPX(C_RIGHT_BACK)

        self.m_left = wpilib.SpeedControllerGroup(self.m_left_front, self.m_left_back)
        self.m_right = wpilib.SpeedControllerGroup(self.m_right_front, self.m_right_back)
	
        # object that handles basic drive operations
        self.drivetrain = wpilib.drive.DifferentialDrive(self.m_left, self.m_right)
        self.drivetrain.setExpiration(0.1)

        # joystick 0
        self.stick = wpilib.Joystick(0)

    def teleopInit(self):
        self.drivetrain.setSafetyEnabled(True)

    def teleopPeriodic(self):
        self.drivetrain.arcadeDrive(
            self.stick.getRawAxis(1), self.stick.getRawAxis(0) * C_BUFFER, True
        )

if __name__ == "__main__":
    wpilib.run(Drivetrain)
