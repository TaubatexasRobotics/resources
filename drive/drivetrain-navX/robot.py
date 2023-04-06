#!/usr/bin/env python3

import wpilib
import wpilib.drive
import ctre


C_LEFT_BACK = 4
C_LEFT_FRONT = 3
C_RIGHT_FRONT = 2
C_RIGHT_BACK = 1

C_BUFFER = 1

from navx import AHRS


class MyRobot(wpilib.TimedRobot):
    def get_yaw_angle(self):
        try:
            # Create an instance of the AHRS class
            # Get the current yaw angle
            yaw = self.navx.getAngle()
            # Print the yaw angle
            # print("Yaw angle: ", yaw)
            return yaw
        except Exception as e:
            print(e)

    def robotInit(self):
        """Robot initialization function"""
        self.navx = AHRS.create_spi()
        # self.initialYaw = self.get_yaw_angle()

        # motor controllers for traction
        self.m_left_back = ctre.WPI_VictorSPX(C_LEFT_BACK)
        # self.m_left_back.setInverted(True)
        self.m_left_front = ctre.WPI_VictorSPX(C_LEFT_FRONT)
        # self.m_left_back.setInverted(True)
        self.m_right_front = ctre.WPI_VictorSPX(C_RIGHT_FRONT)
        self.m_right_front.setInverted(True)
        self.m_right_back = ctre.WPI_VictorSPX(C_RIGHT_BACK)
        self.m_right_back.setInverted(True)

        # self.m_left = wpilib.SpeedControllerGroup(self.m_left_front, self.m_left_back)
        # self.m_right = wpilib.SpeedControllerGroup(self.m_right_front, self.m_right_back)

        # object that handles basic drive operations
        self.drivetrain = wpilib.drive.DifferentialDrive(
            self.m_left_back, self.m_right_back
        )
        self.drivetrain2 = wpilib.drive.DifferentialDrive(
            self.m_left_front, self.m_right_front
        )

        self.drivetrain.setExpiration(0.1)

        # joystick 0
        # self.stick = wpilib.Joystick(0)

    def teleopInit(self):
        # self.drivetrain.setSafetyEnabled(True)
        pass

    def teleopPeriodic(self):
        self.drivetrain.arcadeDrive(
            self.stick.getRawAxis(1), self.stick.getRawAxis(0) * C_BUFFER, True
        )

    def autonomousInit(self):
        self.navx.reset()
        # self.drivetrain.setSafetyEnabled(True)
        pass

    def autonomousPeriodic(self):

        # self.m_left_front.set(0.6)
        # self.m_right_front.set(0.6)
        # self.m_right_back.set(0.6)
        # self.m_left_back.set(0.6)
        yaw = self.navx.getAngle()

        erro = 0.5 * (-yaw)
        print(erro)
        erro = max(erro, 0.3)
        potencia = 0
        self.drivetrain.arcadeDrive(potencia, erro, True)
        self.drivetrain2.arcadeDrive(potencia, erro, True)


if __name__ == "__main__":
    wpilib.run(MyRobot)
