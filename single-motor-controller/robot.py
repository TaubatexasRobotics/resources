#!/usr/bin/env python3

import wpilib
import ctre

C_ID = 11
C_BUTTON_ACTIVATE = 1
C_BUTTON_DEACTIVATE = 2
C_VALUE_ACTIVATE = 0.6
C_VALUE_DEACTIVATE = 0.2
C_BUFFER_ACTIVATE = 1
C_BUFFER_DEACTIVATE = 1

class SingleMotorController(wpilib.TimedRobot):
    def robotInit(self):
        self.m_controller = ctre.WPI_VictorSPX(C_ID)

        # joystick 0
        self.stick = wpilib.Joystick(0)

    def teleopInit(self):
        self.m_controller.set(0)

    def teleopPeriodic(self):
        if self.stick.getRawButton(C_BUTTON_ACTIVATE) == True:
            self.m_controller.set(C_VALUE_ACTIVATE * C_BUFFER_ACTIVATE)
        elif self.stick.getRawButton(C_BUTTON_DEACTIVATE) == True:
            self.m_controller.set(C_VALUE_DEACTIVATE * C_BUFFER_DEACTIVATE)
        else:
            self.m_controller.set(0)

if __name__ == "__main__":
    wpilib.run(SingleMotorController)
