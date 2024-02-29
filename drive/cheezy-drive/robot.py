from wpilib import TimedRobot, Joystick, run
from phoenix5 import WPI_VictorSPX, ControlMode

kLeftFront = 1
kLeftBack = 2
kRightFront = 3
kRightBack = 4

kDriverJoystick = 0

g_xbox_360 = {
    "left-x-stick": 0,
    "left-y-stick": 1,
    "lt": 2,
    "rt": 3,
    "right-x-stick": 4,
    "right-y-stick": 5,
}


class CheezyDrive(TimedRobot):
    def robotInit(self):
        self.left_front = WPI_VictorSPX(kLeftFront)
        self.left_back = WPI_VictorSPX(kLeftBack)
        self.right_front = WPI_VictorSPX(kRightFront)
        self.right_back = WPI_VictorSPX(kRightBack)

        self.joystick = Joystick(0)

    def speedControllerGroupByPercentage(self, controller_1, controller_2, value):
        controller_1.set(ControlMode.PercentOutput, value)
        controller_2.set(ControlMode.PercentOutput, value)

    def cheezyDrive(self, arcade_drive: bool, forward: float, turn: float):
        turn_power = turn if arcade_drive else turn * abs(forward)
        self.speedControllerGroupByPercentage(
            self.left_back, self.left_front, forward + turn
        )
        self.speedControllerGroupByPercentage(
            self.left_back, self.left_front, forward - turn
        )

    def teleopPeriodic(self):
        self.cheezyDrive(
            self.joystick.getRawButton(g_xbox_360["rt"]),
            -self.joystick.getRawAxis(g_xbox_360["left-y-stick"]),
            self.joystick.getRawAxis(g_xbox_360["right-x-stick"]),
        )
