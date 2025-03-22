# WARNING: It's only possible to activate wpilib.Relay in teleop/autonomous mode.

import wpilib

class RobotWithRelay(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.relay = wpilib.Relay(0)
        self.sensor = wpilib.DigitalInput(0)

    def teleopInit(self) -> None:
        self.relay.set(wpilib.Relay.Value.kOff)

    def teleopPeriodic(self) -> None:
        if self.sensor.get() is False:
            self.relay.set(wpilib.Relay.Value.kOn)
        else:
            self.relay.set(wpilib.Relay.Value.kOff)
