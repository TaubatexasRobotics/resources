import wpilib

limitSwitch = wpilib.DigitalInput(0)
analog_input = wpilib.AnalogInput(0)


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        pass

    def robotPeriodic(self):
        # print(analog_input.getVoltage())
        print(limitSwitch.get())


if __name__ == "__main__":
    wpilib.run(MyRobot)
