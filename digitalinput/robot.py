import wpilib



class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.limitSwitch = wpilib.DigitalInput(0)
        self.analog_input = wpilib.AnalogInput(0)


    def robotPeriodic(self):
        # print(self.analog_input.getVoltage())
        print(self.limitSwitch.get())


if __name__ == "__main__":
    wpilib.run(MyRobot)
