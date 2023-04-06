import wpilib
import wpilib.drive
import ctre
import rev
from navx import AHRS
import wpimath


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.encoder = wpilib.Encoder(8, 9)

    def teleopInit(self):
        pass

    def autonomousPeriodic(self):

        print(self.encoder.get())

    def teleopPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
