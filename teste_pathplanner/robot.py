import wpilib
import commands2
from RobotContainer import RobotContainer
# TODO: insert robot code here
class MyRobot(commands2.TimedCommandRobot):

    def robotInit(self):
        self.robotContainer = RobotContainer()

    def autonomousInit(self):
        self.autoCommand = self.robotContainer.getAuto()    

        if self.autoCommand:
            self.autoCommand.schedule()