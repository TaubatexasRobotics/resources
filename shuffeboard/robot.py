import wpilib
import wpilib.shuffleboard

Shuffleboard = wpilib.shuffleboard.Shuffleboard

chooser = wpilib.SendableChooser()
chooser.addOption('option1', '1')
chooser.addOption('option2', '2')

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):        
        self.sd = Shuffleboard.getTab("My Tab")
        my_tab = Shuffleboard.getTab("My Tab")
        my_tab.add('choose', chooser)
    
    def robotPeriodic(self):
        value = chooser.getSelected()
        print(value)

if __name__ == "__main__":
    wpilib.run(MyRobot)