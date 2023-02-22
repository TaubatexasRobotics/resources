import ntcore
import wpilib
import ctre

from dataclasses import dataclass

C_LEFT_BACK = 11
C_LEFT_FRONT = 22
C_RIGHT_FRONT = 33
C_RIGHT_BACK = 44

@dataclass
class Option:
    """Class for keeping track of an item in inventory."""
    name: str

op1 = Option('asdf1')

class EasyNetworkTableExample(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.m_left_back = ctre.WPI_VictorSPX(C_LEFT_BACK)
        self.m_left_front = ctre.WPI_VictorSPX(C_LEFT_FRONT)
        self.m_right_front = ctre.WPI_VictorSPX(C_RIGHT_FRONT)
        self.m_right_back = ctre.WPI_VictorSPX(C_RIGHT_BACK)

        self.m_left = wpilib.MotorControllerGroup(self.m_left_front, self.m_left_back)
        self.m_right = wpilib.MotorControllerGroup(self.m_right_front, self.m_right_back)

        inst = ntcore.NetworkTableInstance.getDefault()

        table = inst.getTable("motor")
        self.xPub = table.getDoubleTopic("x").publish()
        self.yPub = table.getDoubleTopic("y").publish()
        self.cPub = table.getDoubleArrayTopic("c").publish()
        self.c = [0,1,2,1]

        #wpilib.SendableChooser.addOption("testea", op1)

        #self.test = wpilib.SendableChooser()

        #self.test.initSendable(inst)
        #self.test.addOption("ease", 1)

        table2 = inst.getTable("combobox1")
        table2.putNumberArray("controle 1", [1,2,3,4])      

        wpilib.SmartDashboard.getNumber("Forca", 0)
        
        self.x = 0
        self.y = 0

    def robotPeriodic(self) -> None:

        self.xPub.set(self.x, 0)
        self.yPub.set(self.y, 0)
        self.cPub.set(self.c, 0)

        
if __name__ == "__main__":
    wpilib.run(EasyNetworkTableExample)