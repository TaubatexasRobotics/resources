import ntcore
import wpilib
import phoenix5

class EasyNetworkTableExample(wpilib.TimedRobot):
    def robotInit(self) -> None:
        inst = ntcore.NetworkTableInstance.getDefault()

        table = inst.getTable("motor")

        self.xPub = table.getDoubleTopic("x").publish()
        self.yPub = table.getDoubleTopic("y").publish()

        wpilib.SmartDashboard.getNumber("força", 0)

        self.motor = phoenix5.WPI_VictorSPX(0)
        self.x = 0
        self.y = 0

    def robotPeriodic(self) -> None:
        self.xPub.set(self.x)
        self.yPub.set(self.y)
        self.x += 0.05
        self.y += 1.0
