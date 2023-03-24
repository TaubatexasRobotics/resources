import wpilib
import rev
CAN_ID = 52

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""

        self.rev_motor = rev.CANSparkMax(CAN_ID, rev.CANSparkMax.MotorType.kBrushless)
        self.pid_controller = self.rev_motor.getPIDController()

        Kp, Ki, Kd = (0.1, 0, 0.1)

        self.pid_controller.setP(Kp)
        self.pid_controller.setI(Ki)
        self.pid_controller.setD(Kd)

        self.motor_position = 10
        
    def teleopPeriodic(self):
        self.pid_controller.setReference(self.motor_position, rev.CANSparkMax.ControlType.kPosition)
        
if __name__ == "__main__":
    wpilib.run(MyRobot)
