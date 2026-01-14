import wpilib
import phoenix5
from wpimath.trajectory import TrapezoidProfile

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        self.motor = phoenix5.WPI_VictorSPX(1)
        self.controle = wpilib.Joystick(0)
        self.constraints = TrapezoidProfile.Constraints(
            maxVelocity = 2.0,      
            maxAcceleration=2.0  
        )

        self.profile = TrapezoidProfile(self.constraints)

        self.currentState = TrapezoidProfile.State(0)
    
    def teleopPeriodic(self):
        
        
        targetSpeed = self.controle.getRawAxis(0)  # -1 a 1
        goal = TrapezoidProfile.State(targetSpeed)

        dt = 0.02

        self.currentState = self.profile.calculate(dt, self.currentState, goal)

        self.motor.set(self.currentState.position)
        


        