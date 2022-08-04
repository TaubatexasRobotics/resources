import wpilib
from wpilib.drive import DifferentialDrive
from ctre import WPI_VictorSPX
from buttons import g_xbox_360

# Victor SPX motor controllers
C_LEFT_FRONT = 1
C_LEFT_BACK = 2
C_RIGHT_FRONT = 3
C_RIGHT_BACK = 4

C_CLIMBER_LEFT = 5
C_CLIMBER_RIGHT = 6
C_CLIMBER_ANGLE_LEFT = 7
C_CLIMBER_ANGLE_RIGHT = 8

C_SHOOTER = 9

# DMC60
C_CONVEYOR = 0

# Talon SR
C_INTAKE = 0

# Joystick
C_DRIVER_JOYSTICK = 0

class TaubatexasRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Drivetrain
        self.m_left_front = WPI_VictorSPX(C_LEFT_FRONT)
        self.m_left_back = WPI_VictorSPX(C_LEFT_BACK)

        self.m_right_front = WPI_VictorSPX(C_RIGHT_FRONT)
        self.m_right_back = WPI_VictorSPX(C_RIGHT_BACK)

        self.m_left = wpilib.SpeedControllerGroup(self.m_left_front, self.m_left_back)
        self.m_right = wpilib.SpeedControllerGroup(self.m_right_front, self.m_right_back)
        self.drivetrain = DifferentialDrive(self.m_left, self.m_right)

        self.drivetrain.setExpiration(0.1)

        # Joysitck
        self.joystick = wpilib.Joystick(C_DRIVER_JOYSTICK)

        # Create Timer
        self.timer = wpilib.Timer()

        # Climber
        self.m_climber_left = WPI_VictorSPX(C_CLIMBER_LEFT)
        self.m_climber_right = WPI_VictorSPX(C_CLIMBER_RIGHT)

        self.m_climber_angle_left = WPI_VictorSPX(C_CLIMBER_ANGLE_LEFT)
        self.m_climber_angle_right = WPI_VictorSPX(C_CLIMBER_ANGLE_RIGHT)

        self.m_climber = wpilib.SpeedControllerGroup(self.m_climber_left, self.m_climber_right)
        self.m_climber_angle = wpilib.SpeedControllerGroup(self.m_climber_angle_left, self.m_climber_angle_right)

        # Conveyor
        self.m_conveyor = wpilib.DMC60(C_CONVEYOR)

        # Intake
        self.m_intake = wpilib.Talon(C_INTAKE)

        # Shooter
        self.m_shooter = WPI_VictorSPX(C_DRIVER_JOYSTICK)


    def autonomousInit(self):
        self.timer.reset()
        self.timer.start()
    
    def autonomousPeriodic(self):
        # Autonomous
        if self.timer.get() < 2.0:
            self.drivetrain.arcadeDrive(0.5, 0)
        else:
            self.drivetrain.arcadeDrive(0, 0)

    def teleopInit(self):
        # Enable Safety
        self.drivetrain.setSafetyEnabled(True)

    def teleopPeriodic(self):
        # Arcade Drive
        if self.joystick.getRawAxis(g_xbox_360['lt']) > 0:
            self.drivetrain.arcadeDrive(
                self.joystick.getRawAxis(g_xbox_360['lt'] * -1), 
                self.joystick.getRawAxis(g_xbox_360['left-x-stick'])
            )
        else:
            self.drivetrain.arcadeDrive(
                self.joystick.getRawAxis(g_xbox_360['rt']), 
                self.joystick.getRawAxis(g_xbox_360['left-x-stick'])
            )            

        #self.drivetrain.arcadeDrive(self.joystick.getY(), self.joystick.getX())

        # Climber
        if self.joystick.getRawButton(g_xbox_360['lb']):
            self.m_climber.set(-1)
        elif self.joystick.getRawButton(g_xbox_360['rb']):
            self.m_climber.set(1)
        else:
            self.m_climber.set(0)

        # Shooter
        self.m_shooter.set(1) if self.joystick.getRawButton(g_xbox_360['x']) else self.m_shooter.set(0)
        
        # Climber
        self.m_climber_angle.set(self.joystick.getRawAxis(g_xbox_360['right-x-stick']))

        # Intake & Conveyor
        if self.joystick.getRawButton(g_xbox_360['a']):
            self.m_intake.set(1)
            self.m_conveyor.set(1)
        else:
            self.m_intake.set(0)
            self.m_conveyor.set(0)

if __name__ == "__main__":
    wpilib.run(TaubatexasRobot)