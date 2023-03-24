import wpilib
from wpilib.drive import DifferentialDrive
from navx import AHRS
import math
import ctre
class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        # Set up the two encoders
        self.left_encoder = wpilib.Encoder(1, 2)
        self.right_encoder = wpilib.Encoder(3, 4)

        # Set up the NavX
        self.navx = AHRS.create_spi()

        # Set up the drivetrain
        self.m_left_front = ctre.WPI_VictorSPX(2)
        self.m_right_front = ctre.WPI_VictorSPX(1)
        self.m_left_back = ctre.WPI_VictorSPX(4)
        self.m_right_back = ctre.WPI_VictorSPX(3)

        self.m_left= wpilib.MotorControllerGroup(self.m_left_front,self.m_left_back)   
        self.m_right= wpilib.MotorControllerGroup(self.m_right_front,self.m_right_back) 
        self.drive  = wpilib.drive.DifferentialDrive(self.m_left, self.m_right) 

        # Define constants for the PID controller
        self.kP = 0.05  # Proportional gain
        self.kI = 0.0   # Integral gain (not used)
        self.kD = 0.0   # Derivative gain (not used)

        # Define constants for the encoder speed control
        self.kP_encoder = 0.1  # Proportional gain

        # Set up the PID controller
        self.pid_controller = wpilib.PIDController(self.kP, self.kI, self.kD, source=self.navx.getYaw, output=self.drive.arcadeDrive)
        self.pid_controller.setInputRange(-180.0, 180.0)
        self.pid_controller.setOutputRange(-1.0, 1.0)
        self.pid_controller.setAbsoluteTolerance(1.0)
        self.pid_controller.setContinuous(True)

    def autonomousInit(self):
        # Reset the encoders and NavX
        self.left_encoder.reset()
        self.right_encoder.reset()
        self.navx.reset()

        # Move forward 3 meters
        target_distance = 3
        self.move_forward(target_distance)

        # Turn left 45 degrees
        target_angle = self.navx.getYaw() - 45
        self.turn_to_angle(target_angle)

        # Move forward 2 meters
        target_distance = 2
        self.move_forward(target_distance)

    def move_forward(self, target_distance):
        # Set a target distance
        self.pid_controller.setSetpoint(0)

        # Start the PID controller
        self.pid_controller.enable()

        # Drive the robot forward until it has traveled the target distance
        while (self.left_encoder.getDistance() + self.right_encoder.getDistance()) / 2 < target_distance:
            # Set the robot's speed based on the encoder data
            speed_error = target_distance - (self.left_encoder.getDistance() + self.right_encoder.getDistance()) / 2
            speed = self.kP_encoder * speed_error

            # Wait for the next loop iteration
            wpilib.Timer.delay(0.02)

        # Stop the robot when it has reached the target distance
        self.drive.arcadeDrive(0, 0)

        # Disable the PID controller
        self.pid_controller.disable()

    def turn_to_angle(self, target_angle):
        # Set a target angle
        self.pid_controller.setSetpoint(target_angle)

        # Start the PID controller
        self.pid_controller.enable()

        # Turn the robot until it has reached the target angle
        while not self.pid_controller.onTarget():
            # Wait for the next loop iteration
            wpilib.Timer.delay(0.02)

        # Stop the robot when it has reached the target angle
        self.drive.arcadeDrive(0, 0)

        # Disable the PID controller
        self.pid_controller.disable()

    def autonomousPeriodic(self):
        pass  # Nothing to do in this