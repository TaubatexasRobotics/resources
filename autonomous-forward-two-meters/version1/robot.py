import wpilib
from wpilib.drive import DifferentialDrive
from navx import AHRS
import math
import ctre

C_LEFT_BACK = 1
C_LEFT_FRONT = 2
C_RIGHT_FRONT = 3
C_RIGHT_BACK = 4


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Set up the two encoders
        self.left_encoder = wpilib.Encoder(1, 2)
        self.right_encoder = wpilib.Encoder(3, 4)

        # Set up the NavX
        self.navx = AHRS.create_spi()
        # Set up the drivetrain
        self.m_left_back = ctre.WPI_VictorSPX(C_LEFT_BACK)
        self.m_left_front = ctre.WPI_VictorSPX(C_LEFT_FRONT)
        self.m_right_front = ctre.WPI_VictorSPX(C_RIGHT_FRONT)
        self.m_right_back = ctre.WPI_VictorSPX(C_RIGHT_BACK)

        self.m_left = wpilib.MotorControllerGroup(self.m_left_front, self.m_left_back)
        self.m_right = wpilib.MotorControllerGroup(
            self.m_right_front, self.m_right_back
        )
        self.myRobot = wpilib.drive.DifferentialDrive(self.m_left, self.m_right)
        self.stick = wpilib.Joystick(0)

        # Set up variables to keep track of the robot's position and orientation
        self.x = 0
        self.y = 0
        self.heading = 0
        # Define constants for the PID controller
        self.kP = 0.05  # Proportional gain
        self.kI = 0.0  # Integral gain (not used)
        self.kD = 0.0  # Derivative gain (not used)

        # Define constants for the encoder speed control
        self.kP_encoder = 0.1  # Proportional gain

    def autonomousInit(self):
        # Reset the encoders and NavX
        self.left_encoder.reset()
        self.right_encoder.reset()
        self.navx.reset()
        # até aqui ok
        # Set a target distance of 2 meters
        target_distance = 2

        # Drive the robot forward until it has traveled the target distance
        while (
            self.left_encoder.getDistance() + self.right_encoder.getDistance()
        ) / 2 < target_distance:
            # Get the robot's orientation from the NavX
            angle = self.navx.getYaw()

            # Use a PID controller to adjust the robot's heading
            error = angle
            turn_rate = self.kP * error

            # Set the robot's speed and turn rate based on the encoder data and NavX heading
            speed_error = (
                target_distance
                - (self.left_encoder.getDistance() + self.right_encoder.getDistance())
                / 2
            )
            speed = self.kP_encoder * speed_error
            self.myRobot.arcadeDrive(speed, turn_rate)

            # Update the robot's position based on the encoder data
            distance = (
                self.left_encoder.getDistance() + self.right_encoder.getDistance()
            ) / 2
            self.x += distance * math.cos(math.radians(self.heading))
            self.y += distance * math.sin(math.radians(self.heading))

            # Wait for the next loop iteration
            wpilib.Timer.delay(0.02)

        # Stop the robot when it has reached the target distance
        self.myRobot.arcadeDrive(0, 0)

    def autonomousPeriodic(self):
        pass
