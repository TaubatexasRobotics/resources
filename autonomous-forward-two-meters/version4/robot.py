from navx import AHRS
import wpilib
from wpilib.interfaces import GenericHID
from wpilib.drive import DifferentialDrive


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.front_left_motor = wpilib.PWMVictorSPX(0)
        self.front_right_motor = wpilib.PWMVictorSPX(1)
        self.back_left_motor = wpilib.PWMVictorSPX(2)
        self.back_right_motor = wpilib.PWMVictorSPX(3)
        self.left_encoder = wpilib.Encoder(0, 1)
        self.right_encoder = wpilib.Encoder(2, 3)
        self.navx = AHRS.create_spi()

        self.robot_drive = DifferentialDrive(
            wpilib.SpeedControllerGroup(self.front_left_motor, self.back_left_motor),
            wpilib.SpeedControllerGroup(self.front_right_motor, self.back_right_motor),
        )

    def autonomousInit(self):
        self.target_distance = 2  # meters
        self.target_angle = 90  # degrees

        # Reset encoders
        self.left_encoder.reset()
        self.right_encoder.reset()

    def autonomousPeriodic(self):
        # Move forward 2 meters
        if self.left_encoder.getDistance() < self.target_distance:
            self.robot_drive.tankDrive(0.5, 0.5)
        else:
            # Stop the motors and reset encoders
            self.robot_drive.stopMotor()
            self.left_encoder.reset()
            self.right_encoder.reset()

            # Turn right 90 degrees
            if self.navx.getYaw() < self.target_angle:
                self.robot_drive.tankDrive(0.5, -0.5)
            else:
                # Stop the motors and reset encoders
                self.robot_drive.stopMotor()
                self.left_encoder.reset()
                self.right_encoder.reset()

                # Move forward 4 meters
                if self.left_encoder.getDistance() < self.target_distance * 2:
                    self.robot_drive.tankDrive(0.5, 0.5)
                else:
                    # Stop the motors and reset encoders
                    self.robot_drive.stopMotor()
                    self.left_encoder.reset()
                    self.right_encoder.reset()

                    # Turn left 90 degrees
                    if self.navx.getYaw() > -self.target_angle:
                        self.robot_drive.tankDrive(-0.5, 0.5)
                    else:
                        # Stop the motors and reset encoders
                        self.robot_drive.stopMotor()
                        self.left_encoder.reset()
                        self.right_encoder.reset()

                        # Move forward 2 meters
                        if self.left_encoder.getDistance() < self.target_distance:
                            self.robot_drive.tankDrive(0.5, 0.5)
                        else:
                            # Stop the motors and reset encoders
                            self.robot_drive.stopMotor()
                            self.left_encoder.reset()
                            self.right_encoder.reset()
                            self.target_distance_reached = True
