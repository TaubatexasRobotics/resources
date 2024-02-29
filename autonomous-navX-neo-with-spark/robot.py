import wpilib
import wpilib.drive
import phoenix5
import rev
from navx import AHRS
import wpimath


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""
        # creating spark max like encoders in setup
        self.sparkMaxEncoderLeft = rev.CANSparkMax(
            52, rev.CANSparkMax.MotorType.kBrushless
        )
        self.sparkMaxEnocderRight = rev.CANSparkMax(
            51, rev.CANSparkMax.MotorType.kBrushless
        )
        # create navX to my robot
        self.navx = AHRS.create_spi()
        # Configure the encoder
        self.encoderLeft = self.sparkMaxEncoderLeft.getEncoder()
        self.encoderRight = self.sparkMaxEnocderRight.getEncoder()

        leftPosition = self.encoderLeft.getPosition()
        rightPosition = self.encoderRight.getPosition()

        rotation = wpimath.geometry._geometry.Rotation2d(self.navx.getAngle())
        inititalPose = wpimath.geometry._geometry.Pose2d(0, 0, rotation)
        self.odometry = wpimath.kinematics.DifferentialDriveOdometry(
            rotation, leftPosition, rightPosition, inititalPose
        )

    #   self.odometry = wpimath.kinematics.DifferentialDriveOdometry(
    #       rotation, leftPosition, rightPosition, inititalPose
    #   )
    def teleopInit(self):
        pass

    def autonomousPeriodic(self):
        # variable responsible for the get yaw angle
        yaw = self.navx.getAngle()
        # variables to positon and velocity of robot
        positionLeft = self.encoderLeft.getPosition()
        positionRight = self.encoderRight.getPosition()
        velocityLeft = self.encoderLeft.getVelocity()

        rotation2D = wpimath.geometry._geometry.Rotation2d(self.navx.getAngle())
        positionLeftCM = positionLeft * 47.8586
        positionRightCM = positionRight * 47.8586
        self.odometry.update(rotation2D, positionLeftCM, positionRightCM)
        print(self.odometry.getPose())
        # print(yaw)
        # print(f"Position left: {positionLeftCM} cm")
        # print(f"Position right: {positionRightCM} cm")
        # print(f"Velocity: {velocity} RPM")

    def teleopPeriodic(self):
        pass
