import wpilib
import rev
import wpilib.drive
import math
from commands2 import Subsystem
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPLTVController
from pathplannerlib.config import RobotConfig
from wpilib import DriverStation
from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.kinematics import ChassisSpeeds
from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.kinematics import DifferentialDriveWheelSpeeds
from wpimath.units import inchesToMeters
from wpimath.units import inchesToMeters


class DriveSubsystem(Subsystem):
    def __init__(self):
        self.motor1 = rev.SparkMax(51, rev.SparkLowLevel.MotorType.kBrushless)
        self.motor2 = rev.SparkMax(52, rev.SparkLowLevel.MotorType.kBrushless)
        self.motor3 = rev.SparkMax(53, rev.SparkLowLevel.MotorType.kBrushless)
        self.motor4 = rev.SparkMax(54, rev.SparkLowLevel.MotorType.kBrushless)

        config = rev.SparkMaxConfig()

        config.closedLoop \
        .setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder) \
        .setP(0.0002) \
        .setI(0.0) \
        .setD(0.0) \
        .setFF(0.00015)

        self.motor1.configure(config, rev.SparkBase.ResetMode.kResetSafeParameters)
        self.motor3.configure(config, rev.SparkBase.ResetMode.kResetSafeParameters)

        self.rightClosedLoop = self.motor1.getClosedLoopController()
        self.leftClosedLoop = self.motor3.getClosedLoopController()


        self.leftMotor = wpilib.MotorControllerGroup(self.motor1,self.motor2)
        self.rightMotor = wpilib.MotorControllerGroup(self.motor3,self.motor4)

        self.diferentialDrive = wpilib.drive.DifferentialDrive(self.leftMotor,self.rightMotor)
        # Load the RobotConfig from the GUI settings. You should probably
        # store this in your Constants file
        config = RobotConfig.fromGUISettings()

        # Configure the AutoBuilder last
        AutoBuilder.configure(
            self.getPose, # Robot pose supplier
            self.resetPose, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speeds, feedforwards: self.driveRobotRelative(speeds), # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPLTVController(0.02), # PPLTVController is the built in path following controller for differential drive trains
            config, # The robot configuration
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )

        self.kinematics = DifferentialDriveKinematics(inchesToMeters(27.0))
        self.wheelCircunference = inchesToMeters(6) * math.pi

    def shouldFlipPath():
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
    
    def getPose(self):
        pass

    def resetPose(self):
        pass

    def driveRobotRelative(self, chassiSpeed):
        wheelSpeeds = self.kinematics.toWheelSpeeds(chassiSpeed)
        
        leftRpm = (wheelSpeeds.left / self.wheelCircunference) * 60
        rightRpm = (wheelSpeeds.right / self.wheelCircunference) * 60

        leftMotorRpm = leftRpm * 10.7
        rightMotorRpm = rightRpm * 10.7

        self.leftClosedLoop.setReference(leftMotorRpm, rev.SparkBase.ControlType.kVelocity)
        self.rightClosedLoop.setReference(rightMotorRpm, rev.SparkBase.ControlType.kVelocity)

    def getRobotRelativeSpeeds(self):

        leftSpeed = self.getLeftVelocityMetersPerSecond()
        rightSpeed = self.getRightVelocityMetersPerSecond()

        wheelSpeeds = DifferentialDriveWheelSpeeds(leftSpeed,rightSpeed)

        return self.kinematics.toChassisSpeeds(wheelSpeeds)

    def getLeftVelocityMetersPerSecond(self):
        
        gearRatio = 10.7
        rpm = self.motor1.getEncoder().getVelocity() / gearRatio

        return (rpm / 60) * self.wheelCircunference

    def getRightVelocityMetersPerSecond(self):
        
        gearRatio = 10.7
        rpm = self.motor3.getEncoder().getVelocity() / gearRatio

        return (rpm / 60) * self.wheelCircunference