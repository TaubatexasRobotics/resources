from wpilib import TimedRobot, PWMVictorSPX, Joystick
from wpilib.drive import DifferentialDrive
from wpimath.controller import PIDController
from photonlibpy import PhotonCamera
from wpimath.units import (
    degreesToRadians,
    inchesToMeters,
    feetToMeters,
    degreesToRadians,
)
from utils import PhotonUtils

LINEAR_PID = (0.1, 0, 0)
ANGULAR_PID = (0.1, 0, 0)
CAMERA_NAME = "Microsoft_LifeCam_3000"

CAMERA_HEIGHT_METERS = inchesToMeters(24)
TARGET_HEIGHT_METERS = feetToMeters(5)
CAMERA_PITCH_RADIANS = degreesToRadians(0)


class TestBot(TimedRobot):
    def robotInit(self) -> None:
        self.forward_controller = PIDController(*LINEAR_PID)
        self.turn_controller = PIDController(*ANGULAR_PID)

        self.left_motor = PWMVictorSPX(1)
        self.right_motor = PWMVictorSPX(2)

        self.drivetrain = DifferentialDrive(self.left_motor, self.right_motor)

        self.joystick = Joystick(0)
        self.camera = PhotonCamera(CAMERA_NAME)

    def teleopPeriodic(self) -> None:
        rotation = 0
        forward = 0
        if self.joystick.getRawButton(1):
            result = self.camera.getLatestResult()
            if result.hasTargets():
                tag_range = PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS,
                    TARGET_HEIGHT_METERS,
                    CAMERA_PITCH_RADIANS,
                    degreesToRadians(target.getPitch()),
                )
                forward = -self.forward_controller.calculate(
                    tag_range, GOAL_RANGE_METERS
                )
                rotation = -self.turn_controller.calculate(
                    result.getBestTarget().getYaw(), 0
                )
            else:
                rotation = 0
                forward = 0
        else:
            rotation = self.joystick.getRawAxis(0)
            forward = self.joystick.getRawAxis(1)
        self.drivetrain.arcadeDrive(forward, rotation)
