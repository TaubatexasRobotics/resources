from wpilib import TimedRobot, PWMVictorSPX, Joystick
from wpilib.drive import DifferentialDrive
from wpimath.controller import PIDController
from photonlibpy import PhotonCamera

LINEAR_PID = (0.1, 0, 0)
ANGULAR_PID = (0.1, 0, 0)
CAMERA_NAME = "Microsoft_LifeCam_3000"


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
        forward = self.joystick.getRawAxis(0)
        if self.joystick.getRawButton(1):
            result = camera.getLatestResult()
            if result.hasTargets():
                rotation = -turn_controller.calculate(
                    result.getBestTarget().getYaw(), 0
                )
            else:
                rotation = 0
        else:
            rotation = self.joystick.getRawAxis(1)
        self.drivetrain.arcadeDrive(forward, rotation)
