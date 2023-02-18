import wpilib
import photonvision
import wpimath
import wpimath.controller

# Generic Values
CAMERA_HEIGHT_METERS = 1
TARGET_HEIGHT_METERS = 1
CAMERA_PITCH_RADIANS = 1
GOAL_RANGE_METERS = 2

kP = 0.1
kI = 0.2
kD = 0.3

class PhotonVisionTest(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.camera = photonvision.PhotonCamera('Microsoft_LifeCam_HD-3000')
        self.joystick = wpilib.Joystick(0)
        self.controller = wpimath.controller.PIDController(
            0.1, 0.2, 0.3
        )

    def robotPeriodic(self) -> None:
        self.distanceToTarget = photonvision.PhotonUtils.getDistanceToPose(
            
        )

    def teleopInit(self) -> None:
        # Calculating Distance to Target
        if self.joystick.getRawButton(0):
            result = self.camera.getLatestResult()

            if result.hasTargets():
                r = photonvision.PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS,
                    TARGET_HEIGHT_METERS,
                    CAMERA_PITCH_RADIANS,
                    wpimath.units.degreesToRadians(result.getBestTarget().getPitch())
                )
            
            forward = self.controller.calculate(r, GOAL_RANGE_METERS)

if __name__ == '__main__':
    wpilib.run(PhotonVisionTest)