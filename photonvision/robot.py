import wpilib
import photonvision
import constants
import wpimath.controller
import wpilib.drive
import ctre

class myRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.photoCamera = photonvision.PhotonCamera('Teste')
        self.controller = wpimath.controller.PIDController(
            constants.kP, constants.kI, constants.kD
        )
        self.drive = wpilib.drive.DifferentialDrive(
            wpilib.MotorControllerGroup(ctre.WPI_VictorSPX(0), ctre.WPI_VictorSPX(1)),
            wpilib.MotorControllerGroup(ctre.WPI_VictorSPX(2), ctre.WPI_VictorSPX(3)),
        )

    def robotPeriodic(self):
        result = self.photoCamera.getLatestResult()
        if (result.hasTargets()):
            print('achei')
            rangeTarget = photonvision.PhotonUtils.calculateDistanceToTargetMeters(
                constants.CAMERA_HEIGHT_METERS,
                constants.TARGET_HEIGHT_METERS,
                constants.CAMERA_PITCH_RADIANS,
                wpimath.Units.degreesToRadians(result.getBestTarget().getPitch())
            )
            forwardSpeed = self.controller.calculate(rangeTarget, constants.GOAL_RANGE_METERS)
            self.drive.arcadeDrive(forwardSpeed, 0, True)

if __name__ == '__main__':
    wpilib.run(myRobot)
