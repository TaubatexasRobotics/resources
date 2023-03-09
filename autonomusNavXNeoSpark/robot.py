import wpilib
import wpilib.drive
import ctre
import rev
from navx import AHRS
import wpimath
class MyRobot(wpilib.TimedRobot):
        
    def robotInit(self):
      """Robot initialization function"""
      #creating spark max like encoders in setup
      self.sparkMaxEncoderLeft = rev.CANSparkMax(52, rev.CANSparkMax.MotorType.kBrushless)
      # self.sparkMaxEncoderRight = rev.CANSparkMax(53, rev.CANSparkMax.MotorType.kBrushless)
      #create navX to my robot
      self.navx = AHRS.create_spi()
      # Configure the encoder
      self.encoderLeft = self.sparkMaxEncoderLeft.getEncoder()
      # self.encoderRight = self.sparkMaxEncoderRight.getEncoder()
      # self.encoderLeft.setPositionConversionFactor(1)  # Configure for units of revolutions
      # self.encoderLeft.setVelocityConversionFactor(1)  # Configure for units of revolutions per minute
      # self.encoderRight.setPositionConversionFactor(1)  # Configure for units of revolutions
      # self.encoderRight.setVelocityConversionFactor(1)  # Configure for units of revolutions per minute
      leftPosition = self.encoderLeft.getPosition()
      rotation = wpimath.geometry._geometry.Rotation2d(self.navx.getAngle())
      inititalPose = wpimath.geometry._geometry.Pose2d(0,0,rotation)
      self.odometry = wpimath.kinematics.DifferentialDriveOdometry(
          rotation, leftPosition, leftPosition, inititalPose
      )
    def teleopInit(self):
        pass
    def autonomousPeriodic(self):
        # variable responsible for the get yaw angle
        yaw = self.navx.getAngle()
        # variables to positon and velocity of robot
        position = self.encoderLeft.getPosition()
        velocity = self.encoderLeft.getVelocity()
        
        rotation2D = wpimath.geometry._geometry.Rotation2d(self.navx.getAngle())

        self.odometry.update(rotation2D,3,3)
        print(self.odometry.getPose())
        # print(yaw)
        print(f"Position: {position} revolutions")
        # print(f"Velocity: {velocity} RPM")
        
    def teleopPeriodic(self):
        pass
 
if __name__ == "__main__":
    wpilib.run(MyRobot)