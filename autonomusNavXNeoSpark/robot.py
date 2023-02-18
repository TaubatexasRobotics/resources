import wpilib
import wpilib.drive
import ctre
import rev
from navx import AHRS

class MyRobot(wpilib.TimedRobot):
    
    def robotInit(self):
      """Robot initialization function"""
      self.sparkMaxEncoderLeft = rev.CANSparkMax(52, rev.CANSparkMax.MotorType.kBrushless)
      #create navX to my robot
      self.navx = AHRS.create_spi()
      # Configure the encoder
      self.encoderLeft = self.sparkMaxEncoderLeft.getEncoder()
      self.encoderLeft.setPositionConversionFactor(1)  # Configure for units of revolutions
      self.encoderLeft.setVelocityConversionFactor(1)  # Configure for units of revolutions per minute
 
    def teleopInit(self):
        pass
    def autonomousPeriodic(self):
        # variable responsible for the get yaw angle
        yaw = self.navx.getAngle()
        # variables to positon and velocity of robot
        position = self.encoderLeft.getPosition()
        velocity = self.encoderLeft.getVelocity()

        print(yaw)
        print(f"Position: {position} revolutions")
        print(f"Velocity: {velocity} RPM")
        
    def teleopPeriodic(self):
        pass
 
if __name__ == "__main__":
    wpilib.run(MyRobot)