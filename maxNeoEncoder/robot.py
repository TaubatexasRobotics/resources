
import wpilib
import wpilib.drive
import ctre
import rev
from navx import AHRS

class MyRobot(wpilib.TimedRobot):
    
    def robotInit(self):
      """Robot initialization function"""
      self.motor = rev.CANSparkMax(52, rev.CANSparkMax.MotorType.kBrushless)

      # Configure the encoder
      self.encoder = self.motor.getEncoder()
      self.encoder.setPositionConversionFactor(1)  # Configure for units of revolutions
      self.encoder.setVelocityConversionFactor(1)  # Configure for units of revolutions per minute
 
    def teleopInit(self):
        pass
    def autonomousPeriodic(self):
        position = self.encoder.getPosition()
        velocity = self.encoder.getVelocity()

        print(f"Position: {position} revolutions")
        print(f"Velocity: {velocity} RPM")
        
    def teleopPeriodic(self):
        pass
 
if __name__ == "__main__":
    wpilib.run(MyRobot)