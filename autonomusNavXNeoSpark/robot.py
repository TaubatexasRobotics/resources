import wpilib
import wpilib.drive
import ctre
import rev
from navx import AHRS
import wpilib
class MyRobot(wpilib.TimedRobot):
    def get_yaw_angle(self):
        try:
            # Create an instance of the AHRS class
            navx = AHRS.create_spi()
            # Get the current yaw angle
            yaw = navx.getAngle()
            # Print the yaw angle
            # print("Yaw angle: ", yaw)
            return yaw
        except Exception as e:
            print(e)
            
    def get_pitch_angle(self):
        try:
            # Create an instance of the AHRS class
            navx = AHRS.create_spi()
            # Get the current pitch angle
            pitch = navx.getPitch()
            # Print the pitch angle
            # print("Pitch angle: ", pitch)
            return pitch
        except Exception as e:
            print(e)

    def get_roll_angle(self):
        try:
            # Create an instance of the AHRS class
            navx = AHRS.create_spi()
            # Get the current roll angle
            roll = navx.getRoll()
            # Print the roll angle
            # print("Roll angle: ", roll)
            return roll
        except Exception as e:
            print(e)

    def get_angles(self):
        return {
            'pitch': self.get_pitch_angle(),
            'roll': self.get_roll_angle(),
            'yall': self.get_yaw_angle()
        }
    def robotInit(self):
      """Robot initialization function"""
      self.motor = rev.CANSparkMax(52, rev.CANSparkMax.MotorType.kBrushless)
      #create navX to my robot
      self.navx = AHRS.create_spi()
      # Configure the encoder
      self.encoder = self.motor.getEncoder()
      self.encoder.setPositionConversionFactor(1)  # Configure for units of revolutions
      self.encoder.setVelocityConversionFactor(1)  # Configure for units of revolutions per minute
 
    def teleopInit(self):
        pass
    def autonomousPeriodic(self):
        # variable responsible for the get yaw angle
        yaw = self.navx.getAngle()
        # variables to positon and velocity of robot
        position = self.encoder.getPosition()
        velocity = self.encoder.getVelocity()

        print(yaw)
        print(f"Position: {position} revolutions")
        print(f"Velocity: {velocity} RPM")
        
    def teleopPeriodic(self):
        pass
 
if __name__ == "__main__":
    wpilib.run(MyRobot)