import wpilib
import wpilib.drive
import ctre
from navx import AHRS


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
            "pitch": self.get_pitch_angle(),
            "roll": self.get_roll_angle(),
            "yall": self.get_yaw_angle(),
        }

    def robotInit(self):
        """Robot initialization function"""
        self.navx = AHRS.create_spi()

    def teleopInit(self):
        pass

    def autonomousPeriodic(self):
        yaw = self.navx.getAngle()
        print(yaw)

    def teleopPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
