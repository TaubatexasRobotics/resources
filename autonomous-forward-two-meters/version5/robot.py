import wpilib
from wpilib import Encoder
from navx import AHRS
import phoenix5


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Initialize the drivetrain motors and encoders
        self.front_left_motor = phoenix5.VictorSPX(0)
        self.front_right_motor = phoenix5.VictorSPX(1)
        self.back_left_motor = phoenix5.VictorSPX(2)
        self.back_right_motor = phoenix5.VictorSPX(3)

        self.left_encoder = wpilib.Encoder(0, 1)
        self.right_encoder = wpilib.Encoder(2, 3)

        # Set encoder distance per pulse and direction
        self.left_encoder.setDistancePerPulse(0.01)
        self.right_encoder.setDistancePerPulse(0.01)
        self.left_encoder.setReverseDirection(True)
        self.right_encoder.setReverseDirection(False)

        # Initialize the navX
        self.navx = AHRS.create_spi()

    def autonomousInit(self):
        # Reset the encoders and navX
        self.left_encoder.reset()
        self.right_encoder.reset()
        self.navx.reset()

    def autonomousPeriodic(self):
        # Drive forward for 2 meters
        distance_to_drive = 2.0 / (
            6 * 3.14
        )  # distance in meters / (wheel diameter * pi)
        while (
            self.left_encoder.getDistance() < distance_to_drive
            and self.right_encoder.getDistance() < distance_to_drive
        ):
            self.front_left_motor.set(0.5)
            self.front_right_motor.set(-0.5)
            self.back_left_motor.set(0.5)
            self.back_right_motor.set(-0.5)
            wpilib.Timer.delay(0.01)
        self.front_left_motor.set(0)
        self.front_right_motor.set(0)
        self.back_left_motor.set(0)
        self.back_right_motor.set(0)

        # Turn left 45 degrees
        angle_to_turn = 45
        while self.navx.getAngle() < angle_to_turn:
            self.front_left_motor.set(-0.5)
            self.front_right_motor.set(-0.5)
            self.back_left_motor.set(-0.5)
            self.back_right_motor.set(-0.5)
            wpilib.Timer.delay(0.01)
        self.front_left_motor.set(0)
        self.front_right_motor.set(0)
        self.back_left_motor.set(0)
        self.back_right_motor.set(0)

        # Drive forward for 3 meters
        distance_to_drive = 3.0 / (
            6 * 3.14
        )  # distance in meters / (wheel diameter * pi)
        while (
            self.left_encoder.getDistance() < distance_to_drive
            and self.right_encoder.getDistance() < distance_to_drive
        ):
            self.front_left_motor.set(0.5)
            self.front_right_motor.set(-0.5)
            self.back_left_motor.set(0.5)
            self.back_right_motor.set(-0.5)
            wpilib.Timer.delay(0.01)
        self.front_left_motor.set(0)
        self.front_right_motor.set(0)
        self.back_left_motor.set(0)
        self.back_right_motor.set(0)


if __name__ == "__main__":
    wpilib.run(MyRobot)
