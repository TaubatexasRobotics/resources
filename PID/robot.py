
import wpilib
import wpilib.drive
import ctre
# import networktables_project as nt
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
            'pitch': self.get_pitch_angle(),
            'roll': self.get_roll_angle(),
            'yall': self.get_yaw_angle()
        }

    def robotInit(self):
        """Robot initialization function"""
        self.navx = AHRS.create_spi()
        # self.get_pitch_angle()
        # motor controllers for traction
        self.m_left_front = ctre.WPI_VictorSPX(22)
        self.m_right_front = ctre.WPI_VictorSPX(33)
        self.m_left_rear = ctre.WPI_VictorSPX(11)
        self.m_right_rear = ctre.WPI_VictorSPX(44)

        # self.shooter = ctre.WPI_VictorSPX(9)
        # self.track_ball = ctre.WPI_VictorSPX(8)
        # self.ball_catcher = ctre.WPI_VictorSPX(55)

        self.m_right = wpilib.SpeedControllerGroup(self.m_right_front, self.m_right_rear)
        self.m_left = wpilib.SpeedControllerGroup(self.m_left_front, self.m_left_rear)
	
        # object that handles basic drive operations
        # self.myRobot = wpilib.drive.DifferentialDrive(self.m_left, self.m_right)
        # self.myRobot.setExpiration(0.1)

        # joystick 0
        # self.stick = wpilib.Joystick(0)

        # init camera
        # wpilib.CameraServer.launch('vision.py:main')

        # create timer
        # self.timer = wpilib.Timer()

    def teleopInit(self):
        pass
    # Executed at the start of teleop mode

    def autonomousPeriodic(self):
      yaw = self.navx.getAngle()
      # angles = self.get_angles()
      # print(angles['yaw'])
      print(yaw)
      error = 0 - yaw 
      velocidade = 0.01 * error 
      if (yaw > 0):
        self.m_left(velocidade)
      elif (yaw < 0):
        self.m_right(-velocidade)
      self.m_left(0.2)
      self.m_right(0.2)

    def teleopPeriodic(self):
        pass
    #Runs the motors with tank steering
    # to invert the axis when robot turns back
	# quadrado - pegar e subir
	# x - chutar
	# r1 - descer
	# o - desprender a bola

if __name__ == "__main__":
    wpilib.run(MyRobot)
import wpilib
import wpilib.drive
import ctre
# import networktables_project as nt
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
            'pitch': self.get_pitch_angle(),
            'roll': self.get_roll_angle(),
            'yall': self.get_yaw_angle()
        }

    def robotInit(self):
        """Robot initialization function"""
        self.navx = AHRS.create_spi()
        # self.get_pitch_angle()
        # motor controllers for traction
        # self.m_left_front = ctre.WPI_VictorSPX(22)
        # self.m_right_front = ctre.WPI_VictorSPX(33)
        # self.m_left_rear = ctre.WPI_VictorSPX(11)
        # self.m_right_rear = ctre.WPI_VictorSPX(44)

        # self.shooter = ctre.WPI_VictorSPX(9)
        # self.track_ball = ctre.WPI_VictorSPX(8)
        # self.ball_catcher = ctre.WPI_VictorSPX(55)

        # self.m_left = wpilib.SpeedControllerGroup(self.m_left_front, self.m_left_rear)
        # self.m_right = wpilib.SpeedControllerGroup(self.m_right_front, self.m_right_rear)
	
        # object that handles basic drive operations
        # self.myRobot = wpilib.drive.DifferentialDrive(self.m_left, self.m_right)
        # self.myRobot.setExpiration(0.1)

        # joystick 0
        # self.stick = wpilib.Joystick(0)

        # init camera
        # wpilib.CameraServer.launch('vision.py:main')

        # create timer
        # self.timer = wpilib.Timer()

    def teleopInit(self):
        pass
    # Executed at the start of teleop mode

    def autonomousPeriodic(self):
        yaw = self.navx.getAngle()
        # angles = self.get_angles()
        # print(angles['yaw'])
        print(yaw)
        
        
        # if yaw > 30:
        # # Check if yaw angle is greater than 30 degrees
        #     # Move the robot forward
        #     self.myRobot.arcadeDrive(-velocidadeT/1000, 0)
        # # Check if yaw angle is less than 0 degrees
        # elif yaw < 0:
        #     # Move the robot backward
        #     self.myRobot.arcadeDrive(velocidadeT/1000, 0)
        # else:
        #     # Stop the robot
        #     self.myRobot.arcadeDrive(0, 0)
	        

    def teleopPeriodic(self):
        pass
    #Runs the motors with tank steering
    # to invert the axis when robot turns back
	# quadrado - pegar e subir
	# x - chutar
	# r1 - descer
	# o - desprender a bola

if __name__ == "__main__":
    wpilib.run(MyRobot)     