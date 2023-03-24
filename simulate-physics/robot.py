import hal.simulation
from pyfrc.physics import drivetrains
from pyfrc.physics.core import PhysicsInterface

class PhysicsEngine:
	def __init__(self, physics_controller : PhysicsInterface):
		self.physics_controller = physics_controller
        self.m_left_front = ctre.WPI_VictorSPX(constants.C_M_LEFT_FRONT)
        self.m_right_front = ctre.WPI_VictorSPX(constants.C_M_RIGHT_FRONT)
        self.m_left_back = ctre.WPI_VictorSPX(constants.C_M_LEFT_BACK)
        self.m_right_back = ctre.WPI_VictorSPX(constants.C_M_RIGHT_BACK)

        self.m_left = wpilib.SpeedControllerGroup(self.m_left_front, self.m_left_back)
        self.m_right = wpilib.SpeedControllerGroup(self.m_right_front, self.m_right_back)

        self.robot_drive = wpilib..DifferentialDrive(self.m_left, self.m_right)
        self.robot_drive.setExpiration(0.1)
