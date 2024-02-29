import wpilib
import wpilib.drive
import ctre
import rev


class Taubatexas(wpilib.TimedRobot):
    def robotInit(self):
        # Drivetrain
        self.left_back = ctre.WPI_VictorSPX(1)
        self.left_front = ctre.WPI_VictorSPX(3)
        self.right_front = ctre.WPI_VictorSPX(4)
        self.right_back = ctre.WPI_VictorSPX(2)

        self.left = wpilib.MotorControllerGroup(self.left_front, self.left_back)
        self.right = wpilib.MotorControllerGroup(self.right_front, self.right_back)
        self.right.setInverted(True)

        self.drivetrain = wpilib.drive.DifferentialDrive(self.left, self.right)

        # Intake
        self.intake_bar_left = ctre.WPI_VictorSPX(5)
        self.intake_bar_right = ctre.WPI_VictorSPX(12)
        self.intake_bar_right.setInverted(True)
        self.intake = wpilib.MotorControllerGroup(
            self.intake_bar_left, self.intake_bar_right
        )

        # Arm
        self.length = rev.CANSparkMax(52, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.angle = rev.CANSparkMax(50, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.angle.setInverted(True)
        self.angle_pid = self.angle.getPIDController()
        self.angle_encoder = self.angle.getEncoder()
        self.angle_pid.setP(0.1)
        self.angle_pid.setI(0.0)
        self.angle_pid.setD(0.1)
        self.position = self.angle_encoder.getPosition()

        # SmartDashboard
        self.smartdashboard = wpilib.SmartDashboard
        self.smartdashboard.putNumber("Intake Left", 0.25)
        self.smartdashboard.putNumber("Intake Right", 0.25)
        self.smartdashboard.putNumber("Arm Length Speed", 1)
        self.smartdashboard.putNumber("Arm Angle Duty Cycle", 0.15)

        # Joystick
        self.joystick = wpilib.Joystick(0)

        # Timer
        self.timer = wpilib.Timer()

        # self.angle_encoder.setPosition(0)

    def robotPeriodic(self):
        self.smartdashboard.putNumber(
            "Arm Angle Encoder", self.angle_encoder.getPosition()
        )

    def getSpecificAxis(self):
        if self.joystick.getRawAxis(4) > 0:
            return self.joystick.getRawAxis(4)
        elif self.joystick.getRawAxis(3) > 0:
            return -self.joystick.getRawAxis(3)
        else:
            return 0

    def setIntake(self, left: float, right: float):
        self.intake_bar_left.set(left)
        self.intake_bar_right.set(right)

    def teleopInit(self):
        self.drivetrain.setExpiration(0.1)
        self.drivetrain.setSafetyEnabled(True)

    def teleopPeriodic(self):
        self.drivetrain.arcadeDrive(
            self.getSpecificAxis(), -self.joystick.getRawAxis(0), True
        )

        if self.joystick.getRawButton(1):
            # self.intake.set(0.5)
            self.setIntake(
                -self.smartdashboard.getNumber("Intake Left", 0.25),
                -self.smartdashboard.getNumber("Intake Right", 0.25),
            )
        elif self.joystick.getRawButton(2):
            # self.intake.set(-0.5)
            self.setIntake(
                self.smartdashboard.getNumber("Intake Left", 0.25),
                self.smartdashboard.getNumber("Intake Right", 0.25),
            )
        elif self.joystick.getRawButton(9):
            self.intake_bar_left.set(0.1)
        elif self.joystick.getRawButton(10):
            self.intake_bar_right.set(0.1)
        else:
            # self.intake.set(0)
            self.setIntake(0, 0)

        if self.joystick.getRawButton(5):
            self.length.set(self.smartdashboard.getNumber("Arm Length Speed", 1))
        elif self.joystick.getRawButton(6):
            self.length.set(-self.smartdashboard.getNumber("Arm Length Speed", 1))
        else:
            self.length.set(0)

        if self.joystick.getRawButton(4):
            self.angle_pid.setReference(
                self.smartdashboard.getNumber("Arm Angle Duty Cycle", 0.15),
                rev.CANSparkMax.ControlType.kDutyCycle,
            )
            self.position = self.angle_encoder.getPosition()
        elif self.joystick.getRawButton(3):
            self.angle_pid.setReference(
                -self.smartdashboard.getNumber("Arm Angle Duty Cycle", 0.15),
                rev.CANSparkMax.ControlType.kDutyCycle,
            )
            self.position = self.angle_encoder.getPosition()
        else:
            self.angle_pid.setReference(
                self.position, rev.CANSparkMax.ControlType.kPosition
            )

    # def disabledInit(self):
    #    self.angle_encoder.setPosition(0)
    """
    def disabledPeriodic(self):
        if(self.timer.get() < 5)
            self.angle_pid.setReference(
                self.smartdashboard.getNumber('Angle Duty Cycle', 0.15), 
                rev.CANSparkMax.ControlType.kDutyCycle
            )
        else:
            self.timer.stop()
    """


if __name__ == "__main__":
    wpilib.run(Taubatexas)
