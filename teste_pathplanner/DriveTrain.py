import wpilib
import rev
import wpilib.drive
import math
from commands2 import Subsystem
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPLTVController
from pathplannerlib.config import RobotConfig
from navx import AHRS
from wpilib import DriverStation
from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.kinematics import ChassisSpeeds
from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.kinematics import DifferentialDriveWheelSpeeds
from wpimath.kinematics import DifferentialDriveOdometry
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import inchesToMeters
from wpimath.units import inchesToMeters


class DriveSubsystem(Subsystem):
    def __init__(self):

        #Configuração dos motores da tração *Necessário rever o id dos motores
        self.motor1 = rev.SparkMax(51, rev.SparkLowLevel.MotorType.kBrushless)
        self.motor2 = rev.SparkMax(52, rev.SparkLowLevel.MotorType.kBrushless)
        self.motor3 = rev.SparkMax(53, rev.SparkLowLevel.MotorType.kBrushless)
        self.motor4 = rev.SparkMax(54, rev.SparkLowLevel.MotorType.kBrushless)

        #Define o tamanho da correia, A circunferência da roda e a relação engrenagem
        self.kinematics = DifferentialDriveKinematics(inchesToMeters(25.78))
        self.wheelCircunference = inchesToMeters(6) * math.pi
        self.gearRatio = 10.7

        #Cria a navx e reseta
        self.navx = AHRS.create_spi()
        self.navx.reset()

        #Configuração do PID dos sparkmax
        config = rev.SparkMaxConfig()
        config.closedLoop.maxOutput(0.5)
        config.closedLoop.minOutput(-0.5)
        config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
        config.closedLoop.pid(0.000005,0,0)
        config.closedLoop.velocityFF(0.15)

        #Ajusta o getPosition do encoder para ser em metros
        config.encoder.positionConversionFactor(self.wheelCircunference / self.gearRatio)

        #Aplica as configurações aos motores
        self.motor1.configure(config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.motor2.configure(config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.motor3.configure(config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.motor4.configure(config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        #Cria os controladores de loop dos sparkmax
        self.rightClosedLoop = self.motor1.getClosedLoopController()
        self.right2ClosedLoop = self.motor2.getClosedLoopController()
        self.leftClosedLoop = self.motor3.getClosedLoopController()
        self.left2ClosedLoop = self.motor4.getClosedLoopController()

        #Um encoder de cada lado para o controle
        self.leftEncoder = self.motor1.getEncoder()
        self.rightEncoder = self.motor3.getEncoder()

        #Zera os encoders
        self.leftEncoder.setPosition(0)
        self.rightEncoder.setPosition(0)

        #Unificação dos motores da esquerda e da direita
        self.leftMotor = wpilib.MotorControllerGroup(self.motor1,self.motor2)
        self.rightMotor = wpilib.MotorControllerGroup(self.motor3,self.motor4)

        # Criação do diferential drive
        self.differentialDrive = wpilib.drive.DifferentialDrive(self.leftMotor,self.rightMotor)

        #Armazena a config feita no pathplanner
        pathConfig = RobotConfig.fromGUISettings()
        
        self.fictionalEncoderRight = 0
        self.fictionalEncoderLeft = 0
        self.fictionalNavx = 0
        
        #Cria a rotação do robo
        #self.rotation = Rotation2d.fromDegrees(self.navx.getAngle())

        #Cria a rotação para simulação
        self.rotation = Rotation2d.fromDegrees(self.fictionalNavx)

        #Cria a posição inicial do robo
        self.pose = Pose2d(2,7, self.rotation)

        #Cria a odometria usando rotação e posição
        self.odometry = DifferentialDriveOdometry(
            self.rotation, 
            self.leftEncoder.getPosition(), 
            self.rightEncoder.getPosition(), 
            self.pose
        )

        #Configura o autoBuilder
        AutoBuilder.configure(
            self.getPose, # Robot pose supplier
            self.resetPose, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speeds, feedforwards: self.driveRobotRelative(speeds), # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPLTVController(0.02), # PPLTVController is the built in path following controller for differential drive trains
            pathConfig, # The robot configurations
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )

    #Define se está no lado vermelho ou azul
    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        #return DriverStation.getAlliance() == DriverStation.Alliance.kRed
        return False
    #Envia a posição atual do robo
    def getPose(self):
        return self.odometry.getPose()

    #Reseta posição do robo caso estiver ativado no pathPlanner
    def resetPose(self):
        self.odometry.resetPosition(
            self.rotation,
            self.motor1.getEncoder().getPosition(),
            self.motor3.getEncoder().getPosition(),
            self.getPose())
        
    #Comando do robo através do pathPLanner
    def driveRobotRelative(self, chassiSpeed):
        wheelSpeeds = self.kinematics.toWheelSpeeds(chassiSpeed)
        
        leftRpm = (wheelSpeeds.left / self.wheelCircunference) * 60
        rightRpm = (wheelSpeeds.right / self.wheelCircunference) * 60

        leftMotorRpm = leftRpm * self.gearRatio
        rightMotorRpm = rightRpm * self.gearRatio

        self.leftClosedLoop.setReference(leftMotorRpm, rev.SparkMax.ControlType.kVelocity)
        self.left2ClosedLoop.setReference(leftMotorRpm, rev.SparkMax.ControlType.kVelocity)
        self.rightClosedLoop.setReference(rightMotorRpm, rev.SparkMax.ControlType.kVelocity)
        self.right2ClosedLoop.setReference(rightMotorRpm, rev.SparkMax.ControlType.kVelocity)

    #Retorna a velocidade das rodos
    def getRobotRelativeSpeeds(self):

        leftSpeed = self.getLeftVelocityMetersPerSecond()
        rightSpeed = self.getRightVelocityMetersPerSecond()

        wheelSpeeds = DifferentialDriveWheelSpeeds(leftSpeed,rightSpeed)

        return self.kinematics.toChassisSpeeds(wheelSpeeds)

    #Retorna a velocidade das rodas em m/s
    def getLeftVelocityMetersPerSecond(self):
        
        rpm = self.motor1.getEncoder().getVelocity() / self.gearRatio

        return (rpm / 60) * self.wheelCircunference

    def getRightVelocityMetersPerSecond(self):
        
        rpm = self.motor3.getEncoder().getVelocity() / self.gearRatio

        return (rpm / 60) * self.wheelCircunference
    
    #Atualiza a odometria
    def odometryUpdate(self):
        self.odometry.update(
            self.rotation,
            self.motor1.getEncoder().getPosition(),
            self.motor3.getEncoder().getPosition(),
        )

    #Mantém a odometria sendo atualizada
    def periodic(self):
        self.odometry.update(
            self.rotation,
            self.motor1.getEncoder().getPosition(),
            self.motor3.getEncoder().getPosition(),
        )


        #lógica para a simualção de movimento
        if self.rightClosedLoop.getSetpoint() > 0.0:
            self.fictionalEncoderRight += 0.05
            self.rightEncoder.setPosition(self.fictionalEncoderRight)
        elif self.rightClosedLoop.getSetpoint() < 0.0:
            self.fictionalEncoderRight -= 0.05
            self.rightEncoder.setPosition(self.fictionalEncoderRight)
        
        if self.leftClosedLoop.getSetpoint() > 0.0:
            self.fictionalEncoderLeft += 0.05
            self.leftEncoder.setPosition(self.fictionalEncoderLeft)
        elif self.leftClosedLoop.getSetpoint() < 0.0:
            self.fictionalEncoderLeft -= 0.05
            self.leftEncoder.setPosition(self.fictionalEncoderLeft)

        #lógica para a simulação da rotação
        rotateRight = (self.rightClosedLoop.getSetpoint() > self.leftClosedLoop.getSetpoint())
        rotateLeft = (self.leftClosedLoop.getSetpoint() > self.rightClosedLoop.getSetpoint())
        self.simulateNavx(rotateRight, rotateLeft)

    def simulateNavx(self, rotateRight: bool, rotateLeft: bool):
        if self.fictionalNavx < 0:
            self.fictionalNavx = 359.999
        elif self.fictionalNavx > 359.999:
            self.fictionalNavx = 0
        else:
            if rotateRight:
                self.fictionalNavx += 1
            if rotateLeft:
                self.fictionalNavx -= 1
        self.rotation = Rotation2d.fromDegrees(self.fictionalNavx)

        
        
    