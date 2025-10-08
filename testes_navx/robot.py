#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file        robot.py
@brief       Teste do sensor NavX com RobotPy.
@details     Script simples para validar a comunicação e leitura do sensor NavX-MXP/Micro
             utilizando a biblioteca robotpy-navx. Implementa:
             - Inicialização do sensor AHRS via interface SPI.
             - Tratamento de exceção caso o sensor não esteja conectado.
             - Exibição contínua dos valores de Yaw, Pitch e Roll no console.       
             Este código é voltado para testes, servindo para verificar se o módulo navx foi instalado corretamente 
             e se há leitura de dados válida do giroscópio. O código implementa tanto o controle de movimentação do robô 
             quanto a leitura de sensores inerciais da NavX. É adequado para testes integrados de hardware e validação de 
             comunicação com o sensor.
@note        Necessário instalar: python -m pip install robotpy-navx
@version     1.0
@date        2025-10-07
@authors     Taubatexas 7459
"""
import wpilib                      # Importa a biblioteca principal do WPILib para controle de robôs FRC
import wpilib.drive                # Importa o módulo de tração diferencial do WPILib
from wpilib import SmartDashboard  # Interface para exibir dados no SmartDashboard 
import rev                         # Importa a biblioteca para controladores de motor da REV Robotics
import navx                        # Importa a biblioteca para o sensor NavX

# Define a classe principal do robô, herdando de TimedRobot para operação em ciclos fixos
class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Inicializa o motor frontal esquerdo (Spark Max, porta 55, tipo brushed)
        self.motor_esquerda_frente = rev.SparkMax(55, rev.SparkLowLevel.MotorType.kBrushed)
        # Inicializa o motor traseiro esquerdo (Spark Max, porta 51, tipo brushed)
        self.motor_esquerda_tras = rev.SparkMax(51, rev.SparkLowLevel.MotorType.kBrushed)
        # Inicializa o motor frontal direito (Spark Max, porta 54, tipo brushed)
        self.motor_direita_frente = rev.SparkMax(54, rev.SparkLowLevel.MotorType.kBrushed)
        # Inicializa o motor traseiro direito (Spark Max, porta 53, tipo brushed)
        self.motor_direita_tras = rev.SparkMax(53, rev.SparkLowLevel.MotorType.kBrushed)

        # Agrupa os motores do lado esquerdo para controle unificado
        self.esquerda = wpilib.MotorControllerGroup(
            self.motor_esquerda_frente, self.motor_esquerda_tras
        )

        # Agrupa os motores do lado direito para controle unificado
        self.direita = wpilib.MotorControllerGroup(
            self.motor_direita_frente, self.motor_direita_tras
        )

        # Inverte a direção dos motores do lado direito para movimento correto
        self.direita.setInverted(True)

        # Configura o sistema de tração diferencial com os grupos de motores
        self.tracao = wpilib.drive.DifferentialDrive(self.esquerda, self.direita)

        # Inicializa o joystick na porta 0 para controle do robô
        self.joystick = wpilib.Joystick(0)

        # Inicializa o sensor NavX via SPI (ajuste para I2C ou USB se necessário)
        try:
            self.navx = navx.AHRS.create_spi()  # Conexão via SPI
            SmartDashboard.putString("5. NavX Status", "Conectado com sucesso!")
        except Exception as e:
            SmartDashboard.putString("5. NavX Status", f"Erro ao conectar: {str(e)}")

        # Zera o ângulo de guinada (yaw) na inicialização para referência
        self.navx.reset()

    # Método chamado uma vez ao entrar no modo teleoperado
    def teleopInit(self):
        # Ativa a segurança da tração, desativando motores se não houver comandos
        self.tracao.setSafetyEnabled(True)

    # Método executado repetidamente (a cada 20 ms) no modo teleoperado
    def teleopPeriodic(self):
        # Controla o robô no modo arcade: eixo Y (frente/trás) e eixo X (rotação)
        self.tracao.arcadeDrive(-self.joystick.getRawAxis(1), self.joystick.getRawAxis(4))

        # Lê e exibe os dados da NavX no SmartDashboard
        if hasattr(self, 'navx') and self.navx.isConnected():
            # Orientação (ângulos em graus)
            SmartDashboard.putNumber("1. NavX Pitch", self.navx.getPitch()) # Inclinação
            SmartDashboard.putNumber("2. NavX Roll", self.navx.getRoll())   # Rolagem
            SmartDashboard.putNumber("3. NavX Yaw", self.navx.getYaw())     # Guinada (-180 a 180)
            SmartDashboard.putNumber("4. NavX Angle", self.navx.getAngle()) # Ângulo acumulado (contínuo)
        else:
            # Caso a NavX não esteja conectada, exibe mensagem de erro
            SmartDashboard.putString("5. NavX Status", "Desconectado ou não inicializado")

# Outros parâmetros NavX:

#SmartDashboard.putNumber("NavX Compass Heading", self.navx.getCompassHeading()) # Direção magnética (0 a 360)
#SmartDashboard.putNumber("NavX Fused Heading", self.navx.getFusedHeading())     # Direção combinada (0 a 360)
#SmartDashboard.putNumber("NavX Velocity X", self.navx.getVelocityX())           # Velocidade estimada X
#SmartDashboard.putNumber("NavX Velocity Y", self.navx.getVelocityY())           # Velocidade estimada Y
#SmartDashboard.putNumber("NavX Velocity Z", self.navx.getVelocityZ())           # Velocidade estimada Z
#SmartDashboard.putNumber("NavX Displacement X", self.navx.getDisplacementX())   # Deslocamento X
#SmartDashboard.putNumber("NavX Displacement Y", self.navx.getDisplacementY())   # Deslocamento Y
#SmartDashboard.putNumber("NavX Displacement Z", self.navx.getDisplacementZ())   # Deslocamento Z
            
# Status do sensor
#SmartDashboard.putBoolean("NavX Is Calibrating", self.navx.isCalibrating())                        # Calibrando?
#SmartDashboard.putBoolean("NavX Is Magnetometer Calibrated", self.navx.isMagnetometerCalibrated()) # Magnetômetro calibrado?
#SmartDashboard.putBoolean("NavX Is Moving", self.navx.isMoving())                                  # Em movimento?
#SmartDashboard.putBoolean("NavX Is Rotating", self.navx.isRotating())                              # Em rotação?
#SmartDashboard.putBoolean("NavX Is Altitude Valid", self.navx.isAltitudeValid())                   # Altitude válida?
#SmartDashboard.putBoolean("NavX Is Magnetic Disturbance", self.navx.isMagneticDisturbance())       # Distúrbio magnético?

# Taxas angulares (graus por segundo)
#SmartDashboard.putNumber("NavX Rate", self.navx.getRate())           # Taxa de guinada
#SmartDashboard.putNumber("NavX Raw Gyro X", self.navx.getRawGyroX()) # Giroscópio bruto X
#SmartDashboard.putNumber("NavX Raw Gyro Y", self.navx.getRawGyroY()) # Giroscópio bruto Y
#SmartDashboard.putNumber("NavX Raw Gyro Z", self.navx.getRawGyroZ()) # Giroscópio bruto Z

# Acelerações (em g)
#SmartDashboard.putNumber("NavX Accel X", self.navx.getWorldLinearAccelX()) # Aceleração linear X
#SmartDashboard.putNumber("NavX Accel Y", self.navx.getWorldLinearAccelY()) # Aceleração linear Y
#SmartDashboard.putNumber("NavX Accel Z", self.navx.getWorldLinearAccelZ()) # Aceleração linear Z
#SmartDashboard.putNumber("NavX Raw Accel X", self.navx.getRawAccelX())     # Aceleração bruta X
#SmartDashboard.putNumber("NavX Raw Accel Y", self.navx.getRawAccelY())     # Aceleração bruta Y
#SmartDashboard.putNumber("NavX Raw Accel Z", self.navx.getRawAccelZ())     # Aceleração bruta Z

# Magnetômetro (em micro-Tesla)
#SmartDashboard.putNumber("NavX Raw Mag X", self.navx.getRawMagX()) # Campo magnético X
#SmartDashboard.putNumber("NavX Raw Mag Y", self.navx.getRawMagY()) # Campo magnético Y
#SmartDashboard.putNumber("NavX Raw Mag Z", self.navx.getRawMagZ()) # Campo magnético Z

# Outros dados
#SmartDashboard.putNumber("NavX Temperature", self.navx.getTempC()) # Temperatura em °C
#SmartDashboard.putNumber("NavX Altitude", self.navx.getAltitude()) # Altitude (se configurado)
#SmartDashboard.putNumber("NavX Pressure", self.navx.getPressure()) # Pressão barométrica

# Informações de fusão de sensores
#SmartDashboard.putNumber("NavX Quaternion W", self.navx.getQuaternionW()) # Quaternions
#SmartDashboard.putNumber("NavX Quaternion X", self.navx.getQuaternionX())
#SmartDashboard.putNumber("NavX Quaternion Y", self.navx.getQuaternionY())
#SmartDashboard.putNumber("NavX Quaternion Z", self.navx.getQuaternionZ())

# Contagem de atualizações e frequência
#SmartDashboard.putNumber("NavX Update Count", self.navx.getUpdateCount())        # Contagem de atualizações
#SmartDashboard.putNumber("NavX Update Rate Hz", self.navx.getActualUpdateRate()) # Taxa de atualização