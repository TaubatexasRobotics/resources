#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file        robot.py
@brief       Teste do sensor NavX com RobotPy + Encoders AMT103
@details     Script integrado para validar comunicação NavX + Encoders.
             Implementa controle arcade, leitura inercial e odometria.
@note        Necessário instalar: python -m pip install robotpy-navx
@todo        [OSSCILAÇÃO NO GIRO] O comando TurnDegrees ainda apresenta oscilação
@version     2 (calibração de encoders realizada)
@date        2025-15-11
@authors     Taubatexas 7459
"""
# === BIBLIOTECAS IMPORTADAS ===
import wpilib  # Biblioteca principal do WPILib (FRC)
import wpilib.drive  # Módulo para controle de tração diferencial
from wpilib import SmartDashboard  # Painel de depuração em tempo real
import phoenix5  # Drivers oficiais CTRE para motores Victor SPX/Talon
import navx  # Biblioteca do sensor NavX (giroscópio + acelerômetro)
import math  # Funções matemáticas (π, etc.)
import commands2  # Framework de comandos (substitui o antigo Command)
from commands2 import Command, SequentialCommandGroup  # Comandos base e sequências

# ==============================
# == CONSTANTES DE CALIBRAÇÃO ==
# ==============================

kEncoderPPR = 2048.0  # Pulsos por rotação do encoder magnético CTRE (padrão: 2048)
kWheelDiameter = 0.152  # Diâmetro da roda em metros (6 polegadas = 0.152m)
fator_correcao = 3.06  # Fator empírico ajustado em campo no dia 15/11/2025
# → Compensação por: atrito, peso do robô, bateria, piso, etc.

# Cálculo da distância por pulso do encoder (em metros)
kEncoderDistancePerPulse = (kWheelDiameter * math.pi) / kEncoderPPR * fator_correcao
# Explicação:
# - (π × diâmetro) = circunferência da roda
# - / kEncoderPPR = metros por pulso (teórico)
# - * fator_correcao = ajuste real (calibrado em campo)
# → OBS: o "4" foi REMOVIDO porque a biblioteca wpilib.Encoder já considera
#     a contagem em modo quadrature (4 pulsos por ciclo) → duplicaria a distância!

kDefaultSpeed = 0.5  # Velocidade padrão para andar (50% do máximo) - evita derrapagem
kDefaultTurnSpeed = 0.5  # Velocidade padrão para girar - reduzido para suavidade
kDistanceTolerance = (
    0.02  # Tolerância de 2 cm → robô para quando chegar perto o suficiente
)
kAngleTolerance = 10.0  # Tolerância de 10° → AUMENTADA para evitar oscilação no giro


# ==============================
# === COMANDO: ANDAR X METROS ===
# ==============================
class DriveDistance(Command):
    """
    Comando autônomo: faz o robô andar uma distância específica em linha reta.
    Usa encoders para medir a distância percorrida.
    """

    # Variáveis estáticas compartilhadas entre instâncias (injetadas no robotInit)
    tracao = None  # Sistema de tração (DifferentialDrive)
    left_encoder = None  # Encoder esquerdo
    right_encoder = None  # Encoder direito

    def __init__(self, distance_meters, speed=kDefaultSpeed):
        super().__init__()  # Inicializa o comando base
        self.target_distance = distance_meters  # Distância alvo (em metros)
        self.speed = speed  # Velocidade desejada
        self.start_distance = 0.0  # Distância inicial (média dos encoders)
        self.addRequirements(self.tracao)  # Bloqueia a tração para outros comandos

    def initialize(self):
        """Executado uma vez ao iniciar o comando"""
        # Lê a posição inicial dos dois encoders
        left = self.left_encoder.getDistance()
        right = self.right_encoder.getDistance()
        self.start_distance = (left + right) / 2.0  # Média → compensa desalinhamento

        # Atualiza o SmartDashboard para depuração
        SmartDashboard.putString(
            "Auto Status", f"Iniciando: andar {self.target_distance:.2f}m"
        )
        SmartDashboard.putNumber("Auto Start Dist (m)", self.start_distance)

    def execute(self):
        """Executado a cada ~20ms (50Hz) durante o comando"""
        # Lê distância atual dos encoders
        left = self.left_encoder.getDistance()
        right = self.right_encoder.getDistance()
        avg = (left + right) / 2.0
        traveled = avg - self.start_distance  # Distância já percorrida
        error = self.target_distance - traveled  # Quanto falta?

        # Controle bang-bang simples (liga/desliga)
        if abs(error) < kDistanceTolerance:  # Chegou perto o suficiente?
            output = 0.0  # Para
        else:
            output = self.speed if error > 0 else -self.speed  # Anda pra frente ou trás

        # Aplica movimento (frente/trás, sem giro)
        self.tracao.arcadeDrive(output, 0)
        self.tracao.feed()  # Necessário para o watchdog de segurança

        # === DEPURAÇÃO NO SMARTDASHBOARD ===
        SmartDashboard.putNumber("Auto Traveled (m)", traveled)
        SmartDashboard.putNumber("Auto Distance Error (m)", error)
        SmartDashboard.putNumber(
            "Left Raw Auto", self.left_encoder.getRaw()
        )  # Valor bruto (pulsos)

    def isFinished(self):
        """Retorna True quando o comando deve terminar"""
        left = self.left_encoder.getDistance()
        right = self.right_encoder.getDistance()
        avg = (left + right) / 2.0
        traveled = avg - self.start_distance
        return abs(self.target_distance - traveled) <= kDistanceTolerance

    def end(self, interrupted):
        """Executado ao finalizar (com ou sem interrupção)"""
        self.tracao.stopMotor()  # Para os motores
        final = (
            self.left_encoder.getDistance() + self.right_encoder.getDistance()
        ) / 2.0 - self.start_distance
        status = f"Finalizado: {final:.3f}m" if not interrupted else "Interrompido"
        SmartDashboard.putString("Auto Status", status)


# ==============================
# === COMANDO: GIRAR X GRAUS ===
# ==============================
class TurnDegrees(Command):
    """
    Comando autônomo: gira o robô um ângulo específico usando o NavX.
    Usa controle com tolerância e contagem de estabilidade.
    """

    tracao = None  # Sistema de tração
    navx = None  # Sensor NavX

    def __init__(self, degrees, speed=kDefaultTurnSpeed):
        super().__init__()
        self.target_angle = degrees  # Ângulo alvo (em graus)
        self.speed = speed  # Velocidade de giro
        self.start_angle = 0.0  # Ângulo inicial
        self.stable_count = 0  # Contador de ciclos estáveis
        self.required_stable = 6  # Precisa de 6 ciclos (~120ms) dentro da tolerância
        self.addRequirements(self.tracao)

    def initialize(self):
        """Captura o ângulo inicial do NavX"""
        self.start_angle = self.navx.getAngle()
        self.stable_count = 0
        SmartDashboard.putString(
            "Auto Status", f"Iniciando: girar {self.target_angle}°"
        )
        SmartDashboard.putNumber("Auto Start Angle", self.start_angle)

    def execute(self):
        """Executado a cada ciclo"""
        current = self.navx.getAngle()
        # Calcula erro relativo (quanto falta girar)
        error = self.target_angle - (current - self.start_angle)

        # Normaliza o erro para o intervalo [-180, 180]
        # → Evita giros longos (ex: 270° vira -90°)
        while error > 180:
            error -= 360
        while error < -180:
            error += 360

        # Controle com histerese + estabilidade
        if abs(error) < kAngleTolerance:
            output = 0.0
            self.stable_count += 1  # Conta ciclos dentro da tolerância
        else:
            output = self.speed if error > 0 else -self.speed
            self.stable_count = 0  # Reseta se saiu da zona

        self.tracao.arcadeDrive(0, output)  # Gira no lugar
        self.tracao.feed()

        # Depuração
        SmartDashboard.putNumber("Auto Angle Error (°)", error)
        SmartDashboard.putNumber("Stable Count", self.stable_count)

    def isFinished(self):
        """Só termina se estiver dentro da tolerância POR VÁRIOS CICLOS"""
        current = self.navx.getAngle()
        error = self.target_angle - (current - self.start_angle)
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return (
            abs(error) <= kAngleTolerance and self.stable_count >= self.required_stable
        )

    def end(self, interrupted):
        self.tracao.stopMotor()
        final = self.navx.getAngle() - self.start_angle
        status = f"Giro: {final:.1f}°" if not interrupted else "Giro interrompido"
        SmartDashboard.putString("Auto Status", status)


# ==============================
# === CLASSE PRINCIPAL DO ROBÔ ===
# ==============================
class MyRobot(wpilib.TimedRobot):
    """Classe principal do robô - herda de TimedRobot (executa em ciclos de 20ms)"""

    def robotInit(self):
        """Inicialização única do robô (ao ligar)"""

        # === MOTORES (Victor SPX via CAN) ===
        self.motor_esquerda_frente = phoenix5.WPI_VictorSPX(2)  # ID CAN: 2
        self.motor_esquerda_tras = phoenix5.WPI_VictorSPX(1)  # ID CAN: 1
        self.motor_direita_frente = phoenix5.WPI_VictorSPX(3)  # ID CAN: 3
        self.motor_direita_tras = phoenix5.WPI_VictorSPX(4)  # ID CAN: 4

        # Agrupa motores do mesmo lado
        self.esquerda = wpilib.MotorControllerGroup(
            self.motor_esquerda_frente, self.motor_esquerda_tras
        )
        self.direita = wpilib.MotorControllerGroup(
            self.motor_direita_frente, self.motor_direita_tras
        )
        self.direita.setInverted(
            True
        )  # Inverte lado direito (padrão em tração diferencial)

        # Cria sistema de tração diferencial (arcade drive)
        self.tracao = wpilib.drive.DifferentialDrive(self.esquerda, self.direita)

        # === SENSORES ===
        self.joystick = wpilib.Joystick(0)  # Joystick na porta USB 0

        # Inicializa NavX via SPI (com tratamento de erro)
        try:
            self.navx = navx.AHRS.create_spi()
            SmartDashboard.putString("NavX Status", "Conectado!")
        except:
            SmartDashboard.putString("NavX Status", "FALHA")

        # === ENCODERS (canais DIO) ===
        self.left_encoder = wpilib.Encoder(5, 6)  # Canais DIO 5 e 6
        self.right_encoder = wpilib.Encoder(7, 8, False)  # Canal 7 e 8, sem inversão
        self.left_encoder.setDistancePerPulse(kEncoderDistancePerPulse)
        self.right_encoder.setDistancePerPulse(kEncoderDistancePerPulse)

        # === INJEÇÃO DE DEPENDÊNCIAS NOS COMANDOS ===
        # → Como os comandos são classes, usamos variáveis de classe para compartilhar
        DriveDistance.tracao = self.tracao
        DriveDistance.left_encoder = self.left_encoder
        DriveDistance.right_encoder = self.right_encoder

        TurnDegrees.tracao = self.tracao
        TurnDegrees.navx = self.navx

        # === SELETOR DE AUTÔNOMO (SmartDashboard) ===
        self.auto_chooser = wpilib.SendableChooser()
        self.auto_chooser.setDefaultOption("Parado", None)
        self.auto_chooser.addOption("Andar 50 cm", DriveDistance(0.5))
        self.auto_chooser.addOption("Andar 1 metro", DriveDistance(1.0))
        self.auto_chooser.addOption("Girar 90°", TurnDegrees(90))
        self.auto_chooser.addOption("Girar -90°", TurnDegrees(-90))
        self.auto_chooser.addOption("Teste completo", self.getTestSequence())
        SmartDashboard.putData("Modo Autônomo", self.auto_chooser)

    def getTestSequence(self):
        """Sequência de teste"""
        return SequentialCommandGroup(
            DriveDistance(1.0),  # Comando 1
            TurnDegrees(90),  # Comando 2
            DriveDistance(1.0),  # Comando 3
            TurnDegrees(-90),  # Comando "x"
        )

    # ==============================
    # === AUTÔNOMO ===
    # ==============================
    def autonomousInit(self):
        """Inicializa o modo autônomo"""
        self.left_encoder.reset()  # Zera encoders
        self.right_encoder.reset()
        self.selected_command = (
            self.auto_chooser.getSelected()
        )  # Pega comando escolhido
        if self.selected_command:
            self.selected_command.schedule()  # Agenda no scheduler

    def autonomousPeriodic(self):
        pass  # Tudo é feito pelo CommandScheduler

    # ==============================
    # = PERIÓDICO (TODOS OS MODOS) =
    # ==============================
    def robotPeriodic(self):
        """Executado a cada ciclo, em qualquer modo"""
        self.tracao.feed()  # Alimenta o watchdog
        commands2.CommandScheduler.getInstance().run()  # Executa comandos agendados

    # ==============================
    # == TELEOP (CONTROLE MANUAL) ==
    # ==============================
    def teleopInit(self):
        """Ao entrar no modo teleoperado"""
        if hasattr(self, "selected_command") and self.selected_command:
            self.selected_command.cancel()  # Cancela autônomo
        self.tracao.setSafetyEnabled(True)  # Ativa segurança

    def teleopPeriodic(self):
        """Controle manual com joystick"""
        # Eixo Y (frente/trás), Eixo Z (giro) → arcade drive
        self.tracao.arcadeDrive(
            -self.joystick.getRawAxis(1), self.joystick.getRawAxis(4)
        )

        # === DEPURAÇÃO DO NAVX ===
        if hasattr(self, "navx") and self.navx.isConnected():
            SmartDashboard.putNumber("Yaw", self.navx.getYaw())  # -180 a 180
            SmartDashboard.putNumber("Angle", self.navx.getAngle())  # Contínuo

        # === DEPURAÇÃO DOS ENCODERS ===
        left = self.left_encoder.getDistance()
        right = self.right_encoder.getDistance()
        SmartDashboard.putNumber("Left Dist (m)", left)
        SmartDashboard.putNumber("Right Dist (m)", right)
        SmartDashboard.putNumber("Avg Dist (m)", (left + right) / 2)

    # ==============================
    # === DESABILITADO ===
    # ==============================
    def disabledPeriodic(self):
        """Quando o robô está desabilitado (útil para calibração)"""
        SmartDashboard.putNumber("Left Raw", self.left_encoder.getRaw())
        SmartDashboard.putNumber("Right Raw", self.right_encoder.getRaw())


# ==============================
# === EXECUÇÃO DO ROBÔ ===
# ==============================
if __name__ == "__main__":
    wpilib.run(MyRobot)  # Inicia o loop principal do WPILib
