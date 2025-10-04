#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file        robot.py
@brief       Simulação de robô FRC com drivetrain diferencial usando WPILib.
@details     Este script implementa uma simulação educacional de robô FRC, permitindo:
             - Controle via joystick no modo teleoperado.
             - Movimentos programados no modo autônomo.
             - Integração com SmartDashboard e Field2d para visualização da posição do robô no AdvantageScope.
             - Controle de motores REV SparkMax (brushed) em configuração diferencial.

             Utiliza uma simulação cinemática simples (SimpleKinematicSim) para cálculo de
             posições e orientações. Ideal para estudo e prática de programação de robôs FRC
             sem hardware físico.
             Link de download AdvancedScope:
             https://github.com/Mechanical-Advantage/AdvantageScope/releases/download/v4.1.6/advantagescope-wpilib-win-x64.zip
             Obs: No AdvancedScope, após adicionar o robô é necessário mudar Rotation Units para "Degrees"
@note        Sensores reais e feedback de hardware não estão implementados; apenas simulação.
@version     2.1
@date        2025-09-01
@authors     Taubatexas 7459
"""

import wpilib  # Biblioteca principal para robôs FRC
import wpilib.drive  # Controle de movimento do robô
import rev  # Controle de motores REV
import math  # Biblioteca para funções matemáticas
from wpilib import (
    SmartDashboard,
    Field2d,
)  # Ferramentas para monitoramento e visualização
from robot_sim import SimpleKinematicSim  # Importa a simulação de movimento


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Configura os motores do robô
        self.motor_esquerda_frente = rev.SparkMax(
            55, rev.SparkLowLevel.MotorType.kBrushed
        )  # Motor frente esquerda
        self.motor_esquerda_tras = rev.SparkMax(
            51, rev.SparkLowLevel.MotorType.kBrushed
        )  # Motor traseira esquerda
        self.motor_direita_frente = rev.SparkMax(
            54, rev.SparkLowLevel.MotorType.kBrushed
        )  # Motor frente direita
        self.motor_direita_tras = rev.SparkMax(
            53, rev.SparkLowLevel.MotorType.kBrushed
        )  # Motor traseira direita

        # Agrupa motores para controle conjunto
        self.esquerda = wpilib.MotorControllerGroup(
            self.motor_esquerda_frente, self.motor_esquerda_tras
        )
        self.direita = wpilib.MotorControllerGroup(
            self.motor_direita_frente, self.motor_direita_tras
        )
        self.direita.setInverted(
            True
        )  # Inverte motores à direita para girar corretamente

        # Configura o sistema de tração do robô
        self.tracao = wpilib.drive.DifferentialDrive(self.esquerda, self.direita)
        self.tracao.setSafetyEnabled(
            True
        )  # Ativa segurança para evitar comandos perigosos

        # Configura o joystick na porta 0
        self.joystick = wpilib.Joystick(0)

        # Configura o campo virtual para mostrar a posição do robô
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)  # Exibe o campo no AdvantageScope

        # Inicializa a simulação com posição inicial (exemplo: 1.5 m, 2.0 m, 0°)
        self.sim = SimpleKinematicSim(initial_x=1.5, initial_y=2.0, initial_yaw_deg=0.0)

        # Temporizador para referência em modos autônomos
        self.timer = wpilib.Timer()

        # Seletor de modos autônomos no SmartDashboard
        self.selector = wpilib.SendableChooser()
        self.selector.setDefaultOption("Parado", 0)  # Não se move
        self.selector.addOption("Avançar 1 s", 1)  # Move para frente por 1 segundo
        self.selector.addOption(
            "Avançar, Girar 90°, Avançar", 2
        )  # Move, gira -90°, move novamente
        SmartDashboard.putData("Auto Selector", self.selector)

    def teleopPeriodic(self):
        # Lê comandos do joystick
        velocidade = -self.joystick.getRawAxis(
            1
        )  # Eixo Y: frente (positivo) ou trás (negativo)
        rotacao = -self.joystick.getRawAxis(
            4
        )  # Eixo X: girar esquerda/direita (inverte para consistência)

        # Processa comandos com a simulação (ajusta velocidades e calcula nova posição)
        velocidade, rotacao, pose, orientacao = self.sim.update(velocidade, rotacao)

        # Atualiza a posição do robô no campo virtual
        self.field.setRobotPose(pose)

        # Envia comandos ao robô (move e gira)
        self.tracao.arcadeDrive(velocidade, rotacao)

        # Dados para monitoramento (descomente para ver no SmartDashboard)
        # SmartDashboard.putNumber("Velocidade", velocidade)
        # SmartDashboard.putNumber("Rotação", rotacao)
        # SmartDashboard.putNumber("Posição X (m)", pose.X())
        # SmartDashboard.putNumber("Posição Y (m)", pose.Y())
        # SmartDashboard.putNumber("Orientação (graus)", pose.rotation().degrees())

    def autonomousInit(self):
        # Inicia o temporizador e seleciona o modo autônomo
        self.autonomous_mode = self.selector.getSelected()
        self.timer.reset()
        self.timer.start()
        # Armazena a orientação inicial para referência relativa
        self.initial_heading = self.sim.heading

    def autonomousPeriodic(self):
        # Define velocidade e rotação com base no modo autônomo
        velocidade = 0.0
        rotacao = 0.0
        tempo = self.timer.get()

        if self.autonomous_mode == 1 and tempo < 1.0:
            # Modo 1: Avançar por 1 segundo
            velocidade = 0.5  # Velocidade média para frente
        elif self.autonomous_mode == 2:
            # Modo 2: Avançar 1 s, girar -90° relativo, avançar 1 s
            if tempo < 1.0:
                velocidade = 0.5
            else:
                # Calcula o ângulo relativo normalizado
                relative_heading = math.degrees(self.sim.heading - self.initial_heading)
                relative_heading = (
                    relative_heading + 180
                ) % 360 - 180  # Normaliza para [-180, 180]
                if relative_heading > -90.0:
                    rotacao = 0.5  # Gira para a esquerda
                elif tempo < 3.0:
                    velocidade = 0.5

        # Inverte rotação para consistência com teleop
        rotacao = -rotacao

        # Processa comandos com a simulação
        velocidade, rotacao, pose, orientacao = self.sim.update(velocidade, rotacao)

        # Atualiza o campo virtual
        self.field.setRobotPose(pose)

        # Envia comandos ao robô
        self.tracao.arcadeDrive(velocidade, rotacao)

        # Dados para monitoramento (opcional, para verificar o giro)
        # SmartDashboard.putNumber("Orientação (graus)", math.degrees(orientacao))


if __name__ == "__main__":
    import ntcore  # Verifica se o script está sendo executado diretamente (e não importado como módulo).

    inst = (
        ntcore.NetworkTableInstance.getDefault()
    )  # Obtém a instância padrão do NetworkTables para comunicação com ferramentas como o SmartDashboard.
    inst.startServer(
        port=5810
    )  # Inicia o servidor do NetworkTables na porta 5810 para troca de dados.
    wpilib.run(
        MyRobot
    )  # Inicia o programa do robô, executando a classe MyRobot usando o framework do WPILib.
