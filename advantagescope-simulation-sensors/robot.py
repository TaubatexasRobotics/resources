#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file        robot.py
@brief       Simulação de robô FRC com drivetrain diferencial, climber, intake e LED strip.
@details     Este script implementa uma simulação educacional de robô FRC, permitindo:
             - Controle via joystick no modo teleoperado, com movimentação diferencial, controle do climber (quatro níveis), intake (garra) e LED strip.
             - Movimentos programados no modo autônomo.
             - Integração com SmartDashboard e Field2d para visualização da posição do robô no AdvantageScope.
             - Simulação de sensores (giroscópio, encoders) e mecanismos (climber, intake, LED strip) no AdvantageScope.
             - Controle de motores REV SparkMax (brushed) em configuração diferencial.
             Utiliza a classe SimpleKinematicSim para simulação cinemática de movimento, encoders e sensores.
             Link de download do AdvantageScope:
             https://github.com/Mechanical-Advantage/AdvantageScope/releases/download/v4.1.6/advantagescope-wpilib-win-x64.zip
             Obs: No AdvantageScope, após adicionar o robô, mude Rotation Units para "Degrees" para visualização correta.
@note        Sensores reais e feedback de hardware não estão implementados; apenas simulação.
@version     2.2
@date        2025-09-30
@authors     Taubatexas 7459
"""

import wpilib
import wpilib.drive
import rev
from wpilib import SmartDashboard, Field2d, ADIS16470_IMU, Encoder, Mechanism2d
from wpilib.simulation import EncoderSim
from robot_sim import SimpleKinematicSim


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        #################################################################################
        # INICIALIZAÇÃO DO ROBÔ
        #################################################################################

        """
        Inicializa o robô, configurando motores, sensores, simulação, mecanismos e seletores de interface.
        Executado uma vez quando o robô é ligado ou a simulação é iniciada.
        """
        # Configura os motores do drivetrain diferencial
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
        )  # Grupo lado esquerdo
        self.direita = wpilib.MotorControllerGroup(
            self.motor_direita_frente, self.motor_direita_tras
        )  # Grupo lado direito
        self.direita.setInverted(
            True
        )  # Inverte motores à direita para movimento consistente

        # Configura o sistema de tração diferencial
        self.tracao = wpilib.drive.DifferentialDrive(self.esquerda, self.direita)
        self.tracao.setSafetyEnabled(
            False
        )  # Desativado para simulação (conforme seu ajuste)

        # Configura o joystick na porta 0 (controle Xbox)
        self.joystick = wpilib.Joystick(0)

        #################################################################################
        # INICIALIZAÇÃO - SIMULAÇÃO DA ARENA VIRTUAL
        #################################################################################

        # Configura o campo virtual para visualização da posição do robô
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)  # Exibe o campo no AdvantageScope

        #################################################################################
        # INICIALIZAÇÃO - SIMULAÇÃO DOS SENSORES (GYRO E ENCODERS)
        #################################################################################

        # Inicializa sensores simulados
        self.gyro = ADIS16470_IMU()  # Giroscópio simulado (ADIS16470)
        self.gyro.reset()  # Reseta a orientação inicial para 0 graus
        self.encoder_left = Encoder(0, 1)  # Encoder do lado esquerdo (portas 0 e 1)
        self.encoder_right = Encoder(2, 3)  # Encoder do lado direito (portas 2 e 3)
        self.encoder_left.setDistancePerPulse(1.0 / 1000.0)  # 1 pulso = 1 mm
        self.encoder_right.setDistancePerPulse(1.0 / 1000.0)  # 1 pulso = 1 mm

        # Inicializa simulação de encoders
        self.encoder_left_sim = EncoderSim(
            self.encoder_left
        )  # Simulador do encoder esquerdo
        self.encoder_right_sim = EncoderSim(
            self.encoder_right
        )  # Simulador do encoder direito

        #################################################################################
        # INICIALIZAÇÃO - POSIÇÃO INICIAL DO ROBÔ NA ARENA VIRTUAL
        #################################################################################

        # Inicializa a simulação cinemática com posição inicial realista para o campo Reefscape
        self.initial_x = 10.40  # Armazena posição inicial X (m)
        self.initial_y = 1.40  # Armazena posição inicial Y (m)
        self.initial_yaw_deg = 0.0  # Armazena orientação inicial (graus)

        self.sim = SimpleKinematicSim(
            initial_x=self.initial_x,
            initial_y=self.initial_y,
            initial_yaw_deg=self.initial_yaw_deg,  # Posição inicial (m) e orientação (graus)
            wheelbase=0.6,  # Distância entre rodas (m)
            encoder_left_sim=self.encoder_left_sim,
            encoder_right_sim=self.encoder_right_sim,
            robot=self,  # Passa a instância do robô
        )
        self.sim.reset()  # Reseta giroscópio e encoders na inicialização

        #################################################################################
        # INICIALIZAÇÃO - SIMULAÇÃO DO MECANISMO CLIMBER E INTAKE
        #################################################################################

        # Configura o mecanismo para o climber e intake (Reefscape)
        self.mech = Mechanism2d(3.0, 3.0)  # Canvas 3x3 metros para visualização
        self.climber_root = self.mech.getRoot(
            "ClimberRoot", 1.90, 0.1
        )  # Raiz do mecanismo
        self.climber_ligament = self.climber_root.appendLigament(
            "Barge Climber", 0.0, 90.0, 8, wpilib.Color8Bit(wpilib.Color.kSilver)
        )  # Climber (prata)
        self.intake_ligament = self.climber_ligament.appendLigament(
            "Intake Claw", 0.5, -90.0, 8, wpilib.Color8Bit(wpilib.Color.kBlue)
        )  # Garra (azul)
        SmartDashboard.putData("Intake", self.mech)  # Exibe mecanismo no AdvantageScope
        self.intake_angle = 90.0  # Ângulo inicial da garra (graus)
        self.climber_length = 0.80  # Comprimento inicial do climber (m)
        SmartDashboard.putNumber("Intake Angle", self.intake_angle)
        SmartDashboard.putNumber("Climber Length", self.climber_length)

        #################################################################################
        # INICIALIZAÇÃO - SIMULAÇÃO DA LED STRIP
        #################################################################################

        # Configura o mecanismo para simular a LED strip
        self.led_mech = Mechanism2d(1.0, 1.0)  # Canvas 1x1 metro para a LED
        self.led_root = self.led_mech.getRoot("LedRoot", 0.5, 0.5)  # Raiz centrada
        self.led_square = self.led_root.appendLigament(
            "LedStrip", 0.2, 0.0, 10, wpilib.Color8Bit(wpilib.Color.kRed)
        )  # Quadrado vermelho representando a LED
        SmartDashboard.putData("LED Strip", self.led_mech)  # Exibe no AdvantageScope
        self.led_color = "red"  # Cor inicial da LED
        self.led_toggle_triggered = False  # Flag para evitar múltiplas trocas no DPad

        #################################################################################
        # INICIALIZAÇÃO - SIMULAÇÃO DO SELETOR PARA RESETAR POSIÇÃO PARA INICIAL
        #################################################################################

        # Configura seletor para resetar a posição no SmartDashboard
        self.reset_selector = wpilib.SendableChooser()
        self.reset_selector.setDefaultOption("Manter Posição", 0)
        self.reset_selector.addOption("Resetar para Inicial", 1)
        SmartDashboard.putData("Reset Position", self.reset_selector)
        self.reset_triggered = False  # Flag para controlar resets

        #################################################################################
        # CRIAÇÃO DE MODOS AUTÔNOMOS - CRIE QUANTOS MODOS AUTÔNOMOS DESEJAR!
        #################################################################################

        # Configura temporizador para modos autônomos
        self.timer = wpilib.Timer()

        # Configura seletor de modos autônomos no SmartDashboard
        self.selector = wpilib.SendableChooser()
        self.selector.setDefaultOption("Parado", 0)  # Não se move
        self.selector.addOption("Modo Autônomo 1", 1)  # Modo Autônomo 1
        self.selector.addOption("Modo Autônomo 2", 2)  # Modo Autônomo 2
        self.selector.addOption("Modo Autônomo 3", 3)  # Modo Autônomo 3
        SmartDashboard.putData("Auto Selector", self.selector)

    def robotPeriodic(self):
        #################################################################################
        # ATUALIZAÇÃO DE DADOS DOS SENSORES E VERIFICAÇÃO DE RESET DE POSIÇÃO
        #################################################################################

        """
        Executado periodicamente em todos os modos (teleop, autônomo, etc.).
        Atualiza dados de sensores, mecanismos e SmartDashboard para monitoramento.
        """
        # Atualiza SmartDashboard com dados dos sensores
        SmartDashboard.putNumber(
            "Gyro Yaw", self.gyro.getAngle()
        )  # Orientação do giroscópio (graus)
        SmartDashboard.putNumber(
            "Encoder Esquerdo (pulsos)", self.encoder_left.get()
        )  # Pulsos do encoder esquerdo
        SmartDashboard.putNumber(
            "Encoder Direito (pulsos)", self.encoder_right.get()
        )  # Pulsos do encoder direito
        SmartDashboard.putNumber(
            "Sensor de Distância (m)", self.sim.distance_sensor
        )  # Distância simulada (m)
        SmartDashboard.putNumber(
            "Distância Percorrida (m)",
            (self.encoder_left.getDistance() + self.encoder_right.getDistance()) / 2,
        )  # Média dos encoders
        SmartDashboard.putNumber(
            "Posição X (m)", self.sim.pose.X()
        )  # Posição X no campo
        SmartDashboard.putNumber(
            "Posição Y (m)", self.sim.pose.Y()
        )  # Posição Y no campo
        SmartDashboard.putNumber(
            "Orientação (graus)", -self.sim.pose.rotation().degrees()
        )  # Orientação do robô

        # Atualiza o estado do mecanismo (climber e intake) no AdvantageScope
        self.climber_ligament.setLength(
            self.climber_length
        )  # Define comprimento do climber
        self.intake_ligament.setAngle(
            self.intake_angle - 90.0
        )  # Ajusta ângulo da garra
        SmartDashboard.putNumber("Intake Angle", self.intake_angle)
        SmartDashboard.putNumber("Climber Length", self.climber_length)

        # Atualiza a cor da LED strip no mecanismo
        if self.led_color == "red":
            self.led_square.setColor(
                wpilib.Color8Bit(wpilib.Color.kRed)
            )  # Define cor vermelha
        else:  # green
            self.led_square.setColor(
                wpilib.Color8Bit(wpilib.Color.kGreen)
            )  # Define cor verde
        SmartDashboard.putString(
            "LED Color", self.led_color
        )  # Exibe cor atual no SmartDashboard

        # Verifica o seletor de reset
        reset_option = self.reset_selector.getSelected()
        if reset_option == 1 and not self.reset_triggered:
            # Reseta a simulação para a posição inicial
            self.sim = SimpleKinematicSim(
                initial_x=self.initial_x,
                initial_y=self.initial_y,
                initial_yaw_deg=self.initial_yaw_deg,
                wheelbase=0.6,
                encoder_left_sim=self.encoder_left_sim,
                encoder_right_sim=self.encoder_right_sim,
                robot=self,  # Passa a instância do robô
            )
            self.sim.reset()  # Reseta giroscópio e encoders
            self.gyro.reset()  # Reseta o giroscópio simulado
            self.encoder_left.reset()  # Reseta o encoder esquerdo
            self.encoder_right.reset()  # Reseta o encoder direito
            self.field.setRobotPose(self.sim.pose)  # Atualiza a pose no Field2d
            # Reseta o estado do climber, intake e LED
            self.climber_length = 0.80  # Fixed: Align with teleop level 1
            self.intake_angle = 90.0
            self.led_color = "red"
            self.led_square.setColor(wpilib.Color8Bit(wpilib.Color.kRed))
            SmartDashboard.putNumber("Climber Length", self.climber_length)
            SmartDashboard.putNumber("Intake Angle", self.intake_angle)
            SmartDashboard.putString("LED Color", self.led_color)
            self.reset_triggered = True  # Marca o reset como concluído
        elif reset_option == 0:
            self.reset_triggered = (
                False  # Reseta a flag quando "Manter Posição" é selecionado
            )

    def teleopPeriodic(self):
        #################################################################################
        # MODO TELEOPERADO: VELOCIDADE, ROTAÇÃO, CLIMBER, GARRA E LED STRIP
        #################################################################################

        """
        Executado periodicamente no modo teleoperado (20 ms).
        Lê inputs do joystick para controlar o drivetrain, climber, intake e LED strip.
        """
        # Lê comandos do joystick (controle Xbox)
        velocidade = -self.joystick.getRawAxis(
            1
        )  # Eixo Y: frente (positivo) ou trás (negativo)
        rotacao = -self.joystick.getRawAxis(
            4
        )  # Eixo X do stick direito: girar esquerda/direita

        # Controle do Climber (quatro níveis)
        if self.joystick.getRawButtonPressed(1):  # Botão A - Nível 1 (0.80 m)
            self.climber_length = 0.80
            SmartDashboard.putNumber("Climber Length", self.climber_length)
        elif self.joystick.getRawButtonPressed(2):  # Botão B - Nível 2 (1.2 m)
            self.climber_length = 1.2
            SmartDashboard.putNumber("Climber Length", self.climber_length)
        elif self.joystick.getRawButtonPressed(3):  # Botão X - Nível 3 (1.7 m)
            self.climber_length = 1.7
            SmartDashboard.putNumber("Climber Length", self.climber_length)
        elif self.joystick.getRawButtonPressed(4):  # Botão Y - Nível 4 (2.4 m)
            self.climber_length = 2.4
            SmartDashboard.putNumber("Climber Length", self.climber_length)

        # Controle da Garra (Intake)
        if self.joystick.getRawAxis(2) > 0.5:  # Gatilho LT - Garra a 90 graus
            self.intake_angle = 90.0
            SmartDashboard.putNumber("Intake Angle", self.intake_angle)
        elif self.joystick.getRawAxis(3) > 0.5:  # Gatilho RT - Garra a 0 graus
            self.intake_angle = 0.0
            SmartDashboard.putNumber("Intake Angle", self.intake_angle)

        # Controle da LED strip (alterna cor com DPad Down)
        if self.joystick.getPOV() == 180 and not self.led_toggle_triggered:  # DPad Down
            self.led_color = (
                "green" if self.led_color == "red" else "red"
            )  # Alterna entre vermelho e verde
            self.led_toggle_triggered = True
            SmartDashboard.putString("LED Color", self.led_color)
        elif self.joystick.getPOV() != 180:
            self.led_toggle_triggered = False  # Reseta a flag quando o DPad é solto

        # Atualiza a simulação cinemática com os comandos do joystick
        velocidade, rotacao, pose, orientacao, left_dist, right_dist, distance = (
            self.sim.update(velocidade, rotacao)
        )
        self.encoder_left_sim.setDistance(left_dist)  # Atualiza encoder esquerdo
        self.encoder_right_sim.setDistance(right_dist)  # Atualiza encoder direito
        self.field.setRobotPose(pose)  # Atualiza a posição do robô no Field2d
        self.tracao.arcadeDrive(velocidade, rotacao)  # Envia comandos ao drivetrain

    def autonomousInit(self):
        #################################################################################
        # INICIALIZAÇÃO DO MODO AUTÔNOMO: VERIFICA MODO, RESETA SENSORES E INICIA TIMER
        #################################################################################
        """
        Executado uma vez no início do modo autônomo.
        Inicializa o temporizador, seleciona o modo autônomo e reseta sensores.
        """
        self.autonomous_mode = (
            self.selector.getSelected()
        )  # Obtém o modo autônomo selecionado
        self.estado = 1  # Inicializa estado do modo autônomo
        self.gyro.reset()  # Reseta o giroscópio
        self.encoder_left.reset()  # Reseta encoder esquerdo
        self.encoder_right.reset()  # Reseta encoder direito
        self.timer.reset()  # Reseta o temporizador
        self.timer.start()  # Inicia o temporizador
        self.initial_heading = self.gyro.getAngle()

        # ('move_sensor', {'distance': 1.0, 'speed': 0.8, 'reset_encoders': True}),  # Move até 1m do obstáculo

        # Exemplo de modo autônomo utilizando comandos
        if self.autonomous_mode == 3:  # Define sequência para modo 3
            self.sim.reset()  # Garante que a simulação esteja limpa

            # Sequência de comandos
            self.sim.action_sequence = [
                ("set_intake", {"angle": 90.0}),
                ("move", {"distance": 1.10, "speed": 1.0, "reset_encoders": True}),
                ("wait", {"duration": 0.5}),
                ("turn", {"angle": -60.0, "speed": 0.5, "reset_gyro": True}),
                ("wait", {"duration": 0.5}),
                ("move", {"distance": 1.5, "speed": 1.0, "reset_encoders": True}),
                ("wait", {"duration": 0.5}),
                ("set_climber", {"length": 1.3}),
                ("wait", {"duration": 0.5}),
                ("set_intake", {"angle": 0.0}),
                ("wait", {"duration": 0.5}),
                ("turn", {"angle": 180.0, "speed": 0.5, "reset_gyro": True}),
                ("wait", {"duration": 0.5}),
                ("move", {"distance": 1.5, "speed": 1.0, "reset_encoders": True}),
                ("wait", {"duration": 0.5}),
                ("turn", {"angle": 60.0, "speed": 0.5, "reset_gyro": True}),
                ("wait", {"duration": 0.5}),
                ("move", {"distance": 1.5, "speed": 1.0, "reset_encoders": True}),
                ("wait", {"duration": 0.5}),
                ("set_climber", {"length": 2.30}),
                ("wait", {"duration": 0.5}),
                ("set_intake", {"angle": 90.0}),
                ("wait", {"duration": 0.5}),
                ("set_climber", {"length": 0.70}),
                ("wait", {"duration": 0.5}),
                ("move", {"distance": 1.0, "speed": 1.0, "reset_encoders": True}),
                ("set_led", {"color": "green"}),
                ("stop", {}),
            ]
            self.sim.current_action_index = 0

    def autonomousPeriodic(self):
        #################################################################################
        # MODO AUTÔNOMO: PARA PRÁTICA DE ALGORITMOS
        #################################################################################
        """
        Executado periodicamente no modo autônomo (20 ms).
        Executa movimentos programados com base no modo selecionado no SmartDashboard.
        """
        # COMANDOS REFERENTES À SENSORES E MECANISMOS DISPONÍVEIS =======================

        # Encoders:
        # Para calcular distância percorrida (m): average_distance = (self.encoder_left.getDistance() + self.encoder_right.getDistance()) / 2
        # Para resetar medição dos encoders:      self.encoder_left.reset() e self.encoder_right.reset()

        # Gyro:
        # Para obter ângulo do gyro  (graus):     gyro_heading = self.gyro.getAngle()
        # Para calcular ângulo relativo (graus):  relative_heading = self.gyro.getAngle() - self.initial_heading
        # Para resetar a medição do gyro:         self.gyro.reset()

        # Sensor de distância:
        # Para obter a distância até o limite     self.sim.distance_sensor
        # da arena à frente do robô (m):

        # Mecanismos:
        # Para mudar angulo da garra (graus):     self.intake_angle = 90.0
        # Para mudar altura do climber (metros):  self.climber_length = 1.2
        # Para mudar cor da led strip:            self.led_color = "green" ou "red"

        # Novos comandos para modo autônomo simplificado:
        # Para mover uma distância (m):           velocidade, rotacao, completado = self.sim.move_distance(distancia, velocidade)
        # Para girar a um ângulo (graus):         velocidade, rotacao, completado = self.sim.turn_to_angle(angulo, velocidade)
        # Para parar o robô:                      velocidade, rotacao, completado = self.sim.stop()
        # Para mover até uma distância do sensor: velocidade, rotacao, completado = self.sim.move_to_sensor_distance(distancia, velocidade)
        # Para definir ângulo da garra:           velocidade, rotacao, completado = self.sim.set_intake_angle(angulo)
        # Para definir altura do climber:         velocidade, rotacao, completado = self.sim.set_climber_length(comprimento)
        # Para definir cor da LED:                velocidade, rotacao, completado = self.sim.set_led_color(cor)
        # Para executar uma sequência de ações:   velocidade, rotacao = self.sim.run_sequence(sequencia)

        # ===============================================================================

        velocidade = 0.0  # Velocidade linear inicial
        rotacao = 0.0  # Velocidade de rotação inicial
        tempo = self.timer.get()  # Tempo decorrido no modo autônomo

        # MODO AUTÔNOMO 1 (simples, baseado em tempo)
        if self.autonomous_mode == 1:
            if tempo > 0.0 and tempo < 1.0:  # Avança por 1 segundo
                velocidade = 1.0
                rotacao = 0.0
            elif tempo > 1.0 and tempo < 5.0:  # Gira por 4 segundos
                velocidade = 0.5
                rotacao = -0.8

        # MODO AUTÔNOMO 2 (utilizando sensores e máquina de estados)
        elif self.autonomous_mode == 2:
            # ESTADO 1 (avançar 1 m)
            if self.estado == 1:
                distancia = (
                    self.encoder_left.getDistance() + self.encoder_right.getDistance()
                ) / 2
                if distancia < 1:
                    velocidade = 1.0
                else:
                    velocidade = 0.0
                    self.encoder_left.reset()
                    self.encoder_right.reset()
                    self.estado = 2
            # ESTADO 2 (girar -60 graus)
            elif self.estado == 2:
                oriantacao = self.gyro.getAngle() - self.initial_heading
                if oriantacao > -60:
                    rotacao = -0.5
                else:
                    rotacao = 0.0
                    self.gyro.reset()
                    self.initial_heading = self.gyro.getAngle()
                    self.estado = 3
            # ESTADO 3 (avançar 1.5 m)
            elif self.estado == 3:
                distancia = (
                    self.encoder_left.getDistance() + self.encoder_right.getDistance()
                ) / 2
                if distancia < 1.5:
                    velocidade = 1.0
                else:
                    velocidade = 0.0
                    self.encoder_left.reset()
                    self.encoder_right.reset()
                    self.estado = 4
            # ESTADO 4 (parar)
            elif self.estado == 4:
                velocidade = 0.0
                rotacao = 0.0

        # MODO AUTÔNOMO 3 (exemplo com comandos)
        elif self.autonomous_mode == 3:
            velocidade, rotacao = self.sim.run_sequence(self.sim.action_sequence)

        #################################################################################
        # ATUALIZA A SIMULAÇÃO
        #################################################################################

        # Inverte rotação para consistência com o modo teleoperado
        rotacao = -rotacao

        # Atualiza os parâmetros simulados de acordo com os comandos de velocidade e rotação
        velocidade, rotacao, pose, orientacao, left_dist, right_dist, distance = (
            self.sim.update(velocidade, rotacao)
        )

        # Atualiza encoders e pose do robô
        self.encoder_left_sim.setDistance(left_dist)
        self.encoder_right_sim.setDistance(right_dist)
        self.field.setRobotPose(pose)

        #################################################################################
        # ENVIA COMANDOS PARA O ROBÔ REAL
        #################################################################################

        self.tracao.arcadeDrive(velocidade, rotacao)


#################################################################################
# CONFIGURA CONEXÃO NETWORKTABLES E INICIA PROGRAMA DO ROBÔ
#################################################################################
if __name__ == "__main__":
    import ntcore

    inst = (
        ntcore.NetworkTableInstance.getDefault()
    )  # Obtém a instância do NetworkTables
    inst.startServer(port=5810)  # Inicia o servidor na porta 5810
    wpilib.run(MyRobot)  # Inicia o programa do robô
