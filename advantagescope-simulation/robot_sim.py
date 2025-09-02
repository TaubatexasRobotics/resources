#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file        robot_sim.py
@brief       Módulo de simulação de robô diferencial FRC para fins educacionais.
@details     Contém a classe SimpleKinematicSim, que implementa uma simulação cinemática simples
             para robôs com drivetrain diferencial. Permite:
             - Processar comandos de joystick (velocidade linear e rotação).
             - Aplicar limites de segurança e faixa morta (deadband).
             - Calcular nova posição (x, y) e orientação do robô.
             - Retornar a pose para visualização no AdvantageScope ou Field2d.
             
             Ideal para estudo e prática de programação de robôs FRC sem hardware físico.
@usage       from robot_sim import SimpleKinematicSim
             sim = SimpleKinematicSim(initial_x=1.5, initial_y=2.0, initial_yaw_deg=0.0)
             velocidade, rotacao, pose, orientacao = sim.update(joystick_y, joystick_x)
@note        Sensores reais e feedback de hardware não estão implementados; apenas simulação.
@version     2.1
@date        2025-09-01
@license     MIT
@authors     Taubatexas 7459 
"""

# Importa bibliotecas necessárias
import math  # Funções matemáticas (seno, cosseno, radianos, etc.)
import time  # Controle de tempo e temporização (time.sleep, time.time)
from wpimath.geometry import Pose2d, Rotation2d  
# Pose2d: representa posição (x, y) + orientação no plano
# Rotation2d: representa apenas a orientação (ângulo) no plano

class SimpleKinematicSim:
    def __init__(self, initial_x=0.0, initial_y=0.0, initial_yaw_deg=0.0,
                 max_linear_mps=2.0, max_angular_rps=math.radians(120.0)):
        """
        Inicializa a simulação do robô.
        Parâmetros:
        - initial_x, initial_y: Posição inicial em metros
        - initial_yaw_deg: Orientação inicial em graus
        - max_linear_mps: Velocidade linear máxima (m/s)
        - max_angular_rps: Velocidade angular máxima (rad/s)
        """
        # Define a posição inicial e orientação
        self.pose = Pose2d(initial_x, initial_y, Rotation2d.fromDegrees(initial_yaw_deg))
        self.heading = math.radians(initial_yaw_deg)  # Orientação em radianos
        self.MAX_LINEAR_MPS = max_linear_mps          # Velocidade máxima linear
        self.MAX_ANGULAR_RPS = max_angular_rps        # Velocidade máxima angular
        self.DEADBAND = 0.2                           # Faixa morta do joystick
        self._last_time = None                        # Último tempo para cálculo de dt

    def apply_deadband(self, value):
        """
        Aplica faixa morta (deadband) ao valor de entrada.
        Retorna 0 se o valor estiver dentro da faixa morta.
        """
        if abs(value) < self.DEADBAND:
            return 0.0
        return value

    def clamp(self, value, min_value, max_value):
        """
        Limita o valor entre min_value e max_value.
        """
        return max(min_value, min(max_value, value))

    def update(self, raw_v, raw_r, dt=None):
        """
        Atualiza a simulação com base nos comandos do joystick.
        Parâmetros:
        - raw_v: Velocidade linear (de -1 a 1)
        - raw_r: Velocidade de rotação (de -1 a 1)
        - dt: Intervalo de tempo (opcional, calculado automaticamente se None)
        Retorna:
        - raw_v, raw_r: Comandos ajustados
        - pose: Nova pose do robô (x, y, orientação)
        - heading: Orientação atual em radianos
        """
        # Calcula o intervalo de tempo (dt)
        current_time = time.time()
        if dt is None:
            if self._last_time is None:
                dt = 0.02  # Padrão de 50 Hz (20 ms)
            else:
                dt = current_time - self._last_time
        self._last_time = current_time

        # Aplica faixa morta e limita comandos
        raw_v = self.apply_deadband(self.clamp(raw_v, -1.0, 1.0))
        raw_r = self.apply_deadband(self.clamp(raw_r, -1.0, 1.0))

        # Converte comandos em velocidades físicas
        linear_vel = raw_v * self.MAX_LINEAR_MPS
        angular_vel = raw_r * self.MAX_ANGULAR_RPS

        # Atualiza a orientação (heading)
        self.heading += angular_vel * dt
        # Normaliza o ângulo de orientação para o intervalo [-π, π] usando atan2.
        # Isso evita que o ângulo cresça indefinidamente (ex., >2π ou <0) ao acumular rotações,
        # mantendo-o equivalente em termos de direção para cálculos e visualização.
        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))

        # Calcula deslocamento em x e y
        # Usa a velocidade linear (m/s), a orientação atual (self.heading, em radianos) e o intervalo de tempo (dt, em segundos).
        # math.cos(self.heading) e math.sin(self.heading) decompõem a velocidade nas direções x e y, respectivamente,
        # para determinar quanto o robô se move em cada eixo durante o intervalo dt.
        dx = linear_vel * math.cos(self.heading) * dt  # Deslocamento na direção x (metros)
        dy = linear_vel * math.sin(self.heading) * dt  # Deslocamento na direção y (metros)

        # Converte a orientação para graus para o Field2d
        heading_for_field = math.degrees(self.heading)

        # Atualiza a pose do robô
        self.pose = Pose2d(self.pose.X() + dx, self.pose.Y() + dy, Rotation2d.fromDegrees(heading_for_field))

        return raw_v, raw_r, self.pose, self.heading