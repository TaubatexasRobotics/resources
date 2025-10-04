#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file        robot_sim.py
@brief       Módulo de simulação cinemática para robô diferencial FRC, além de comandos de ações.
@details     Contém a classe SimpleKinematicSim, que implementa uma simulação cinemática simples
             para robôs com drivetrain diferencial. Permite:
             - Processar comandos de joystick (velocidade linear e rotação).
             - Simular sensores (giroscópio, encoders, sensor de distância).
             - Aplicar faixa morta (deadband) e limites de velocidade.
             - Calcular posição (x, y) e orientação do robô no campo.
             - Retornar dados para visualização no AdvantageScope ou Field2d.
             - Métodos para ações autônomas simplificadas (move_distance, turn_to_angle, stop, move_to_sensor_distance, set_intake_angle, set_climber_length, set_led_color, run_sequence).
             Ideal para estudo e prática de programação FRC sem hardware físico.
@usage       from robot_sim import SimpleKinematicSim
             sim = SimpleKinematicSim(initial_x=10.40, initial_y=1.40, initial_yaw_deg=0.0, robot=robot_instance)
             velocidade, rotacao, pose, orientacao, left_dist, right_dist, distance = sim.update(joystick_y, joystick_x)
@note        Sensores reais e feedback de hardware não estão implementados; apenas simulação.
@version     2.9
@date        2025-09-30
@authors     Taubatexas 7457
"""

import wpilib
import math
import time
import random
from wpimath.geometry import Pose2d, Rotation2d
from wpilib.simulation import SimDeviceSim
from wpilib import SmartDashboard

class SimpleKinematicSim:
    def __init__(self, initial_x=0.0, initial_y=0.0, initial_yaw_deg=0.0,
                 max_linear_mps=2.0, max_angular_rps=math.radians(120.0),
                 wheelbase=0.6, counts_per_meter=1000.0,
                 encoder_left_sim=None, encoder_right_sim=None, robot=None):
        """
        Inicializa a simulação do robô com suporte a sensores simulados e mecanismos.
        Parâmetros:
        - initial_x, initial_y: Posição inicial em metros
        - initial_yaw_deg: Orientação inicial em graus
        - max_linear_mps: Velocidade linear máxima (m/s)
        - max_angular_rps: Velocidade angular máxima (rad/s)
        - wheelbase: Distância entre rodas (m) para cálculos diferenciais
        - counts_per_meter: Contagens do encoder por metro
        - encoder_left_sim, encoder_right_sim: Objetos EncoderSim para simulação
        - robot: Instância do robô (MyRobot) para acessar intake_angle, climber_length, led_color
        """
        self.pose = Pose2d(initial_x, initial_y, Rotation2d.fromDegrees(initial_yaw_deg))
        self.heading = math.radians(initial_yaw_deg)
        self.field_heading = math.radians(initial_yaw_deg)
        self.MAX_LINEAR_MPS = max_linear_mps
        self.MAX_ANGULAR_RPS = max_angular_rps
        self.DEADBAND = 0.2
        self._last_time = None
        self.wheelbase = wheelbase
        self.counts_per_meter = counts_per_meter
        self.encoder_left_sim = encoder_left_sim
        self.encoder_right_sim = encoder_right_sim
        self.robot = robot  # Referência à instância do robô
        self.FIELD_LENGTH_X = 17.548
        self.FIELD_WIDTH_Y = 8.052
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.gyro_sim = SimDeviceSim("Gyro:ADIS16470[0]")
        self.gyro_sim_yaw = self.gyro_sim.getDouble("gyro_angle_z")
        self.gyro_sim_yaw.set(initial_yaw_deg)
        self.gyro_sim_pitch = self.gyro_sim.getDouble("gyro_angle_x")
        self.gyro_sim_roll = self.gyro_sim.getDouble("gyro_angle_y")
        self.gyro_sim_rate_x = self.gyro_sim.getDouble("gyro_rate_x")
        self.gyro_sim_rate_y = self.gyro_sim.getDouble("gyro_rate_y")
        self.gyro_sim_rate_z = self.gyro_sim.getDouble("gyro_rate_z")
        self.gyro_sim_accel_x = self.gyro_sim.getDouble("accel_x")
        self.gyro_sim_accel_y = self.gyro_sim.getDouble("accel_y")
        self.gyro_sim_accel_z = self.gyro_sim.getDouble("accel_z")
        self.distance_sensor = 10.0
        self.last_linear_vel = 0.0

        # Variáveis para ações autônomas
        self.action_active = False
        self.action_type = None
        self.target_distance = 0.0
        self.target_angle = 0.0
        self.initial_heading = 0.0
        self.initial_distance = 0.0
        # Variáveis para sequência de ações
        self.action_sequence = []
        self.current_action_index = 0

    def apply_deadband(self, value):
        return 0.0 if abs(value) < self.DEADBAND else value

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def reset(self):
        self.heading = 0.0
        self.gyro_sim_yaw.set(0.0)
        self.gyro_sim_pitch.set(0.0)
        self.gyro_sim_roll.set(0.0)
        self.gyro_sim_rate_x.set(0.0)
        self.gyro_sim_rate_y.set(0.0)
        self.gyro_sim_rate_z.set(0.0)
        self.gyro_sim_accel_x.set(0.0)
        self.gyro_sim_accel_y.set(0.0)
        self.gyro_sim_accel_z.set(9.81)
        self.left_distance = 0.0
        self.right_distance = 0.0
        if self.encoder_left_sim is not None:
            self.encoder_left_sim.setDistance(0.0)
        if self.encoder_right_sim is not None:
            self.encoder_right_sim.setDistance(0.0)
        self.last_linear_vel = 0.0
        self.action_active = False
        self.action_type = None
        self.action_sequence = []
        self.current_action_index = 0

    def move_distance(self, distance, speed=1.0, left_distance=0.0, right_distance=0.0, reset_encoders=True):
        """
        Move o robô por uma distância específica.
        Parâmetros:
        - distance: Distância alvo em metros (positivo = frente, negativo = trás)
        - speed: Velocidade desejada (0 a 1)
        - left_distance: Distância atual do encoder esquerdo (m)
        - right_distance: Distância atual do encoder direito (m)
        - reset_encoders: Se True, reseta encoders ao completar a ação
        Retorna:
        - Tuple (velocidade, rotacao, completado)
        """
        if not self.action_active:
            self.action_active = True
            self.action_type = 'move'
            self.target_distance = distance
            self.initial_distance = (left_distance + right_distance) / 2

        current_distance = (left_distance + right_distance) / 2
        distance_traveled = current_distance - self.initial_distance

        if abs(distance_traveled) < abs(self.target_distance):
            velocidade = speed if distance >= 0 else -speed
            rotacao = 0.0
            return velocidade, rotacao, False
        else:
            self.action_active = False
            self.action_type = None
            if reset_encoders and self.encoder_left_sim is not None and self.encoder_right_sim is not None:
                self.encoder_left_sim.setDistance(0.0)
                self.encoder_right_sim.setDistance(0.0)
                self.left_distance = 0.0
                self.right_distance = 0.0
            return 0.0, 0.0, True

    def turn_to_angle(self, angle, speed=0.5, current_heading=0.0, reset_gyro=True):
        """
        Gira o robô até um ângulo relativo específico.
        Parâmetros:
        - angle: Ângulo alvo em graus (positivo = horário, negativo = anti-horário)
        - speed: Velocidade de rotação desejada (0 a 1)
        - current_heading: Orientação atual em graus (ângulo absoluto do giroscópio)
        - reset_gyro: Se True, reseta o giroscópio ao completar a ação
        Retorna:
        - Tuple (velocidade, rotacao, completado)
        """
        if not self.action_active:
            self.action_active = True
            self.action_type = 'turn'
            self.target_angle = angle
            self.initial_heading = current_heading

        relative_heading = current_heading - self.initial_heading
        if abs(relative_heading) < abs(self.target_angle) - 0.5:  # Tolerância de 0.5 grau
            rotacao = speed if self.target_angle < 0 else -speed
            velocidade = 0.0
            return velocidade, rotacao, False
        else:
            self.action_active = False
            self.action_type = None
            if reset_gyro:
                self.gyro_sim_yaw.set(0.0)
                self.heading = 0.0
                self.initial_heading = 0.0
            return 0.0, 0.0, True

    def stop(self):
        """
        Para o robô.
        Retorna:
        - Tuple (velocidade, rotacao, completado)
        """
        self.action_active = False
        self.action_type = None
        return 0.0, 0.0, True

    def move_to_sensor_distance(self, distance, speed=1.0, current_distance=0.0, reset_encoders=True):
        """
        Move o robô até uma distância específica medida pelo sensor de distância.
        Parâmetros:
        - distance: Distância alvo em metros (positivo = frente até atingir a distância)
        - speed: Velocidade desejada (0 a 1)
        - current_distance: Distância atual do sensor de distância (m)
        - reset_encoders: Se True, reseta encoders ao completar a ação
        Retorna:
        - Tuple (velocidade, rotacao, completado)
        """
        if not self.action_active:
            self.action_active = True
            self.action_type = 'move_sensor'
            self.target_distance = distance

        if current_distance > self.target_distance + 0.05:  # Tolerância de 5 cm
            velocidade = speed
            rotacao = 0.0
            return velocidade, rotacao, False
        else:
            self.action_active = False
            self.action_type = None
            if reset_encoders and self.encoder_left_sim is not None and self.encoder_right_sim is not None:
                self.encoder_left_sim.setDistance(0.0)
                self.encoder_right_sim.setDistance(0.0)
                self.left_distance = 0.0
                self.right_distance = 0.0
            return 0.0, 0.0, True

    def set_intake_angle(self, angle):
        """
        Define o ângulo da garra (intake).
        Parâmetros:
        - angle: Ângulo alvo em graus (ex.: 0.0 ou 90.0)
        Retorna:
        - Tuple (velocidade, rotacao, completado)
        """
        if self.robot is not None:
            self.robot.intake_angle = angle
            self.robot.intake_ligament.setAngle(angle - 90.0)  # Ajusta para visualização
            SmartDashboard.putNumber("Intake Angle", angle)
        return 0.0, 0.0, True

    def set_climber_length(self, length):
        """
        Define o comprimento do climber.
        Parâmetros:
        - length: Comprimento alvo em metros (ex.: 0.8, 1.2, 1.7, 2.4)
        Retorna:
        - Tuple (velocidade, rotacao, completado)
        """
        if self.robot is not None:
            self.robot.climber_length = length
            self.robot.climber_ligament.setLength(length)
            SmartDashboard.putNumber("Climber Length", length)
        return 0.0, 0.0, True

    def set_led_color(self, color):
        """
        Define a cor da LED strip.
        Parâmetros:
        - color: Cor alvo ("red" ou "green")
        Retorna:
        - Tuple (velocidade, rotacao, completado)
        """
        if self.robot is not None:
            self.robot.led_color = color
            if color == "red":
                self.robot.led_square.setColor(wpilib.Color8Bit(wpilib.Color.kRed))
            else:  # green
                self.robot.led_square.setColor(wpilib.Color8Bit(wpilib.Color.kGreen))
            SmartDashboard.putString("LED Color", color)
        return 0.0, 0.0, True

    def run_sequence(self, sequence):
        """
        Executa uma sequência de ações autônomas.
        Parâmetros:
        - sequence: Lista de tuplas (ação, parâmetros), onde ação é 'move', 'turn', 'stop', 'move_sensor', 'set_intake', 'set_climber', 'set_led'
        Retorna:
        - Tuple (velocidade, rotacao)
        """
        if not sequence:
            return 0.0, 0.0

        if self.current_action_index >= len(sequence):
            return 0.0, 0.0

        action, params = sequence[self.current_action_index]
        if action == 'move':
            velocidade, rotacao, completado = self.move_distance(
                params.get('distance', 0.0),
                params.get('speed', 1.0),
                left_distance=self.left_distance,
                right_distance=self.right_distance,
                reset_encoders=params.get('reset_encoders', True)
            )
        elif action == 'turn':
            velocidade, rotacao, completado = self.turn_to_angle(
                params.get('angle', 0.0),
                params.get('speed', 0.5),
                current_heading=math.degrees(-self.heading),
                reset_gyro=params.get('reset_gyro', True)
            )
        elif action == 'stop':
            velocidade, rotacao, completado = self.stop()
        elif action == 'move_sensor':
            velocidade, rotacao, completado = self.move_to_sensor_distance(
                params.get('distance', 0.0),
                params.get('speed', 1.0),
                current_distance=self.distance_sensor,
                reset_encoders=params.get('reset_encoders', True)
            )
        elif action == 'set_intake':
            velocidade, rotacao, completado = self.set_intake_angle(
                params.get('angle', 90.0)
            )
        elif action == 'set_climber':
            velocidade, rotacao, completado = self.set_climber_length(
                params.get('length', 0.8)
            )
        elif action == 'set_led':
            velocidade, rotacao, completado = self.set_led_color(
                params.get('color', 'red')
            )
        elif action == 'wait':
            # Apenas aguarda pelo tempo especificado em 'duration'
            if 'start_time' not in params:
                params['start_time'] = time.time()
            elapsed = time.time() - params['start_time']
            duration = params.get('duration', 1.0)
            if elapsed < duration:
                velocidade, rotacao, completado = 0.0, 0.0, False
            else:
                velocidade, rotacao, completado = 0.0, 0.0, True            
        else:
            velocidade, rotacao, completado = 0.0, 0.0, True

        if completado:
            self.current_action_index += 1

        return velocidade, rotacao

    def update(self, raw_v, raw_r, dt=None):
        """
        Atualiza a simulação com base nos comandos do joystick ou ações autônomas.
        Parâmetros:
        - raw_v: Velocidade linear (de -1 a 1)
        - raw_r: Velocidade de rotação (de -1 a 1)
        - dt: Intervalo de tempo (opcional, calculado automaticamente se None)
        Retorna:
        - raw_v, raw_r, pose, heading, left_distance, right_distance, distance_sensor
        """
        current_time = time.time()
        if dt is None:
            dt = 0.02 if self._last_time is None else current_time - self._last_time
        self._last_time = current_time

        if self.encoder_left_sim is not None and abs(self.encoder_left_sim.getDistance()) < 1e-6:
            self.left_distance = 0.0
        if self.encoder_right_sim is not None and abs(self.encoder_right_sim.getDistance()) < 1e-6:
            self.right_distance = 0.0

        # Se uma sequência de ações estiver definida, usa run_sequence
        if self.action_sequence:
            raw_v, raw_r = self.run_sequence(self.action_sequence)
        # Caso contrário, usa os comandos existentes (para move_distance, turn_to_angle, stop)
        elif self.action_active:
            if self.action_type == 'move':
                raw_v, raw_r, _ = self.move_distance(
                    self.target_distance, 
                    speed=1.0, 
                    left_distance=self.left_distance, 
                    right_distance=self.right_distance
                )
            elif self.action_type == 'turn':
                raw_v, raw_r, _ = self.turn_to_angle(
                    self.target_angle, 
                    speed=0.5, 
                    current_heading=math.degrees(-self.heading)
                )
            elif self.action_type == 'stop':
                raw_v, raw_r, _ = self.stop()
            elif self.action_type == 'move_sensor':
                raw_v, raw_r, _ = self.move_to_sensor_distance(
                    self.target_distance,
                    speed=1.0,
                    current_distance=self.distance_sensor
                )

        raw_v = self.apply_deadband(self.clamp(raw_v, -1.0, 1.0))
        raw_r = self.apply_deadband(self.clamp(raw_r, -1.0, 1.0))

        linear_vel = raw_v * self.MAX_LINEAR_MPS
        angular_vel = raw_r * self.MAX_ANGULAR_RPS

        left_vel = linear_vel - (angular_vel * self.wheelbase / 2)
        right_vel = linear_vel + (angular_vel * self.wheelbase / 2)

        self.left_distance += left_vel * dt
        self.right_distance += right_vel * dt

        if self.encoder_left_sim is not None:
            self.encoder_left_sim.setDistance(self.left_distance)
        if self.encoder_right_sim is not None:
            self.encoder_right_sim.setDistance(self.right_distance)

        self.heading += angular_vel * dt
        self.field_heading += angular_vel * dt

        yaw_degrees = math.degrees(-self.heading)
        self.gyro_sim_yaw.set(yaw_degrees)

        pitch = random.gauss(0.0, 0.5)
        roll = random.gauss(0.0, 0.5)
        self.gyro_sim_pitch.set(pitch)
        self.gyro_sim_roll.set(roll)

        self.gyro_sim_rate_z.set(math.degrees(angular_vel))
        self.gyro_sim_rate_x.set(random.gauss(0.0, 0.1))
        self.gyro_sim_rate_y.set(random.gauss(0.0, 0.1))

        accel_x = (linear_vel - self.last_linear_vel) / dt if dt > 0 else 0.0
        accel_y = random.gauss(0.0, 0.1)
        accel_z = random.gauss(9.81, 0.05)
        self.gyro_sim_accel_x.set(accel_x + random.gauss(0.0, 0.05))
        self.gyro_sim_accel_y.set(accel_y)
        self.gyro_sim_accel_z.set(accel_z)
        self.last_linear_vel = linear_vel

        x, y = self.pose.X(), self.pose.Y()
        heading = self.field_heading
        distance_to_left = x
        distance_to_right = self.FIELD_LENGTH_X - x
        distance_to_bottom = y
        distance_to_top = self.FIELD_WIDTH_Y - y
        cos_heading = math.cos(heading)
        sin_heading = math.sin(heading)
        if cos_heading > 0:
            distance_x = distance_to_right / cos_heading if cos_heading > 0.01 else float('inf')
        else:
            distance_x = -distance_to_left / cos_heading if cos_heading < -0.01 else float('inf')
        if sin_heading > 0:
            distance_y = distance_to_top / sin_heading if sin_heading > 0.01 else float('inf')
        else:
            distance_y = -distance_to_bottom / sin_heading if sin_heading < -0.01 else float('inf')
        self.distance_sensor = min(distance_x, distance_y)
        if self.distance_sensor < 0 or math.isinf(self.distance_sensor):
            self.distance_sensor = 0.0

        dx = linear_vel * math.cos(self.field_heading) * dt
        dy = linear_vel * math.sin(self.field_heading) * dt

        heading_for_field = math.degrees(self.field_heading) % 360
        if heading_for_field > 180:
            heading_for_field -= 360
        self.pose = Pose2d(self.pose.X() + dx, self.pose.Y() + dy, Rotation2d.fromDegrees(heading_for_field))

        return raw_v, raw_r, self.pose, self.heading, self.left_distance, self.right_distance, self.distance_sensor

    def get_encoder_counts(self, side):
        distance = self.left_distance if side == 'left' else self.right_distance
        return int(distance * self.counts_per_meter)