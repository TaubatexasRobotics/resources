# TODO: insert robot code here
import wpilib
import rev
from wpimath.trajectory import TrapezoidProfile

class MyRobot(wpilib.TimedRobot):
        
        def robotInit(self):
                self.controle = wpilib.Joystick(0)
                self.motor1 = rev.SparkMax(53,rev.SparkLowLevel.MotorType.kBrushless)
                self.motor2 = rev.SparkMax(52,rev.SparkLowLevel.MotorType.kBrushless)
                self.constraints = TrapezoidProfile.Constraints(
                maxVelocity = 1.0,      
                maxAcceleration=5.0  
                )

                self.profile = TrapezoidProfile(self.constraints)

                self.currentState = TrapezoidProfile.State(0)

                #Velocidade que o motor irá reduzir antes de reverter o motor
                self.contrary_rotation = 0.5

                #Velocidade que o motor irá reduzir antes de freiar bruscamente
                self.min_speed_brake = 0.8

                #Limite da velocidade do motor
                self.max_motor_speed = 0.8

        def teleopPeriodic(self):

                #verifica se irá ocorrer reversão do motor em movimento
                if  ((self.currentState.position > 0 and self.controle.getRawAxis(0) < 0) or (self.currentState.position < 0 and self.controle.getRawAxis(0) > 0)):
                        if self.currentState.position > self.contrary_rotation or self.currentState.position < -self.contrary_rotation:        
                                goal = TrapezoidProfile.State(0)

                                dt = 0.02

                                self.currentState = self.profile.calculate(dt, self.currentState, goal)
                                
                                self.motor1.set((self.currentState.position*self.max_motor_speed))
                                self.motor2.set((self.currentState.position*self.max_motor_speed))
                        else:
                                self.motor1.set((0))
                                self.motor2.set((0))
                                self.currentState = TrapezoidProfile.State(0)

                #Dead zone que irá mandar freiar o robô
                elif self.controle.getRawAxis(0) > 0.2 or self.controle.getRawAxis(0) < - 0.2:
                        targetSpeed = self.controle.getRawAxis(0)  # -1 a 1
                        goal = TrapezoidProfile.State(targetSpeed)

                        dt = 0.02

                        self.currentState = self.profile.calculate(dt, self.currentState, goal)

                        self.motor1.set((self.currentState.position*self.max_motor_speed))
                        self.motor2.set((self.currentState.position*self.max_motor_speed))

                #Freio do robo
                else:
                        if self.currentState.position > self.min_speed_brake or self.currentState.position < -self.min_speed_brake:        
                                goal = TrapezoidProfile.State(0)

                                dt = 0.02

                                self.currentState = self.profile.calculate(dt, self.currentState, goal)

                                self.motor1.set((self.currentState.position*self.max_motor_speed))
                                self.motor2.set((self.currentState.position*self.max_motor_speed))
                        else:
                                self.motor1.set((0))
                                self.motor2.set((0))
                                self.currentState = TrapezoidProfile.State(0)
                                
                
                