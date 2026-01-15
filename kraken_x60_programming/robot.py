
import wpilib 
import phoenix6
from wpilib import SmartDashboard

class Robot(wpilib.TimedRobot):
    def robotInit(self):
       self.kraken = phoenix6.hardware.TalonFX(20, "rio")
       self.joystick = wpilib.Joystick(0)
       self.last_print_time = 0
       self.print_interval = 3
       self.kraken.set_position(0)  
       
       config = phoenix6.configs.TalonFXConfiguration()
       config.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE #Kraken Brake Mode on Configuration
       self.kraken.configurator.apply(config)
       print("Iniciar teste kraken")

    
        

    def teleopPeriodic(self):
        if self.joystick.getRawButton(1):
            self.kraken.set(0.8)
            current_time = wpilib.Timer.getFPGATimestamp()
            '''
            if current_time - self.last_print_time >= self.print_interval:
                print("Kraken ativado")
                self.last_print_time = current_time 

                '''
            position = self.kraken.get_position().value
            velocidade = self.kraken.get_velocity().value    
            SmartDashboard.putNumber("Kraken Position", position)
            SmartDashboard.putNumber("Kraken Velocity", velocidade)
            

        elif self.joystick.getRawButton(2):
            self.kraken.set(-0.8)
            current_time = wpilib.Timer.getFPGATimestamp()
            position = self.kraken.get_position().value
            velocidade = self.kraken.get_velocity().value    
            SmartDashboard.putNumber("Kraken Position", position)
            SmartDashboard.putNumber("Kraken Velocity", velocidade)
                       
            if current_time - self.last_print_time >= self.print_interval:
                print("Kraken reverso ativado")
                self.last_print_time = current_time
        

        else:

            self.kraken.set(0)
            
            current_time = wpilib.Timer.getFPGATimestamp()
            if current_time - self.last_print_time >= self.print_interval:
                print("Kraken desativado")
                self.last_print_time = current_time #teste
    
    def simulationPeriodic(self):
        output = self.kraken.get()
        print(f"Kraken Output: {output}")

 