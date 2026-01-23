from pathplannerlib.auto import PathPlannerAuto
from DriveTrain import DriveSubsystem

class RobotContainer:

    #Inicia o subSystem
    def __init__(self):
        self.drive = DriveSubsystem()

    #Retorna um command de auto do pathPLanner
    def getAuto(self):
        return PathPlannerAuto("New Auto")