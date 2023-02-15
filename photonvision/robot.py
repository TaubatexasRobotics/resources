import wpilib
import photonvision
import ntcore

class PhotonVisionTest(wpilib.TimedRobot):
    def robotInit(self) -> None:
        #nt = ntcore.NetworkTableInstance.getDefault()

        #self.nt.setServerTeam(7459)

        self.camera = photonvision.PhotonCamera('Microsoft_LifeCam_HD-3000')

    def robotPeriodic(self) -> None:
        self.result = self.camera.getLatestResult()
        
        if self.result.hasTargets():
            print('it worked!')
        else:
            print('it did not work')
        #print(self.result.getTargets())
       

if __name__ == '__main__':
    wpilib.run(PhotonVisionTest)