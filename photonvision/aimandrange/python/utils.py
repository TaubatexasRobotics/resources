from math import tan


class PhotonUtils:
    @staticmethod
    def calculateDistanceToTargetMeters(
        cameraHeightMeters: float,
        targetHeightMeters: float,
        cameraPitchRadians: float,
        targetPitchRadians: float,
    ) -> float:
        return (targetHeightMeters - cameraHeightMeters) / tan(
            cameraPitchRadians + targetPitchRadians
        )
