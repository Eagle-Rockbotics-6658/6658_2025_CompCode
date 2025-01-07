from photonlibpy import PhotonCamera
from photonlibpy.targeting import PhotonPipelineResult, PhotonTrackedTarget

class VisionCamera():
    
    def __init__(self, camera_name: str):
        self.camera = PhotonCamera(camera_name)
        
    def _getResults(self) -> PhotonPipelineResult | None:
        resultsList = self.camera.getAllUnreadResults()
        if len(resultsList) > 0:
            return self.camera.getAllUnreadResults()[-1]
    
    def getID(self, id_num: int) -> PhotonTrackedTarget | None:
        """
        Returns the PhotonTrackedTarget of the requested id, or None if id isn't found
        """
        results = self._getResults()
        if results is not None:
            for target in results.getTargets() or []:
                if target.fiducialId == id_num:
                    return target
        return None