from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy
from photonlibpy.targeting import PhotonPipelineResult, PhotonTrackedTarget
from photonlibpy.photonPoseEstimator import EstimatedRobotPose
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from constants import PhotonVisionConstants as c

class VisionCamera():
    
    def __init__(self, camera_name: str):
        self.camera = PhotonCamera(camera_name)
        self.camPoseEst = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.kDefaultField),
            PoseStrategy.LOWEST_AMBIGUITY,
            self.camera,
            c.RobotToCam,
        )
        self.resultsList = []
        
    def periodic(self):
        self.resultsList = self.camera.getAllUnreadResults()
        
    def _getResults(self) -> (PhotonPipelineResult | None):
        if len(self.resultsList) > 0:
            return self.camera.getAllUnreadResults()[-1]
    
    def getID(self, id_num: int) -> (PhotonTrackedTarget | None):
        """
        Returns the PhotonTrackedTarget of the requested id, or None if id isn't found
        """
        results = self._getResults()
        if results is not None:
            for target in results.getTargets() or []:
                if target.fiducialId == id_num:
                    return target
        return None
    
    def getEstimatedPose(self) -> (EstimatedRobotPose | None):
        return self.camPoseEst.update()
