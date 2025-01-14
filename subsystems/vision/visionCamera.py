from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy
from photonlibpy.targeting import PhotonPipelineResult, PhotonTrackedTarget
from photonlibpy.photonPoseEstimator import EstimatedRobotPose
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from constants import PhotonVisionConstants as c
from wpimath.kinematics import ChassisSpeeds
from wpimath.controller import PIDController

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
        
        self.robotToTagCenteringPIDController = PIDController(*c.RobotToTagCenteringPID)
        
    def periodic(self):
        """Function that needs to be run periodically"""
        self.resultsList = self.camera.getAllUnreadResults()
        
    def _getResults(self) -> (PhotonPipelineResult | None):
        if len(self.resultsList) > 0:
            return self.camera.getAllUnreadResults()[-1]
    
    def getID(self, id_num: int) -> (PhotonTrackedTarget | None):
        """Returns information on the requested apriltag, or None if id isn't found

        **Args**:
            `id_num` (int): The id number of the requested apriltag

        Returns:
            `(PhotonTrackedTarget | None)`: The `PhotonTrackedTarget` if the requested id is found, otherwise None
        """
        results = self._getResults()
        if results is not None:
            for target in results.getTargets() or []:
                if target.fiducialId == id_num:
                    return target
        return None
    
    def getEstimatedPose(self) -> (EstimatedRobotPose | None):
        """Gets the estimated pose of the robot

        Returns:
            `(EstimatedRobotPose | None)`: The `EstimatedRobotPose` of the robot, returns `None` if no targets are found
        """
        return self.camPoseEst.update()
    
    def centerRobotOnTag(self, tag_id: int) -> ChassisSpeeds:
        """Returns a `ChassisSpeeds` object containing the direction for the robot to move relative to the robot (NOT field relative)

        Args:
            `tag_id` (int): The id of the tag to center on

        Returns:
            `ChassisSpeeds`: the `ChassisSpeeds` object containing the direction for the robot to move relative to the robot (NOT field relative)
        """
        offset = self.robotToTagCenteringPIDController.calculate(self.getID(tag_id).getYaw(), 0)
        return ChassisSpeeds(offset, 0, 0)
