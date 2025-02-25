from ntcore import NetworkTableInstance
from commands2 import Subsystem, Command, InstantCommand
from commands2.cmd import run
from phoenix6.hardware import Pigeon2
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from math import pi
from typing import Callable

class LimeLight(Subsystem):
    
    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("rock")

    def sendRobotOrientationCommand(self, gyro: Pigeon2) -> Command:
        run(lambda: self.table.getEntry("robot_orientation_set").setDoubleArray(gyro.get_yaw().value_as_double, gyro.get_angular_velocity_z_device().value_as_double, gyro.get_pitch().value_as_double, gyro.get_angular_velocity_x_device().value_as_double, gyro.get_roll().value_as_double, gyro.get_angular_velocity_y_device().value_as_double), LimeLight)

    def getRobotPose(self) -> Pose2d | None:
        """Get the robot pose based on visible april tags

        **Returns**:
            `Pose2d | None`: The `Pose2d` of the robot on the field, or None if no tags available
        """
        val = self.table.getEntry("botpose_orb").getDoubleArray(None)
        # if entry does not exist, or number of visible ids is zero, return None
        if val is None or val[7] == 0:
            return None
        return Pose2d(Translation2d(val[0], val[1]), Rotation2d(val[5] * pi / 180))
    
    def setFiducialIdFilter(self, id_filters: list[float]) -> bool:
        """Sets the Fiducial ID Filter for the limelight

        **Args**:
            `id_filters` (list[float]): Override valid fiducial ids for localization with input floats

        **Returns**:
            `bool`: _description_
        """
        return self.table.getEntry("fiducial_id_filters_set").setDoubleArray(id_filters)
