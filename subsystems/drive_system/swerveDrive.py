from subsystems.drive_system.swerveModule import SwerveModule
from subsystems.vision.limelight import LimeLight

from constants import DriveConstants as c
from constants import PathPlannerConstants as p
from phoenix6.hardware import Pigeon2
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Rotation2d, Pose2d
from ntcore import NetworkTableInstance
from wpilib import DriverStation

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants, RobotConfig

from pathplannerlib.path import PathConstraints
from wpimath.units import degreesToRadians
from commands2.command import Command
from wpilib import SmartDashboard
from wpilib import Timer
from wpilib import PowerDistribution

from commands2 import Subsystem
from typing import Callable


class SwerveDrive(Subsystem):
    r"""
    Class for controlling a single swerve module
    
    **Methods**:
    - `getHeading` - Get the heading of the robot
    - `getModuleStates` - Get the state of each module
    - `getModulePositions` - Get the position of each module
    - `zeroHeading` - Zero the heading of the robot
    - `resetPose` - Reset the estimated position of the robot on the field
    - `getPose` - Get the estimated position of the robot on the field
    - `getRobotRelativeSpeeds` - Get the robot relative ChassisSpeeds of the robot
    - `resetEncoders` - Reset the encoders of all the modules
    - `setModuleStates` - Set the desired states of the swerve modules
    - `driveFieldRelative` - Drive the robot using field relative speeds
    - `driveRobotRelative` - Drive the robot using robot relative speeds
    - `setX` - Set modules into an X configuration to prevent movement
    """
    def __init__(self) -> None:
        self.moduleFL = SwerveModule(c.FLDrivingCAN, c.FLTurningCAN, c.FLEncoderCAN, False, False)
        self.moduleFR = SwerveModule(c.FRDrivingCAN, c.FRTurningCAN, c.FREncoderCAN, True, False)
        self.moduleRL = SwerveModule(c.RLDrivingCAN, c.RLTurningCAN, c.RLEncoderCAN, False, False)
        self.moduleRR = SwerveModule(c.RRDrivingCAN, c.RRTurningCAN, c.RREncoderCAN, True, False)
        
        self.swerveModuleArray = [self.moduleFL, self.moduleFR, self.moduleRL, self.moduleRR]
        
        self.gyro = Pigeon2(c.PigeonGyro)
        
        self._publishStates()
                
        self.odometry = SwerveDrive4PoseEstimator(
            c.kinematics,
            self.getHeading(),
            (
                self.moduleFL.getPosition(),
                self.moduleFR.getPosition(),
                self.moduleRL.getPosition(),
                self.moduleRR.getPosition(),
            ),
            Pose2d()
        )
        
        # Load the RobotConfig from the GUI settings. You should probably
        # store this in your Constants file
        config = RobotConfig.fromGUISettings()


        AutoBuilder.configure(
            self.getPose,
            self.resetPose,
            self.getRobotRelativeSpeeds,
            lambda speeds, feedforwards: self.driveRobotRelative(speeds),
            PPHolonomicDriveController(
                PIDConstants(*p.translationPID),
                PIDConstants(*p.rotationPID)
            ),
            config,
            self._shouldFlipPath,
            self
        )
        
        self.pathFindingConstraints = PathConstraints(3.0, 5.0, degreesToRadians(540), degreesToRadians(720))

        # self.limelight = LimeLight()
        # self.limelight.sendRobotOrientationCommand(self.gyro).schedule()
    
    def periodic(self):
        # if DriverStation.isTeleop():
        #     poseAndLatency = self.limelight.getRobotPoseAndLatency()
        #     if poseAndLatency != None:
        #         self.odometry.addVisionMeasurement(poseAndLatency[0], Timer.getTimestamp() - (poseAndLatency[1] / 1000))
        # SmartDashboard.putNumberArray("turning current", [self.moduleFL.turningSparkMax.getOutputCurrent(), self.moduleFR.turningSparkMax.getOutputCurrent(), self.moduleRL.turningSparkMax.getOutputCurrent(), self.moduleRR.turningSparkMax.getOutputCurrent()])
        return super().periodic()
    
    # def resetOdometryToLimelight(self):
    #     pose = self.limelight.getRobotPose()
    #     if pose != None:
    #         self.odometry.resetPose(pose)
        
    def _shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
        
    def _publishStates(self) -> None:
        """Publishes the estimated robot state to the driverstation"""
        self.OdometryPublisher = NetworkTableInstance.getDefault().getStructTopic("/SwerveStates/Odometry", Pose2d).publish()
        self.ObservedPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates/Observed", SwerveModuleState).publish()
        self.DesiredPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates/Desired", SwerveModuleState).publish()

    def _updateStates(self, desiredStates: tuple[SwerveModuleState]) -> None:
        self.OdometryPublisher.set(self.odometry.getEstimatedPosition())
        self.ObservedPublisher.set(self.getModuleStates())
        self.DesiredPublisher.set(desiredStates)

    def getHeading(self) -> Rotation2d:
        """Gets the heading of the robot

        **Returns**:
            `Rotation2d`: The heading of the robot
        """
        return Rotation2d.fromDegrees(self.gyro.get_yaw().value_as_double)
    
    def getModuleStates(self) -> list[SwerveModuleState]:
        """Gets a list containing the state of each module

        **Returns**:
            `list[SwerveModuleState]`: A list containing the state of each module
        """
        return [self.moduleFL.getState(), self.moduleFR.getState(), self.moduleRL.getState(), self.moduleRR.getState()]
    
    def getModulePositions(self) -> list[SwerveModulePosition]:
        """Gets a list containing the position of each module

        **Returns**:
            `list[SwerveModulePosition]`: A list containing the position of each module
        """
        return [self.moduleFL.getPosition(), self.moduleFR.getPosition(), self.moduleRL.getPosition(), self.moduleRR.getPosition()]
    
    def zeroHeading(self, degrees = 0) -> None:
        """Zeros the heading of the robot provided by the gyroscope"""
        self.gyro.set_yaw(0)
        
    def resetPose(self, pose: Pose2d = Pose2d()) -> None:
        """Resets the estimated position of the robot on the field to the specified pose, the position is (0, 0) if no argument is given

        **Args**:
            `pose` (Pose2d, optional): The position to set the robot odometry to. Defaults to Pose2d().
        """
        self.odometry.resetPosition(self.getHeading(), self.getModulePositions(), pose)
        
    def getPose(self) -> Pose2d:
        """Gets the position of the robot on the field

        **Returns**:
            `Pose2d`: The estimated position of the robot on the field
        """
        return self.odometry.getEstimatedPosition()
    
    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        """Gets the robot relative ChassisSpeeds of the robot

        **Returns**:
            `ChassisSpeeds`: The robot relative ChassisSpeeds of the robot
        """
        return c.kinematics.toChassisSpeeds(self.getModuleStates())
        
    def resetEncoders(self) -> None:
        """Resets the encoders of all the modules"""
        self.moduleFL.resetEncoders()
        self.moduleFR.resetEncoders()
        self.moduleRL.resetEncoders()
        self.moduleRR.resetEncoders()
        
    def setModuleStates(self, desiredStates: tuple[SwerveModuleState]) -> None:
        """Sets the desired states of the swerve modules

        **Args**:
            `desiredStates` (tuple[SwerveModuleState]): a 4 tuple containing the SwerveModuleState for each module
        """
        desiredStates = c.kinematics.desaturateWheelSpeeds(desiredStates, c.MaxSpeed)
        self.moduleFL.setDesiredState(desiredStates[0])
        self.moduleFR.setDesiredState(desiredStates[1])
        self.moduleRL.setDesiredState(desiredStates[2])
        self.moduleRR.setDesiredState(desiredStates[3])
        self.odometry.updateWithTime(
            Timer.getTimestamp(),
            self.getHeading(), 
            (
                self.moduleFL.getPosition(),
                self.moduleFR.getPosition(),
                self.moduleRL.getPosition(),
                self.moduleRR.getPosition()
            )
        )
        self._updateStates(desiredStates)
        
    def driveFieldRelative(self, chassisSpeeds: ChassisSpeeds) -> None:
        """Drives the robot using field relative speeds
        
        **Args**:
            `chassisSpeeds` (ChassisSpeeds): The ChassisSpeeds containing the direction to drive relative to the field
        """
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, self.getHeading())
        moduleStates = c.kinematics.toSwerveModuleStates(speeds)
        self.setModuleStates(moduleStates)
            
    def driveRobotRelative(self, chassisSpeeds: ChassisSpeeds) -> None:
        """Drives the robot using robot relative speeds
        
        **Args**:
            `chassisSpeeds` (ChassisSpeeds): The ChassisSpeeds containing the direction to drive relative to the robot
        """
        moduleStates = c.kinematics.toSwerveModuleStates(chassisSpeeds)
        self.setModuleStates(moduleStates)
        
    def setX(self) -> None:
        """Sets modules into an X configuration to prevent movement"""
        self.moduleFL.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.moduleFR.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.moduleRL.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.moduleRR.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
    
    def pathFindToPose(self, targetPose: Pose2d) -> Command:
        return AutoBuilder.pathfindToPose(
            targetPose,
            self.pathFindingConstraints,
            goal_end_vel=0.0, # Goal end velocity in meters/sec
            # rotation_delay_distance=0.0 # Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        )
    def goToPose(self, targetPose: Pose2d, vision: LimeLight) -> Command:
        h = PositionResetter(vision.getRobotPose, self.resetPose)
        return self.pathFindToPose(targetPose).beforeStarting(h)

class PositionResetter:
    def __init__(self, getFn: Callable[[], (Pose2d | None)], resetFn: Callable[[Pose2d], None]):
        self.getFn = getFn
        self.resetFn = resetFn
    def __call__(self):
        while self.getFn() == None:
            True
        self.resetFn(self.getFn)
