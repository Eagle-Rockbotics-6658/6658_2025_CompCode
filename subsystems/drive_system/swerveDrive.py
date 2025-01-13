from subsystems.drive_system.swerveModule import SwerveModule
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

class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

class SwerveDrive(metaclass=Singleton):
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
        self.moduleFL = SwerveModule(c.FLDrivingCAN, c.FLTurningCAN, c.FLEncoderCAN, True, False)
        self.moduleFR = SwerveModule(c.FRDrivingCAN, c.FRTurningCAN, c.FREncoderCAN, False, False)
        self.moduleRL = SwerveModule(c.RLDrivingCAN, c.RLTurningCAN, c.RLEncoderCAN, False, False)
        self.moduleRR = SwerveModule(c.RRDrivingCAN, c.RRTurningCAN, c.RREncoderCAN, False, False)

        #
        self.lastDesiredSpeedFL = 0
        self.controlArray = []
        
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
                PIDConstants(p.translationP, p.translationI, p.translationD),
                PIDConstants(p.rotationP, p.rotationI, p.rotationD)
            ),
            config,
            self._shouldFlipPath,
            self
        )
        
    def _shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
        
    def _publishStates(self) -> None:
        """Publishes the estimated robot state to the driverstation"""
        self.OdometryPublisher = NetworkTableInstance.getDefault().getStructTopic("/SwerveStates/Odometry", Pose2d).publish()
        self.RedPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates/Red", SwerveModuleState).publish()

    def _updateStates(self) -> None:
        self.RedPublisher.set(self.getModuleStates())
        self.OdometryPublisher.set(self.odometry.getEstimatedPosition())
        
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
    
    def zeroHeading(self) -> None:
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
        self.odometry.update(
            self.getHeading(), 
            (
                self.moduleFL.getPosition(),
                self.moduleFR.getPosition(),
                self.moduleRL.getPosition(),
                self.moduleRR.getPosition()
            )
        )
        self._updateStates()
        if len(self.controlArray) <= 100000:
            self.controlArray.append((self.lastDesiredSpeedFL, self.moduleFL.drivingEncoder.getVelocity()))
        self.lastDesiredSpeedFL = desiredStates[0].speed
        
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
        