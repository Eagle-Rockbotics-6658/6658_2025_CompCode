from rev import SparkMaxConfig
from wpimath.units import inchesToMeters
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d, Transform3d, Translation3d, Rotation3d
from math import pi

class SwerveModuleConstants():
    drivingPosFactor = (.09 * pi) / 6.75  # motor to wheel conversion factor * circumference, meters
    drivingVelFactor = drivingPosFactor / 60.0  # meters per second
    drivingIdleMode = SparkMaxConfig.IdleMode.kBrake
    
    drivingMotorConfig = SparkMaxConfig()
    drivingMotorConfig.encoder.positionConversionFactor(drivingPosFactor).velocityConversionFactor(drivingVelFactor).uvwMeasurementPeriod(16)
    drivingMotorConfig.setIdleMode(drivingIdleMode)
    
    drivingPID = (.04, 0, .008)
    drivingSVA = (0, 6.102634556313851, 0)
    drivingMinOutput = -1.0
    drivingMaxOutput = 1.0
    
    
    turningIdleMode = SparkMaxConfig.IdleMode.kBrake
    
    turningMotorConfig = SparkMaxConfig()
    turningMotorConfig.setIdleMode(turningIdleMode)
    
    turningPID = (0.16, 0, 0.008)
    wheelDiameter = .09

    turnEncoderMin = 0.0
    turnEncoderMax = 2 * pi
    
class DriveConstants():
    deadband = 0.07
    FLDrivingCAN = 4
    FRDrivingCAN = 6
    RLDrivingCAN = 8
    RRDrivingCAN = 2

    FLTurningCAN = 3
    FRTurningCAN = 5
    RLTurningCAN = 7
    RRTurningCAN = 1

    FLEncoderCAN = 10
    FREncoderCAN = 13
    RLEncoderCAN = 12
    RREncoderCAN = 11

    PigeonGyro = 14
    
    halfTrackWidth = inchesToMeters(58)/2
    halfWheelBase = inchesToMeters(58)/2
    
    kinematics = SwerveDrive4Kinematics(
        Translation2d(halfWheelBase, halfTrackWidth),
        Translation2d(halfWheelBase, -halfTrackWidth),
        Translation2d(-halfWheelBase, halfTrackWidth),
        Translation2d(-halfWheelBase, -halfTrackWidth),
    )
    
    MaxSpeed = 2.0  # 3.8 meters per second
    
class IntakeConstants():
    angleID = 21
    powerID = 22
    encoderID = 23
    pitchP = 1
    pitchI = 0
    pitchD = 0
    pitchTolerance = 5
    kPitchKG = .07

class robotConstants():
    joystickID = 0

class PathPlannerConstants():
    translationPID = (2.00, 0, -0.1)
    rotationPID = (-2.00, 0, 0.05)

class PhotonVisionConstants():
    RobotToCam = Transform3d(
        Translation3d(0, 0, 0),
        Rotation3d.fromDegrees(0, 0, 0)
    )
    
    RobotToTagCenteringPID = (1, 0, 0)