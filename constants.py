from rev import SparkMaxConfig
from wpimath.units import inchesToMeters
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d
from math import pi

from wpilib import SmartDashboard

class SwerveModuleConstants():
    drivingPosFactor = (.09 * pi) / 6.75  # motor to wheel conversion factor * circumference, meters
    drivingVelFactor = drivingPosFactor / 60.0  # meters per second
    drivingIdleMode = SparkMaxConfig.IdleMode.kBrake
    
    drivingMotorConfig = SparkMaxConfig()
    drivingMotorConfig.encoder.positionConversionFactor(drivingPosFactor).velocityConversionFactor(drivingVelFactor).uvwMeasurementPeriod(16)
    drivingMotorConfig.setIdleMode(drivingIdleMode)
    
    drivingPID = (0.048519 * (180/pi) * 2.0 * 0.0254, 0, .008)
    drivingSVA = (0.164, 0.12592 * (180/pi) * 2.0 * 0.0254 * 6, 0.16283 * (180/pi) * 2.0 * 0.0254 * 4)
    drivingMinOutput = -1.0
    drivingMaxOutput = 1.0
    
    
    turningIdleMode = SparkMaxConfig.IdleMode.kBrake
    
    turningMotorConfig = SparkMaxConfig()
    turningMotorConfig.setIdleMode(turningIdleMode)
    
    turningPID = (1.0, 0, 0.000)
    wheelDiameter = .09

    turnEncoderMin = 0.0
    turnEncoderMax = 2 * pi
    
class DriveConstants():
    deadband = 0.07
    FLDrivingCAN = 4
    FRDrivingCAN = 8
    RLDrivingCAN = 2
    RRDrivingCAN = 6

    FLTurningCAN = 3
    FRTurningCAN = 7
    RLTurningCAN = 1
    RRTurningCAN = 5

    FLEncoderCAN = 12
    FREncoderCAN = 10
    RLEncoderCAN = 13
    RREncoderCAN = 11

    PigeonGyro = 14
    
    halfTrackWidth = inchesToMeters(26)/2
    halfWheelBase = inchesToMeters(26)/2
    
    kinematics = SwerveDrive4Kinematics(
        Translation2d(halfWheelBase, halfTrackWidth),
        Translation2d(halfWheelBase, -halfTrackWidth),
        Translation2d(-halfWheelBase, halfTrackWidth),
        Translation2d(-halfWheelBase, -halfTrackWidth),
    )
    
    MaxSpeed = 3.8  # 3.8 meters per second
    
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
    translationPID = (5, 0, 0)
    SmartDashboard.putNumberArray("translationPID", translationPID)

    rotationPID = (5.00, 0, 0.00)

class SubsystemConstants():
    class Algae:
        intakeCanID = 17
        pivotCanID = 18
        intakeMotorConfig = SparkMaxConfig().smartCurrentLimit(3)
        pivotMotorConfig = SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        canCoderId = 0
        endSwitchInputId = 0
        
        #in radians
        inPoint = 0.25
        outPoint = 0.02

        pivotPower = 0
        intakePowerIn = -0.25
        intakePowerOut = 0.25
        intakePowerStalled = 0.1


        #In rotations
        pivotStartRotations = 0.32
        
        pivotGearRatio = -3/125

        kP = 1.75
        kI = 0
        kD = 0.01

        kS = 0
        kV = 0
        kG = 0.1
        kA = 0
    
    class Climb:
        leftMotorId = 0
        rightMotorId = 0
        motorConfig = SparkMaxConfig()

        encoderId = 0
        cutoff = 0
        pwr = 0