from rev import SparkMaxConfig, SparkFlexConfig
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
    drivingMotorConfig.smartCurrentLimit(50)
    
    drivingPID = (0.048519 * (180/pi) * 2.0 * 0.0254, 0, .016)
    drivingSVA = (0.164, 0.12592 * (180/pi) * 2.0 * 0.0254 * 6, 0.16283 * (180/pi) * 2.0 * 0.0254 * 4)
    drivingMinOutput = -1.0
    drivingMaxOutput = 1.0
    
    
    turningIdleMode = SparkMaxConfig.IdleMode.kBrake
    
    turningMotorConfig = SparkMaxConfig()
    turningMotorConfig.smartCurrentLimit(50)
    turningMotorConfig.setIdleMode(turningIdleMode)
    
    turningPID = (1, 0, 0.000)
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
    
    halfTrackWidth = inchesToMeters(24.75)/2
    halfWheelBase = inchesToMeters(24.75)/2
    
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
    driveStickID = 0
    helperStickID = 1

class PathPlannerConstants():
    translationPID = (5, 0, 0)

    rotationPID = (5.00, 0, 0.00)

class SubsystemConstants():
    class Algae:
        intakeCanID = 16
        pivotCanID = 18
        intakeMotorConfig = SparkFlexConfig().smartCurrentLimit(40)
        pivotMotorConfig = SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        canCoderId = 0
        endSwitchInputId = 0
        
        #in rotations
        stowedPoint = 0.25
        groundPickupPoint = 0.00
        coralPickupPoint = 0.13

        pivotPower = 0
        intakePowerIn = -0.25
        intakePowerOut = .25
        intakePowerStalled = 0  # -0.1


        #In rotations
        pivotStartRotations = 0.3
        
        pivotGearRatio = -3/125

        kP = 1.75
        kI = 0
        kD = 0.01

        kS = 0
        kV = 0
        kG = 0.1
        kA = 0
    
    class Climb:
        leftMotorId = 15
        rightMotorId = 17
        motorConfig = SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast)

        encoderId = 0
        cutoff = 34
        pwr = 0
    
    class Coral:
        intakeSlightIn = .1
        intakeOut = -1
        intakeMotorID = 19