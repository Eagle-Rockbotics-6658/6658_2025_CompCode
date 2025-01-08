from rev import SparkMax, SparkLowLevel
from phoenix6.hardware import CANcoder
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from constants import SwerveModuleConstants as c
from wpilib.sysid import SysIdRoutineLog
from wpimath.units import volts
from wpilib import RobotController
from rev import SparkMaxConfig


class SwerveModule:
    
    def __init__(self, drivingCANId: int, turningCANId: int, encoderNum: int, reversedDrive: bool, reversedSteer: bool) -> None:
        
        # create driving motor
        self.drivingSparkMax = SparkMax(drivingCANId, SparkLowLevel.MotorType.kBrushless)
        # configure driving motor
        drivingMotorConfig = SparkMaxConfig()
        drivingMotorConfig.encoder.positionConversionFactor(c.drivingPosFactor).velocityConversionFactor(c.drivingVelFactor).uvwMeasurementPeriod(c.drivingEncoderMeasurementPeriod)
        drivingMotorConfig.setIdleMode(c.drivingIdleMode)
        drivingMotorConfig.closedLoop.outputRange(c.drivingMinOutput, c.drivingMaxOutput).pidf(c.drivingP, c.drivingI, c.drivingD, 1/c.drivingV)
        self.drivingSparkMax.configure(drivingMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters)
        self.drivingSparkMax.setInverted(reversedDrive)
        # get driving encoder
        self.drivingEncoder = self.drivingSparkMax.getEncoder()
        self.drivingEncoder.setPosition(0.0)
        # get driving closed loop controller
        self.drivingClosedLoopController = self.drivingSparkMax.getClosedLoopController()
        
        # set up turning motor
        self.turningSparkMax = SparkMax(turningCANId, SparkLowLevel.MotorType.kBrushless)
        # configure turning motor
        turningMotorConfig = SparkMaxConfig()
        turningMotorConfig.setIdleMode(c.turningIdleMode)
        turningMotorConfig.closedLoop.pid(c.turningP, c.turningI, c.turningD).positionWrappingEnabled(True).positionWrappingInputRange(c.turnEncoderMin, c.turnEncoderMax)
        self.turningSparkMax.configure(turningMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters)
        self.turningSparkMax.setInverted(reversedSteer)
        # get turning encoder
        self.turningEncoder = CANcoder(encoderNum)
        # get turning closed loop controller
        self.turningClosedLoopController = self.turningSparkMax.getClosedLoopController()
                        
    def log(self, sys_id_routine: SysIdRoutineLog) -> None:
        sys_id_routine.motor("drive-motor").voltage(
            self.drivingSparkMax.get() * RobotController.getBatteryVoltage()
        ).position(self.getPosition().distance).velocity(
            self.getState().speed
        )
        
    def getCurrentRotation(self) -> Rotation2d:
        return Rotation2d.fromRotations(self.turningEncoder.get_absolute_position().value_as_double)
    
    def resetEncoders(self):
        self.drivingEncoder.setPosition(0.0)
        self.turningEncoder.set_position(0.0)
                
    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.drivingEncoder.getVelocity(), self.getCurrentRotation())
        
    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.drivingEncoder.getPosition(),self.getCurrentRotation())

    def setDesiredState(self, desiredState: SwerveModuleState):
        desiredState.optimize(self.getCurrentRotation)
        self.turningClosedLoopController.setReference(desiredState.angle.radians(), SparkMax.ControlType.kMAXMotionPositionControl)
        self.drivingClosedLoopController.setReference(desiredState.speed, SparkMax.ControlType.kMAXMotionVelocityControl)
