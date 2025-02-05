from rev import SparkMax, SparkLowLevel
from phoenix6.hardware import CANcoder
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.controller import PIDController
from constants import SwerveModuleConstants as c
from rev import SparkMaxConfig


class SwerveModule:
    r"""
    Class for controlling a single swerve module
    
    **Parameters**:
        `drivingCANId`: The CAN ID of the driving motor
        `turningCANId`: The CAN ID of the turning motor
        `encoderNum`: The CAN ID of the encoder
        `reversedDrive`: Whether the driving motor is reversed
        `reversedSteer`: Whether the turning motor is reversed
    
    **Methods**:
    - `getCurrentRotation` - Get current rotation of the module
    - `resetEncoders` - Resets the rotation and driving encoders to 0
    - `getState` - Get current state of the module
    - `getPosition` - Get current position of the module
    - `setDesiredState` - Sets the state of the module
    """
    def __init__(self, drivingCANId: int, turningCANId: int, encoderNum: int, reversedDrive: bool, reversedSteer: bool) -> None:
        
        # create driving motor
        self.drivingSparkMax = SparkMax(drivingCANId, SparkLowLevel.MotorType.kBrushless)
        # configure driving motor
        drivingMotorConfig = SparkMaxConfig()
        drivingMotorConfig.encoder.positionConversionFactor(c.drivingPosFactor).velocityConversionFactor(c.drivingVelFactor).uvwMeasurementPeriod(c.drivingEncoderMeasurementPeriod)
        drivingMotorConfig.setIdleMode(c.drivingIdleMode)
        drivingMotorConfig.closedLoop.outputRange(c.drivingMinOutput, c.drivingMaxOutput).pidf(*c.drivingPIDF)
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
        self.turningSparkMax.configure(turningMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters)
        self.turningSparkMax.setInverted(reversedSteer)
        # create turning encoder
        self.turningEncoder = CANcoder(encoderNum)
        
        # create turning PID Controller and enable continuous input
        self.turningPIDController = PIDController(*c.turningPID)
        self.turningPIDController.enableContinuousInput(c.turnEncoderMin, c.turnEncoderMax)
                
    # def log(self, sys_id_routine: SysIdRoutineLog) -> None:
    #     """_summary_

    #     **Args**:
    #         `sys_id_routine` (SysIdRoutineLog): _description_
    #     """
    #     sys_id_routine.motor("drive-motor").voltage(
    #         self.drivingSparkMax.get() * RobotController.getBatteryVoltage()
    #     ).position(self.getPosition().distance).velocity(
    #         self.getState().speed
    #     )
        
    def getCurrentRotation(self) -> Rotation2d:
        """Gets the current rotation of the module

        **Returns**:
            `Rotation2d`: Rotation of the module
        """
        return Rotation2d.fromRotations(self.turningEncoder.get_absolute_position().value_as_double)
    
    def resetEncoders(self) -> None:
        """Resets the rotation and driving encoders to 0"""
        self.drivingEncoder.setPosition(0.0)
        self.turningEncoder.set_position(0.0)
                
    def getState(self) -> SwerveModuleState:
        """Get current state of the module
        
        **Returns**:
            `SwerveModuleState`: The current state of the module
        """
        return SwerveModuleState(self.drivingEncoder.getVelocity(), self.getCurrentRotation())
        
    def getPosition(self) -> SwerveModulePosition:
        """
        Get current position of the module
        
        **Returns**:
            `SwerveModulePosition`: The current position of the module
        """
        return SwerveModulePosition(self.drivingEncoder.getPosition(),self.getCurrentRotation())

    def setDesiredState(self, desiredState: SwerveModuleState):
        """Sets the state of the module

        **Args**:
            `desiredState` (SwerveModuleState): The desired state of the module
        """
        desiredState.optimize(self.getCurrentRotation())
        
        # turning
        self.turningSparkMax.set(
            -self.turningPIDController.calculate(
                self.getCurrentRotation().radians(), 
                desiredState.angle.radians()
            )
        )

        # driving
        self.drivingClosedLoopController.setReference(desiredState.speed, SparkMax.ControlType.kMAXMotionVelocityControl)
