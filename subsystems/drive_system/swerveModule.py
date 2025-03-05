from rev import SparkMax, SparkLowLevel
from phoenix6.hardware import CANcoder
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from constants import SwerveModuleConstants as c
from wpilib.sysid import SysIdRoutineLog
from wpilib import RobotController

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
        
        # set up driving motor, encoder, and controller
        self.drivingSparkMax = SparkMax(drivingCANId, SparkLowLevel.MotorType.kBrushless)
        self.drivingSparkMax.configure(c.drivingMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters)
        self.drivingSparkMax.setInverted(reversedDrive)
        
        self.drivingEncoder = self.drivingSparkMax.getEncoder()
        self.drivingEncoder.setPosition(0.0)
        
        self.drivingController = self.drivingSparkMax.getClosedLoopController()
        
        
        # set up turning motor, encoder, and controller
        self.turningSparkMax = SparkMax(turningCANId, SparkLowLevel.MotorType.kBrushless)
        self.turningSparkMax.configure(c.turningMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters)
        self.turningSparkMax.setInverted(reversedSteer)
        
        self.turningEncoder = self.turningSparkMax.getEncoder()
        self.turningEncoder.setPosition(CANcoder(encoderNum).get_absolute_position())
        
        self.turningController = self.turningSparkMax.getClosedLoopController()
                
    def log(self, sys_id_routine: SysIdRoutineLog) -> None:
        """_summary_

        **Args**:
            `sys_id_routine` (SysIdRoutineLog): _description_
        """
        sys_id_routine.motor("drive-motor").voltage(
            self.drivingSparkMax.get() * RobotController.getBatteryVoltage()
        ).position(self.getPosition().distance).velocity(
            self.getState().speed
        )
        
    def getCurrentRotation(self) -> Rotation2d:
        """Gets the current rotation of the module

        **Returns**:
            `Rotation2d`: Rotation of the module
        """
        return Rotation2d.fromRotations(self.turningEncoder.getPosition())
    
    def resetEncoders(self) -> None:
        """Resets the rotation and driving encoders to 0"""
        self.drivingEncoder.setPosition(0.0)
        self.turningEncoder.setPosition(0.0)
                
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

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the state of the module

        **Args**:
            `desiredState` (SwerveModuleState): The desired state of the module
        """
        desiredState.optimize(self.getCurrentRotation())
        self.turningController.setReference(desiredState.angle.radians(), SparkMax.ControlType.kMAXMotionPositionControl)
        self.drivingController.setReference(desiredState.speed / c.drivingPosFactor, SparkMax.ControlType.kMAXMotionVelocityControl)
