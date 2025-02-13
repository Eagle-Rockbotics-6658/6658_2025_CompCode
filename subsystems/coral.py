from commands2 import Subsystem
from rev import SparkMax, SparkLowLevel, SparkMaxConfig
from phoenix6.hardware import CANcoder
from wpimath.geometry import Rotation2d
from wpimath.units import radians_per_second, rotationsToRadians
from wpimath.controller import PIDController, ArmFeedforward


class CoralMechanism(Subsystem):
    
    def __init__(self):
        self.pivotMotor = SparkMax(5, SparkLowLevel.MotorType.kBrushless)
        pivotConfig = SparkMaxConfig()
        self.pivotMotor.configure(pivotConfig)
        
        self.intakeMotor = SparkMax(5, SparkLowLevel.MotorType.kBrushless)
        intakeConfig = SparkMaxConfig()
        self.intakeMotor.configure(intakeConfig)
        
        self.pivotPIDController = PIDController(0, 0, 0)
        self.pivotFeedForward = ArmFeedforward(0, 0)
        
        self.pivotEncoder = CANcoder(0)
        super().__init__()
    
    def getPivotPosition(self) -> Rotation2d:
        return Rotation2d.fromRotations(self.pivotEncoder.get_absolute_position().value_as_double)
    
    def getPivotVelocity(self) -> float:
        return rotationsToRadians(self.pivotEncoder.get_velocity().value_as_double)
    
    def setPivotPosition(self, position: Rotation2d) -> None:
        self.pivotMotor.setVoltage(
            self.pivotPIDController.calculate(self.getPivotPosition().radians(), position.radians()) + \
                self.pivotFeedForward.calculate(self.getPivotPosition().radians(), self.getPivotVelocity())
        )
    
    def setIntakeSpeed(self, speed) -> None:
        self.intakeMotor.set(speed)
