from commands2 import Subsystem
from rev import SparkMax, SparkLowLevel
from phoenix6.hardware import CANcoder
from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController, ArmFeedforward


class CoralMechanism(Subsystem):
    
    def __init__(self):
        self.armMotor = SparkMax()
        super().__init__()