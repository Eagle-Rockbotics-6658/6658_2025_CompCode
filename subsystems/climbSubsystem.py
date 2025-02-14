from commands2.subsystem import Subsystem
from constants import SubsystemConstants as sc
from rev import SparkLowLevel, SparkMax
from phoenix6.hardware import CANcoder

class ClimbSubsystem(Subsystem):
    def __init__(self):
        self.climbMotorLeft = SparkMax(sc.Climb.leftMotorId, SparkLowLevel.MotorType.kBrushless)
        self.climbMotorRight = SparkMax(sc.Climb.rightMotorId, SparkLowLevel.MotorType.kBrushless)

        self.climbMotorLeft.configure(sc.Climb.motorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters)
        self.climbMotorRight.configure(sc.Climb.motorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters)

        self.leftClimbEncoder = CANcoder(sc.Climb.leftEncoderId)
        self.rightClimbEncoder = CANcoder(sc.Climb.rightEncoderId)
        self.leftClimbEncoder.set_position(0)
        self.rightClimbEncoder.set_position(0)

    def run(self):
        if abs(self.leftClimbEncoder.get_absolute_position()) >= sc.Climb.cutoff or abs(self.rightClimbEncoder.get_absolute_position()) >= sc.Climb.cutoff:
            self.climbMotorRight.set(0)
            self.climbMotorLeft.set(0)
        else:
            self.climbMotorLeft.set(sc.Climb.pwr)
            self.climbMotorRight.set(-sc.Climb.pwr)
    