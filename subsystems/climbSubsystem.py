from commands2.subsystem import Subsystem
from constants import SubsystemConstants as sc
from rev import SparkLowLevel, SparkMax
from wpilib import SmartDashboard, DriverStation

from wpilib import DataLogManager
from wpiutil.log import DoubleLogEntry

class ClimbSubsystem(Subsystem):
    def __init__(self):
        self.climbMotorLeft = SparkMax(sc.Climb.leftMotorId, SparkLowLevel.MotorType.kBrushless)
        self.climbMotorRight = SparkMax(sc.Climb.rightMotorId, SparkLowLevel.MotorType.kBrushless)

        self.climbMotorLeft.configure(sc.Climb.motorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters)
        self.climbMotorRight.configure(sc.Climb.motorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters)

        self.leftEncoder = self.climbMotorLeft.getEncoder()
        self.rightEncoder = self.climbMotorRight.getEncoder()

        self.leftEncoder.setPosition(0)
        self.rightEncoder.setPosition(0)

        # logging
        log = DataLogManager.getLog()
        self.leftLog = DoubleLogEntry(log, "/U/logs/ClimbCurrentLeft")
        self.rightLog = DoubleLogEntry(log, "/U/logs/ClimbCurrentRight")
        self.leftEncoderPositionLog = DoubleLogEntry(log, "/U/logs/CLimbLeftEncoder")
        self.rightEncoderPositionLog = DoubleLogEntry(log, "/U/logs/CLimbRightEncoder")
    
    def resetEncoders(self) -> None:
        self.leftEncoder.setPosition(0)
        self.rightEncoder.setPosition(0)
    
    def getRightPosition(self) -> float:
        return self.rightEncoder.getPosition()

    def getLeftPosition(self) -> float:
        return self.leftEncoder.getPosition()

    def periodic(self):
        self.rightLog.append(self.climbMotorRight.getOutputCurrent())
        self.leftLog.append(self.climbMotorLeft.getOutputCurrent())
        self.leftEncoderPositionLog.append(self.getLeftPosition())
        self.rightEncoderPositionLog.append(self.getRightPosition())
        # SmartDashboard.putNumber("Left climb", self.getLeftPosition())
        # SmartDashboard.putNumber("Right climb", self.getRightPosition())
        # SmartDashboard.putNumber("Left Current", self.climbMotorLeft.getOutputCurrent())
        # SmartDashboard.putNumber("Right Current", self.climbMotorRight.getOutputCurrent())
    
    def runMotors(self, power: float) -> None:
        if abs(power) < .1:
            self.climbMotorLeft.set(0)
            self.climbMotorRight.set(0)
        else:
            
            power = (power - .1) * 10 / 9

            if abs(self.getRightPosition()) < sc.Climb.cutoff:
                self.climbMotorRight.set(power)
            else:
                self.climbMotorRight.set(0)

            if abs(self.getLeftPosition()) < sc.Climb.cutoff:
                self.climbMotorLeft.set(power)
            else:
                self.climbMotorLeft.set(0)

    # def run(self):
    #     if abs(self.climbEncoder.get_absolute_position()) >= sc.Climb.cutoff:
    #         self.climbMotorRight.set(0)
    #         self.climbMotorLeft.set(0)
    #     else:
    #         self.climbMotorLeft.set(sc.Climb.pwr)
    #         self.climbMotorRight.set(-sc.Climb.pwr)
    