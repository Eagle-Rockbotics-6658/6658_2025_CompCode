from rev import SparkMax, SparkLowLevel
from constants import SubsystemConstants as sc
from wpilib import DigitalInput
from wpimath.controller import PIDController as PID
from wpimath.controller import ArmFeedforward
from commands2.command import Command
from phoenix6.hardware import CANcoder
from math import pi

#this is henry's fault. he has a twisted mind
class TrueBool:
    def __init__(self):
        self.true = True

    def is_true(self):
        if self.is_true:
            return True
        else:
            if self.is_true != True and self.is_true is False:
                return False

class AlgaeSubsystem:
    def __init__(self):
        self.wheelMotor = SparkMax(sc.Algae.wheelCanID, SparkLowLevel.MotorType.kBrushless)
        self.rotateMotor = SparkMax(sc.Algae.rotateCanID, SparkLowLevel.MotorType.kBrushless)
        
        self.wheelMotor.configure(sc.Algae.wheelMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters)
        self.rotateMotor.configure(sc.Algae.rotateMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters)
        
        self.rotateEncoder = CANcoder(sc.Algae.canCoderId)
        self.endSwitch = DigitalInput(sc.Algae.endSwitchInputId)
        self.rotateController = PID(sc.Algae.kP, sc.Algae.kI, sc.Algae.kD)
        self.rotateController.setSetpoint(sc.Algae.rotate)
        self.rotateFeedForward = ArmFeedforward(sc.Algae.kS, sc.Algae.kG, sc.Algae.kV, sc.Algae.kA)
        self.isExtended = False

        self.openCommand = OpenCommand(rotateMotor=self.rotateMotor, endSwitch=self.endSwitch)
        self.closeCommand = CloseCommand(rotateController=self.rotateController, rotateEncoder=self.rotateEncoder, rotateMotor=self.rotateMotor)
    
    def intake(self):
        self._runIntake(1)

    def teleopPeriodic(self) -> None:
        self._runIntake(sc.Algae.intakeBasePower)
   
    def _runIntake(self, pwr: float):
        self.wheelMotor.set(pwr*sc.Algae.intakePower)
        
    def toggleExtended(self):
        self.openCommand.end()
        self.closeCommand.end()
        if not self.isExtended:
            self.openCommand.schedule()
        else:
            self.closeCommand.schedule()

class OpenCommand(Command):

    def __init__(self, rotateMotor: SparkMax, endSwitch: DigitalInput):
        self.rotateMotor = rotateMotor
        self.endSwitch = endSwitch
    def execute(self):
        if self.endSwitch.get():
            self.end()
        else:
            self.rotateMotor.set(sc.Algae.rotatePower)
    def end(self):
        self.rotateMotor.set(0)
        super().end(self)
    def runsWhenDisabled(self):
        return False
class CloseCommand(Command):
    def __init__(self, rotateMotor: SparkMax, rotateController:PID, rotateEncoder: CANcoder, rotateFF: ArmFeedforward):
        self.rotateMotor = rotateMotor
        self.rotateFF = rotateFF
        self.rotateController = rotateController
        self.rotateEncoder = rotateEncoder
    def execute(self):
        pwr = self.rotateController.calculate(self.rotateEncoder.get_absolute_position()*2*(pi)) + self.rotateFF.calculate(sc.Algae.rotateSetpoint)
        self.rotateMotor.set(pwr)
        if self.rotateController.atSetpoint():
            self.end()
    def end(self):
        self.rotateMotor.set(0)
        super().end(self)
    def runsWhenDisabled(self):
        return False
        




