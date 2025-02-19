from rev import SparkMax, SparkLowLevel
from constants import SubsystemConstants as sc
from wpilib import DigitalInput
from wpimath.controller import PIDController as PID
from wpimath.controller import ArmFeedforward
from commands2.command import Command
from commands2.subsystem import Subsystem
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

class AlgaeSubsystem(Subsystem):
    def __init__(self):
        #motor that runs the intake rollers
        self.wheelMotor = SparkMax(sc.Algae.wheelCanID, SparkLowLevel.MotorType.kBrushless)
        #motor that runs the deploy and retract of the mechanism
        self.rotateMotor = SparkMax(sc.Algae.rotateCanID, SparkLowLevel.MotorType.kBrushless)
        
        self.wheelMotor.configure(sc.Algae.wheelMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters)
        self.rotateMotor.configure(sc.Algae.rotateMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters)
        
        #encoder which tells us what angle the mechanism is at.
        self.rotateEncoder = CANcoder(sc.Algae.canCoderId)
        self.rotateEncoder.set_position(0)

        #endswitch which is triggered when a ball goes in
        self.endSwitch = DigitalInput(sc.Algae.endSwitchInputId)
        self.rotateController = PID(sc.Algae.kP, sc.Algae.kI, sc.Algae.kD)
        self.rotateFeedForward = ArmFeedforward(sc.Algae.kS, sc.Algae.kG, sc.Algae.kV, sc.Algae.kA)
        self.isExtended = False

        self.foldCommand = FoldCommand(rotateMotor=self.rotateMotor, rotateController=self.rotateController, rotateEncoder=self.rotateEncoder, rotateFF=self.rotateFeedForward, isOpening=False)


    #Run when button to intake ball is held. will not run if there is already a ball in the intake
    def intake(self):
        if self.isBallIntook():
            return
        self._runIntake(1)
    
    #Run when the button to push balls out is held.
    def runOut(self):
        self._runIntake(-1)

    #gets if there is a ball fully in the intake
    def isBallIntook(self) -> bool:
        return self.endSwitch.get()

    def teleopPeriodic(self) -> None:
        self._runIntake(sc.Algae.intakeBasePower)
   
    def _runIntake(self, pwr: float):
        self.wheelMotor.set(pwr*sc.Algae.intakePower)

    def toggleExtended(self):
        if self.foldCommand.isScheduled():
            self.foldCommand.toggleInOut()
        else:
            self.foldCommand.schedule()
            self.foldCommand.toggleInOut()


class FoldCommand(Command):
    def __init__(self, rotateMotor: SparkMax, rotateController:PID, rotateEncoder: CANcoder, rotateFF: ArmFeedforward, isOpening: bool):
        self.rotateMotor = rotateMotor
        self.rotateFF = rotateFF
        self.rotateController = rotateController
        self.rotateEncoder = rotateEncoder
        self.isOpening = isOpening
        if self.isOpening:
            self.rotateController.setSetpoint(sc.Algae.outPoint)
        else:
            self.rotateController.setSetpoint(sc.Algae.inPoint)
    
    def toggleInOut(self):
        if self.isOpening:
            self.rotateController.setSetpoint(sc.Algae.inPoint)
        else:
            self.rotateController.setSetpoint(sc.Algae.outPoint)
        self.isOpening = not self.isOpening
    def execute(self):
        angle = self.rotateEncoder.get_absolute_position()*2*pi
        velocity = self.rotateEncoder.get_velocity()*2*pi
        pwr = self.rotateController.calculate(angle) + self.rotateFF.calculate(angle,velocity)
        self.rotateMotor.set(pwr)
        if self.rotateController.atSetpoint():
            self.end(True)
    def end(self, interrupted: bool):
        self.rotateMotor.set(0)
        super().end(self, interrupted)
    def runsWhenDisabled(self):
        return False
        




