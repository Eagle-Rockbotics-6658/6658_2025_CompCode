from rev import SparkMax, SparkLowLevel, SparkRelativeEncoder
from constants import SubsystemConstants as sc
from wpilib import DigitalInput
from wpimath.controller import PIDController as PID
from wpimath.controller import ArmFeedforward
from commands2.command import Command
from commands2.subsystem import Subsystem
from phoenix6.hardware import CANcoder
from math import pi
from wpilib import SmartDashboard

# #this is henry's fault. he has a twisted mind
# class TrueBool:
#     def __init__(self):
#         self.true = True

#     def is_true(self):
#         if self.is_true:
#             return True
#         else:
#             if self.is_true != True and self.is_true is False:
#                 return False


class AlgaeIntake(Subsystem):

    def __init__(self):
        #motor that runs the intake rollers
        self.intakeMotor = SparkMax(sc.Algae.intakeCanID, SparkLowLevel.MotorType.kBrushless)
        super().__init__()
    
    # #Run when button to intake ball is held. will not run if there is already a ball in the intake
    # def intake(self):
    #     self._runIntake(1)
    
    # #Run when the button to push balls out is held.
    # def runOut(self):
    #     self._runIntake(-1)
    
    # def stopIntake(self):
    #     self._runIntake(0)
    
    def _runIntake(self, pwr: float):
        self.intakeMotor.set(pwr)
        
    
    def controlIntake(self, runOutButton: bool, runInButton: bool):
        if runOutButton:
            self._runIntake(sc.Algae.intakePowerOut)
        elif runInButton:
            self._runIntake(sc.Algae.intakePowerIn)
        else:
            self._runIntake(sc.Algae.intakePowerStalled)

class AlgaePivot(Subsystem):
    def __init__(self):
        #motor that runs the deploy and retract of the mechanism
        self.pivotMotor = SparkMax(sc.Algae.pivotCanID, SparkLowLevel.MotorType.kBrushless)
        
        # self.intakeMotor.configure(sc.Algae.intakeMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters)
        self.pivotMotor.configure(sc.Algae.pivotMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters)
        
        #encoder which tells us what angle the mechanism is at.
        self.pivotEncoder = self.pivotMotor.getEncoder()
        self.resetEncoder(sc.Algae.pivotStartRotations/sc.Algae.pivotGearRatio)
        self.pivotController = PID(sc.Algae.kP, sc.Algae.kI, sc.Algae.kD)
        self.pivotFeedForward = ArmFeedforward(sc.Algae.kS, sc.Algae.kG, sc.Algae.kV, sc.Algae.kA)
        # endswitch which is triggered when a ball goes in
        self.isExtended = False

        self.foldCommand = FoldCommand(pivotMotor=self.pivotMotor, pivotController=self.pivotController, pivotEncoder=self.pivotEncoder, pivotFF=self.pivotFeedForward, isOpening=False)

    def teleopPeriodic(self) -> None:
        # self._runIntake(sc.Algae.intakeBasePower)
        angle = self.pivotEncoder.getPosition()*2*pi*sc.Algae.pivotGearRatio
        angularVelocity = self.pivotEncoder.getVelocity()*2*pi*sc.Algae.pivotGearRatio
        pwr = -self.pivotFeedForward.calculate(angle, angularVelocity)
        self.pivotMotor.set(pwr)

    def toggleExtended(self):
        if self.foldCommand.isScheduled():
            self.foldCommand.toggleInOut()
        else:
            self.foldCommand.schedule()
            self.foldCommand.toggleInOut()
    
    def resetEncoder(self, position = 0) -> None:
        self.pivotEncoder.setPosition(position)
    
    def getPivotPosition(self) -> float:
        return self.pivotEncoder.getPosition()*2*pi*sc.Algae.pivotGearRatio
    
    def getPivotVelocity(self) -> float:
        return self.pivotEncoder.getVelocity()*2*pi*sc.Algae.pivotGearRatio

    def periodic(self):
        SmartDashboard.putNumber("Algae pivot position", self.pivotEncoder.getPosition()*sc.Algae.pivotGearRatio)
        return super().periodic()
    
    # def runFeedForward(self, positionChange: float):
    #     self.pivotMotor.set(0)
        
    #     SmartDashboard.putNumber("Algae Pivot Position", self.getPivotPosition()/(2*pi))

class FoldCommand(Command):
    def __init__(self, pivotMotor: SparkMax, pivotController:PID, pivotEncoder: SparkRelativeEncoder, pivotFF: ArmFeedforward, isOpening: bool):
        self.pivotMotor = pivotMotor
        self.pivotFF = pivotFF
        self.pivotController = pivotController
        self.pivotEncoder = pivotEncoder
        self.isOpening = isOpening
        if self.isOpening:
            self.pivotController.setSetpoint(sc.Algae.outPoint)
        else:
            self.pivotController.setSetpoint(sc.Algae.inPoint)
    
    def toggleInOut(self):
        if self.isOpening:
            self.pivotController.setSetpoint(sc.Algae.inPoint)
        else:
            self.pivotController.setSetpoint(sc.Algae.outPoint)
        self.isOpening = not self.isOpening
    def execute(self):
        angle = self.pivotEncoder.getPosition()*2*pi*sc.Algae.pivotGearRatio
        angularVelocity = self.pivotEncoder.getVelocity()*2*pi*sc.Algae.pivotGearRatio
        pwr = -self.pivotController.calculate(angle/(2*pi)) - self.pivotFF.calculate(angle, angularVelocity)
        self.pivotMotor.set(pwr)
    def end(self, interrupted: bool):
        self.pivotMotor.set(0)
    def runsWhenDisabled(self):
        return False
    
# class IntakeCommand(Command):
#     def __init__(self, parent: AlgaeSubsystem):
#         self.parent = parent
#         self.running = 
#     def        
