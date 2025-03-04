from rev import SparkFlex, SparkLowLevel 
from commands2 import Subsystem
from constants import SubsystemConstants as sc
from wpimath.controller import PIDController
from enum import Enum

class ElevatorPositions(Enum):
    TIER1 = (1, 0)
    TIER2 = (2, 25)
    INTAKE = (3, 30)
    TIER3 = (4, 50)
    TIER4 = (5, 75)
    
class Elevator(Subsystem):
    def __init__(self):
        self.elevatorMotor = SparkFlex(sc.Elevator.motorID, SparkLowLevel.MotorType.kBrushless)
        self.elevatorEncoder = self.elevatorMotor.getEncoder()
        self.elevatorMotor.configure(sc.Elevator.elevatorConfig, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kNoPersistParameters)
        self.elevatorPID = PIDController(sc.Elevator.elevatorP, sc.Elevator.elevatorI, sc.Elevator.elevatorD)
        self.currentPosition = 0
        self.state = ElevatorPositions.TIER1

        super().__init__()
    
    def getPosition(self) -> float:
        return self.elevatorEncoder.getPosition() 
    
    def elevatorPositionSet(self, desiredPosition: float):
        if desiredPosition > sc.Elevator.elevatorMax: 
            desiredPosition = sc.Elevator.elevatorMax
        if desiredPosition < sc.Elevator.elevatorMin:
            desiredPosition = sc.Elevator.elevatorMin
        self.elevatorMotor.set(self.elevatorPID.calculate(self.getPosition(), desiredPosition) + sc.Elevator.elevatorFF)
    def setState(self, goUp: bool, goDown: bool):
        if goUp: 
            self.state.value[0] += 1
        elif goDown:
            self.state.value[0] -= 1
        self.elevatorPositionSet(self.state.value[1])