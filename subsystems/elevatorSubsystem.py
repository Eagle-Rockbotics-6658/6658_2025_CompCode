from rev import SparkFlex, SparkLowLevel 
from commands2 import Subsystem
from constants import SubsystemConstants as sc
from wpimath.controller import PIDController
from wpilib import SmartDashboard

# class ElevatorPositions(Enum):
#     TIER1 = 1, 100
#     TIER2 = 2, 0
#     # INTAKE = 3, -100
#     # TIER3 = 4, -50
#     # TIER4 = 5, -100

#     def __new__(cls, value, position):
#         obj = object.__new__(cls)
#         obj._value_ = value
#         obj.position = position
#         return obj

#     def up(self):
#         position = self.value
#         position += 1
#         if position > 2:
#             return ElevatorPositions.TIER2
#         print(ElevatorPositions(position))
#         return ElevatorPositions(position)

#     def down(self):
#         position = self.value
#         position -= 1
#         if position < 1:
#             return ElevatorPositions.TIER1
#         return ElevatorPositions(position)
    
class Elevator(Subsystem):

    # negative is up

    def __init__(self):
        self.elevatorMotor = SparkFlex(sc.Elevator.motorID, SparkLowLevel.MotorType.kBrushless)
        self.elevatorEncoder = self.elevatorMotor.getEncoder()
        self.elevatorMotor.configure(sc.Elevator.elevatorConfig, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kNoPersistParameters)
        self.elevatorPID = PIDController(sc.Elevator.elevatorP, sc.Elevator.elevatorI, sc.Elevator.elevatorD)

        self.targetPosition = sc.Elevator.tier1

        SmartDashboard.putData("elevator PID", self.elevatorPID)

        super().__init__()
    
    def getPosition(self) -> float:
        return self.elevatorEncoder.getPosition()
    
    def periodic(self):
        SmartDashboard.putNumber("Elevator position", self.getPosition())

    def resetEncoder(self) -> None:
        self.elevatorEncoder.setPosition(0)
    
    def run(self, power: float):
        self.elevatorMotor.set(power)
        self.runElevator()
    
    def runElevator(self):
        self.elevatorPositionSet(self.targetPosition)
    
    def elevatorPositionSet(self, desiredPosition: float):
        if desiredPosition > sc.Elevator.elevatorMax: 
            desiredPosition = sc.Elevator.elevatorMax
        if desiredPosition < sc.Elevator.elevatorMin:
            desiredPosition = sc.Elevator.elevatorMin
        SmartDashboard.putNumber("Elevator target position", desiredPosition)
        SmartDashboard.putNumber("Elevator power", self.elevatorPID.calculate(self.getPosition(), desiredPosition))
        self.elevatorMotor.set(self.elevatorPID.calculate(self.getPosition(), desiredPosition))
    
    def toggleElevatorPosition(self):
        if self.targetPosition == sc.Elevator.tier1:
            self.targetPosition = sc.Elevator.tier2
        else:
            self.targetPosition = sc.Elevator.tier1