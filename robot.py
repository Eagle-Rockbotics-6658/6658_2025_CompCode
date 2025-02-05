from commands2.command import Command
from commands2 import TimedCommandRobot, CommandScheduler
import typing


from robotcontainer import RobotContainer


class Robot(TimedCommandRobot):

    def robotInit(self) -> None:
        self.autonomousCommand: typing.Optional[Command] = None
        self.robotContainer = RobotContainer()
        
    def robotPeriodic(self):
        CommandScheduler.getInstance().run()
        return super().robotPeriodic()
        
    def testInit(self) -> None:
        pass
        
    def testPeriodic(self) -> None:
        pass
    
    def autonomousInit(self):
        self.autonomousCommand = self.robotContainer.getAutonomousCommand()
        if self.autonomousCommand:
            self.autonomousCommand.schedule()
        
    def teleopInit(self) -> None:
        if self.autonomousCommand:
            self.autonomousCommand.cancel()
