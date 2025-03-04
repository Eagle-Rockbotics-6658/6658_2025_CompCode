from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath

from commands2.command import Command
from commands2 import TimedCommandRobot, CommandScheduler
import typing


from robotcontainer import RobotContainer

from wpilib import PowerDistribution, DataLogManager
from wpiutil.log import DoubleArrayLogEntry, DoubleLogEntry
from wpilib import SmartDashboard


class Robot(TimedCommandRobot):

    def robotInit(self) -> None:
        self.autonomousCommand: typing.Optional[Command] = None
        self.robotContainer = RobotContainer()
        self.autonomousCommand = None
        self.testCommand = None
        self.teleopInitCommand = None

        # DataLogManager.start()
        # self.PDH = PowerDistribution(9, PowerDistribution.ModuleType.kRev)
        # log = DataLogManager.getLog()
        # self.currentLogIndividuals = DoubleArrayLogEntry(log, "/U/logs/Current")
        # self.currentLogAll = DoubleLogEntry(log, "/U/logs/TotalCurrent")
        # self.voltageLog = DoubleLogEntry(log, "/U/logs/Voltage")
        
    def robotPeriodic(self):
        CommandScheduler.getInstance().run()

        # self.currentLogIndividuals.append(self.PDH.getAllCurrents())
        # self.currentLogAll.append(self.PDH.getTotalCurrent())
        # self.voltageLog.append(self.PDH.getVoltage())

        return super().robotPeriodic()
        
    def testInit(self) -> None:
        self.testCommand = self.robotContainer.getTestCommand()
        if self.testCommand:
            self.testCommand.schedule()

    def autonomousInit(self):
        self.autonomousCommand = self.robotContainer.getAutonomousCommand()
        if self.autonomousCommand:
            self.autonomousCommand.schedule()
        
    def teleopInit(self) -> None:
        if self.autonomousCommand:
            self.autonomousCommand.cancel()
        if self.testCommand:
            self.testCommand.cancel()
        self.teleopInitCommand = self.robotContainer.getPreTeleopCommand()
        if self.teleopInitCommand:
            self.teleopInitCommand.schedule()
