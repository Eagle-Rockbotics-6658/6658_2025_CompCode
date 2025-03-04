from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath

from commands2.command import Command
from commands2 import TimedCommandRobot, CommandScheduler
import typing

import cProfile, pstats, io
from pstats import SortKey


from robotcontainer import RobotContainer

from wpilib import PowerDistribution, DataLogManager
from wpiutil.log import DoubleArrayLogEntry, DoubleLogEntry
from wpilib import SmartDashboard, Joystick
from wpilib import Timer, DataLogManager, Joystick
from wpiutil.log import StringLogEntry


class Robot(TimedCommandRobot):

    def _stopLogging(self):
        self.prof.disable()
        self.timer.stop()
        s = io.StringIO()
        sortby = SortKey.CUMULATIVE
        ps = pstats.Stats(self.prof, stream=s).sort_stats(sortby)
        ps.print_stats()
        out = s.getvalue()
        self.dataLog.append(out)



    def robotInit(self) -> None:
        self.autonomousCommand: typing.Optional[Command] = None
        self.robotContainer = RobotContainer()
        self.autonomousCommand = None
        self.testCommand = None
        self.teleopInitCommand = None
        self.timer = Timer()

        self.joystick = Joystick(0)

        DataLogManager.start()
        log = DataLogManager.getLog()
        self.dataLog = StringLogEntry(log, "/U/logs/profiler")
        
        self.prof = cProfile.Profile(self.timer.get, 1)

        self.timer.start()
        self.prof.enable()

        # DataLogManager.start()
        # self.PDH = PowerDistribution(9, PowerDistribution.ModuleType.kRev)
        # log = DataLogManager.getLog()
        # self.currentLogIndividuals = DoubleArrayLogEntry(log, "/U/logs/Current")
        # self.currentLogAll = DoubleLogEntry(log, "/U/logs/TotalCurrent")
        # self.voltageLog = DoubleLogEntry(log, "/U/logs/Voltage")
        
    def robotPeriodic(self):
        CommandScheduler.getInstance().run()

        if self.joystick.getRawButtonPressed(5):
            self._stopLogging

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
