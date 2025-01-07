from wpilib import Joystick, SmartDashboard
from subsystems.drive_system.swerveDrive import SwerveDrive
from constants import robotConstants as c
from wpimath.kinematics import ChassisSpeeds
from constants import DriveConstants as d
from math import copysign

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath

from commands2.command import Command
from commands2 import TimedCommandRobot
import typing


class Robot(TimedCommandRobot):

    autonomousCommand: typing.Optional[Command] = None

    def getJoystickDeadband(self, axis: int) -> float:
        rawAxis = self.driveStick.getRawAxis(axis)
        if(abs(rawAxis) <= d.deadband):
            return 0
        else:
            rawAxis -= d.deadband * copysign(1, rawAxis)
            rawAxis *= 1/(1-d.deadband)
            return rawAxis
    
    def robotInit(self) -> None:
        
        self.drive = SwerveDrive()
        self.driveStick = Joystick(c.joystickID)

        # Build an auto chooser. This will use Commands.none() as the default option.
        self.autoChooser = AutoBuilder.buildAutoChooser()

        # Another option that allows you to specify the default auto by its name
        # self.autoChooser = AutoBuilder.buildAutoChooser("My Default Auto")

        SmartDashboard.putData("Auto Chooser", self.autoChooser)
        
    def teleopPeriodic(self) -> None:
        self.drive.driveFieldRelative(ChassisSpeeds(-self.getJoystickDeadband(1)/2, -self.getJoystickDeadband(0)/2, -self.getJoystickDeadband(4)/2))
        if self.driveStick.getRawButtonPressed(1):
            self.drive.zeroHeading()
        
    def testInit(self) -> None:
        pass
        
    def testPeriodic(self) -> None:
        pass
    
    def autonomousInit(self):
        self.autonomousCommand = self.getAutonomousCommand()
        if self.autonomousCommand:
            self.autonomousCommand.schedule()
        
    def teleopInit(self) -> None:
        if self.autonomousCommand:
            self.autonomousCommand.cancel()
    
    def autonomousPeriodic(self):
        pass
    
    def getAutonomousCommand(self):
       return self.autoChooser.getSelected()
   
    def getAutonomousPathFollow(self) -> Command:
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Example Path"))
