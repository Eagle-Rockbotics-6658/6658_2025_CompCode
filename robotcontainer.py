from subsystems.drive_system.swerveDrive import SwerveDrive
from subsystems.algaeSubsystem import AlgaeSubsystem

from constants import DriveConstants as d
from constants import robotConstants as c
from constants import SubsystemConstants as s

from wpilib import Joystick, SmartDashboard
from pathplannerlib.auto import AutoBuilder

from commands2.button import JoystickButton
from commands2 import Command
from commands2.cmd import runOnce, run

from math import copysign

from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Pose2d


class RobotContainer:
    
    def __init__(self):
        self.drive = SwerveDrive()
        self.driveStick = Joystick(c.joystickID)

        self.algaeSystem = AlgaeSubsystem()

        # Build an auto chooser. This will use Commands.none() as the default option.
        self.autoChooser = AutoBuilder.buildAutoChooser()

        # Another option that allows you to specify the default auto by its name
        # self.autoChooser = AutoBuilder.buildAutoChooser("My Default Auto")
        SmartDashboard.putData("Auto Chooser", self.autoChooser)
        
        self.configureButtonBindings()
        self.drive.setDefaultCommand(run(lambda: self.drive.driveFieldRelative((ChassisSpeeds(-self.getJoystickDeadband(1)/2, -self.getJoystickDeadband(0)/2, -self.getJoystickDeadband(4)/2))), self.drive))
        
    def configureButtonBindings(self) -> None:
        JoystickButton(self.driveStick, 3).whileTrue(run(self.drive.setX, self.drive))
        JoystickButton(self.driveStick, 1).onTrue(runOnce(self.drive.zeroHeading, self.drive))
        JoystickButton(self.driveStick, 2).onTrue(runOnce(lambda: self.algaeSystem.resetEncoder(s.Algae.pivotStartRotations/s.Algae.pivotGearRatio), self.algaeSystem))
        JoystickButton(self.driveStick, 4).onTrue(runOnce(self.algaeSystem.toggleExtended, self.algaeSystem))
        # JoystickButton(self.driveStick, 4).whileTrue(run(lambda: self.drive.pathFindToPose(Pose2d(0, 0, 0)), self.drive))
    
    def getJoystickDeadband(self, axis: int) -> float:
        rawAxis = self.driveStick.getRawAxis(axis)
        if(abs(rawAxis) <= d.deadband):
            return 0
        else:
            rawAxis -= d.deadband * copysign(1, rawAxis)
            rawAxis *= 1/(1-d.deadband)
            return rawAxis

    def getAutonomousCommand(self) -> Command:
        return self.autoChooser.getSelected()
