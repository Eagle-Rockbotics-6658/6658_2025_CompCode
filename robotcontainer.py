from subsystems.drive_system.swerveDrive import SwerveDrive
# from subsystems.coral import CoralMechanism
# from subsystems.algaeSubsystem import AlgaePivot, AlgaeIntake
from subsystems.climbSubsystem import ClimbSubsystem

from constants import DriveConstants as d
from constants import robotConstants as c
from constants import SubsystemConstants as s

from wpilib import Joystick, SmartDashboard
from wpilib.cameraserver import CameraServer as CS

from pathplannerlib.auto import AutoBuilder, NamedCommands

from commands2.button import JoystickButton
from commands2 import Command
from commands2.cmd import runOnce, run, runEnd, parallel, none

from math import copysign

from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Pose2d


class RobotContainer:
    
    def __init__(self):
        # CS.launch("vision.py")

        self.drive = SwerveDrive()
        self.driveStick = Joystick(c.driveStickID)
        self.helperStick = Joystick(c.helperStickID)

        # self.algaePivot = AlgaePivot()
        # self.algaeIntake = AlgaeIntake()
        self.climbMechanism = ClimbSubsystem()

        # self.coralMechanism = CoralMechanism()

        # register commands
        # NamedCommands.registerCommand("Algae To Coral Pickup", runOnce(self.algaePivot.foldCommand.toCoralPickup))
        # NamedCommands.registerCommand("Algae To Stowed", runOnce(self.algaePivot.toggleExtended))
        # NamedCommands.registerCommand("Algae In", run(lambda: self.algaeIntake.controlIntake(False, True), self.algaeIntake))
        # NamedCommands.registerCommand("Algae Out", run(lambda: self.algaeIntake.controlIntake(True, False), self.algaeIntake))
        # NamedCommands.registerCommand("Algae Slight In", run(lambda: self.algaeIntake.controlIntake(False, False), self.algaeIntake))
        NamedCommands.registerCommand("Zero Heading", runOnce(lambda: self.drive.zeroHeading(0), self.drive))
        # NamedCommands.registerCommand("Coral Slight In", run(lambda: self.coralMechanism.setIntakeSpeed(s.Coral.intakeSlightIn), self.coralMechanism))
        # NamedCommands.registerCommand("Coral Out", run(lambda: self.coralMechanism.setIntakeSpeed(s.Coral.intakeOut), self.coralMechanism))
        # NamedCommands.registerCommand("Coral Off", run(lambda: self.coralMechanism.setIntakeSpeed(0), self.coralMechanism))
        NamedCommands.registerCommand("Zero Heading to 180", runOnce(lambda: self.drive.zeroHeading(180), self.drive))

        # NamedCommands.registerCommand("Algae Intake Off", run(lambda: self.algaeIntake._runIntake(0), self.algaeIntake))
        # NamedCommands.registerCommand("Algae Pivot Off", run(lambda: self.algaePivot.pivotMotor.set(0), self.algaePivot))
        NamedCommands.registerCommand("Climb Off", run(self.climbMechanism.runMotors(0)))

        # Build an auto chooser. This will use Commands.none() as the default option.
        self.autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.autoChooser)
        
        self.configureButtonBindings()
        self.drive.setDefaultCommand(run(lambda: self.drive.driveFieldRelative((ChassisSpeeds(-self.getJoystickDeadband(1), -self.getJoystickDeadband(0), -self.getJoystickDeadband(4)))), self.drive))
        # self.algaeIntake.setDefaultCommand(run(lambda: self.algaeIntake.controlIntake(self.helperStick.getRawButton(5), self.helperStick.getRawButton(6)), self.algaeIntake))
        self.climbMechanism.setDefaultCommand(run(lambda: self.climbMechanism.runMotors(-abs(self.helperStick.getRawAxis(1)/4)), self.climbMechanism))
        # self.coralMechanism.setDefaultCommand(run(lambda: self.coralMechanism.setIntakeSpeed(self.helperStick.getRawAxis(5)), self.coralMechanism))

    def getPreTeleopCommand(self) -> Command:
        return runOnce(self.climbMechanism.resetEncoders, self.climbMechanism)
        
    def configureButtonBindings(self) -> None:
        # JoystickButton(self.driveStick, 3).whileTrue(run(self.drive.setX, self.drive))
        JoystickButton(self.driveStick, 1).onTrue(runOnce(self.drive.zeroHeading, self.drive))
        JoystickButton(self.driveStick, 2).onTrue(runOnce(lambda: self.drive.resetPose(Pose2d()), self.drive))
        # JoystickButton(self.helperStick, 4).onTrue(runOnce(self.algaePivot.toggleExtended, self.algaePivot))
        # JoystickButton(self.driveStick, 7).onTrue(runOnce(self.drive.
        # tryToLimelight, self.drive))
        # JoystickButton(self.driveStick, 4).whileTrue(run(lambda: self.drive.pathFindToPose(Pose2d(0, 0, 0)), self.drive))
    
    def getJoystickDeadband(self, axis: int) -> float:
        rawAxis = self.driveStick.getRawAxis(axis)
        if(abs(rawAxis) <= d.deadband):
            return 0
        else:
            rawAxis -= d.deadband * copysign(1, rawAxis)
            rawAxis *= 1/(1-d.deadband)
            return rawAxis
    
    def getTestCommand(self) -> Command:
        return None
        # return parallel(run(lambda: self.climbMechanism.runMotors(self.helperStick.getRawAxis(5)), lambda: self.climbMechanism), run(lambda: self.coralMechanism.setIntakeSpeed(self.helperStick.getRawAxis(1)), self.coralMechanism))

    def getAutonomousCommand(self) -> Command:
        return self.autoChooser.getSelected()
        # autoCommand: Command = self.autoChooser.getSelected()
        # return autoCommand.beforeStarting(runOnce(self.drive.resetOdometryToLimelight, self.drive))
