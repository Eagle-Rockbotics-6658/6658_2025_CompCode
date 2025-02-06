from commands2 import Command

from wpilib import Timer
from wpiutil.log import DoubleArrayLogEntry

from subsystems.drive_system.swerveDrive import SwerveDrive

class QuasitasticRoutine(Command):
        
        def __init__(self, drive: SwerveDrive, forward: bool, log: DoubleArrayLogEntry):
            self.timer = Timer()
            self.timer.start()
            
            self.drive = drive
            self.forward = forward
            self.log = log
            
            super().__init__()
            
        def execute(self):
            if self.timer.get() > 4:
                self.drive.voltageControl(0)
                return
            if self.forward:
                values = self.drive.voltageControl(12)
            else:
                values = self.drive.voltageControl(-12)
            self.log.append(values, self.timer.get())
            return super().execute()
        
        def isFinished(self):
            return self.timer.get() > 4

class DynamicRoutine(Command):
        
        def __init__(self, drive: SwerveDrive, forward: bool, log: DoubleArrayLogEntry):
            self.timer = Timer()
            self.timer.start()
            
            self.drive = drive
            self.forward = forward
            self.log = log
            
            super().__init__()
            
        def execute(self):
            if self.timer.get() > 6:
                self.drive.voltageControl(0)
                return
            if self.forward:
                values = self.drive.voltageControl(self.timer.get())
            else:
                values = self.drive.voltageControl(-12)
            self.log.append(values, self.timer.get())
        
        def isFinished(self):
            return self.timer.get() > 8