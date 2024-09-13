from wpimath import applyDeadband

from commands2 import Command

from subsystems import SwerveDrive

from typing import Callable

class DriveByStick(Command):

    controllerDeadband = 0.02

    def __init__(self,
                 swerveDrive:SwerveDrive,
                 velocityX:Callable,
                 velocityY:Callable,
                 rotation:Callable
                 ):
        self.setName("DriveByStick")
        self.addRequirements(swerveDrive)

        self.drive = swerveDrive
        self.vX=velocityX
        self.vY=velocityY
        self.rO=rotation #rotation = Omega
    
    def execute(self):
        xSpeed = applyDeadband(self.vX(), self.controllerDeadband) * self.drive.kMaxSpeed
        ySpeed = applyDeadband(self.vY(), self.controllerDeadband) * self.drive.kMaxSpeed
        rotSpeed = applyDeadband(self.rO(), self.controllerDeadband) * self.drive.kMaxSpeed

        self.drive.drive(xSpeed, ySpeed, rotSpeed, True)
