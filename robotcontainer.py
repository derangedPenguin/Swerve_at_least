import wpilib

import phoenix5.sensors

import commands2

from subsystems import *
from commands import *

class RobotContainer:

    ignoreSim = True

    def __init__(self) -> None:
        #Tunables

        #subsytems
        swerveModules = None
        gyro = None

        if not wpilib.RobotBase.isSimulation() or self.ignoreSim:
            swerveModules = [
                SwerveModule( "FL", 0, 1, 9, 0.0 ),
                SwerveModule( "FR", 2, 3, 9, 0.0 ),
                SwerveModule( "BL", 4, 5, 9, 0.0 ),
                SwerveModule( "BR", 6, 7, 9, 0.0 ),
            ]
            gyro = phoenix5.sensors.WPI_Pigeon2( 9 , 'rio')
        else:
            swerveModules = [
                #sim modules
            ]
            gyro = ...
        
        self.swerve = SwerveDrive( swerveModules, gyro )

        #Controls
        self.controller = wpilib.XboxController(0)

        #commands
        self.swerve.setDefaultCommand(
            DriveByStick(
                self.swerve,
                self.controller.getLeftX,
                self.controller.getLeftY,
                lambda: (self.controller.getLeftTriggerAxis() - self.controller.getRightTriggerAxis())
            )
        )

        #put everything on SmartDashboard