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
                SwerveModule("FL", 7, 8, 18, 97.471 ),
                SwerveModule("FR", 1, 2, 12, 5.361 ),
                SwerveModule("BL", 5, 6, 16, 298.828 ),
                SwerveModule("BR", 3, 4, 14, 60.557 )
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
        wpilib.SmartDashboard.putData('ServeDrive', self.swerve)