import wpilib

from subsystems import *

class RobotContainer:

    ignoreSim = True

    def __init__(self) -> None:
        #Tunables

        #subsytems
        swerveModules = None
        gyro = None

        if not wpilib.RobotBase.isSimulation() or self.ignoreSim:
            swerveModules = [
                SwerveModule(0,1),
                SwerveModule(2,3),
                SwerveModule(4,5),
                SwerveModule(6,7),
            ]
            gyro = ...
        else:
            swerveModules = [
                #sim modules
            ]
            gyro = ...
        
        self.swerve = SwerveDrive( swerveModules, gyro )

        #commands

        #put everything on SmartDashboard