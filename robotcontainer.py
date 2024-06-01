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
                #real modules
            ]
            gyro = ...
        else:
            swerveModules = [
                #sim modules
            ]
            gyro = ...
        
        self.swerve = Drivetrain( swerveModules, gyro )

        #commands

        #put everything on SmartDashboard