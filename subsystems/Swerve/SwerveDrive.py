import wpilib
import wpimath
import wpimath.geometry

from commands2 import Subsystem
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, ChassisSpeeds
from phoenix5.sensors import WPI_Pigeon2

# from phoenix5.sensors import WPI_Pigeon2

from .SwerveModule import SwerveModule
from util import * #Tunables

class SwerveDrive(Subsystem):
    '''
    Constructs a Holonomic (Swerve) Drivetrain
    '''

    kMaxSpeed = 2.5 #meters per sec, arbitrary

    def __init__(self, modules:tuple[SwerveModule], gyro:WPI_Pigeon2) -> None:
        ##Subsystem Setup
        self.setName("SwerveDrive")

        ##Holonomic setup
        #SwerveModule offsets from center, in meters
        fl_offset = wpimath.geometry.Translation2d(0.2667,  0.2667)
        fr_offset = wpimath.geometry.Translation2d(0.2667, -0.2667)
        bl_offset = wpimath.geometry.Translation2d(-0.2667,  0.2667)
        br_offset = wpimath.geometry.Translation2d(-0.2667, -0.2667)

        #low-level components
        self.modules = modules
        self.gyro = gyro

        #converts between chassis velocity and Swerve Module States
        self.kinematics = SwerveDrive4Kinematics(
            fl_offset,
            fr_offset,
            bl_offset,
            br_offset
        )

        #tracks field position, args are mostly starting position
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            [
                module.getPosition() for module in self.modules
            ]
        )

        self.gyro.reset()

    def drive(self,
              xSpeed:float, ySpeed:float,
              rot:float,
              fieldRelative:bool,
              periodSeconds:float=...) -> None:
        """
        Parameters to construct desired ChassisSpeeds
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """
        #get module states through kinematics
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            # ChassisSpeeds.discretize(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, self.gyro.getRotation2d()) if fieldRelative
                else ChassisSpeeds(xSpeed, ySpeed, rot)#,

                # periodSeconds
            # )
        )

        #handle desired speeds > max speed
        SwerveDrive4Kinematics.desaturateWheelSpeeds( swerveModuleStates, self.kMaxSpeed )

        for i, module in enumerate(self.modules):
            module.setDesiredState(swerveModuleStates[i])
    
    def updateOdometry(self) -> None:
        """update field relative position of robot"""
        self.odometry.update(
            self.gyro.getRotation2d(),
            [
                module.getPosition() for module in self.modules
            ]
        )