import wpilib
import wpimath
import wpimath.geometry

# from phoenix5.sensors import WPI_Pigeon2

# from subsystems import SwerveModule

class SwerveDrive:
    '''
    Constructs a Swerve (Holonomic) Drivetrain
    '''
    def __init__(self, module_port_range:tuple[int], gyro_port:int=0) -> None:
        ##Holonomic setup
        #SwerveModule offsets from center, in meters
        fl_offset = wpimath.geometry.Translation2d(-0.25,0.25)
        fr_offset = wpimath.geometry.Translation2d(0.25,0.25)
        bl_offset = wpimath.geometry.Translation2d(-0.25,-0.25)
        br_offset = wpimath.geometry.Translation2d(0.25,-0.25)

        self.modules = [
            SwerveModule
        ]

