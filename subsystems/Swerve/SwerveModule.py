
from rev import CANSparkMax
from phoenix5.sensors import WPI_CANCoder

from wpimath.controller import PIDController, ProfiledPIDController, SimpleMotorFeedforwardMeters
from wpimath.trajectory import TrapezoidProfile
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d

from math import pi

from util import *

class SwerveModule:

    kMaxVelocity = pi
    kMaxAcceleration = pi * 2

    def __init__(self,
                 ssName:str,
                 drive_motor_port:int,
                 turn_motor_port:int,
                 abs_encoder_port:int, abs_encoder_offset:float) -> None:
        ##Tunables
        #PID vals
        self.drive_kP = 0.1# make these tunable
        self.drive_kD = 0.0

        self.turn_kP = 0.1
        self.turn_kD = 0.0

        self.turn_motor_inverted = NTTunableBoolean(f'/Swerve/modules/{ssName}/turnMotorInverted', False, lambda:(self.turn_motor.setInverted(self.turn_motor_inverted.get())))
        
        ##Physical Component Inits
        self.drive_motor = CANSparkMax( drive_motor_port, CANSparkMax.MotorType.kBrushless )
        self.turn_motor = CANSparkMax( turn_motor_port, CANSparkMax.MotorType.kBrushless )

        self.abs_turn_encoder = WPI_CANCoder(abs_encoder_port)
        self.abs_encoder_offset = abs_encoder_offset

        self.drive_motor_encoder = self.drive_motor.getEncoder()
        self.turn_motor_encoder = self.turn_motor.getEncoder()

        #Configs
        #change CAN timeout for cofigurating & run multiple times to ensure settings
        self.drive_motor.restoreFactoryDefaults()
        self.turn_motor.restoreFactoryDefaults()

        self.turn_motor.setInverted(self.turn_motor_inverted.get())
        #current limiting & Voltage Compensation

        #set motor encoder positions, measurment periods, & depths

        self.turn_motor.burnFlash()
        self.drive_motor.burnFlash()

        #drive/turn Position Queues?
        
        #PID Controllers
        self.drivePID = PIDController(self.drive_kP, 0.0, self.drive_kD)
        self.turnPID = ProfiledPIDController(
            self.turn_kP, 0.0, self.turn_kD,
            TrapezoidProfile.Constraints(
                self.kMaxVelocity,
                self.kMaxAcceleration
            )
        )
        self.turnPID.enableContinuousInput(-pi,pi)

        self.driveFF = SimpleMotorFeedforwardMeters(1, 3)#???
        self.turnFF = SimpleMotorFeedforwardMeters(1, 0.5)#???

        #set dist per pulse on encoder
        # self.drive_motor_encoder.
        # hunt in tyler code, need to update neo from revolutions

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.drive_motor_encoder.getVelocity(),
            Rotation2d(self.getAbsoluteEncoderPosition())
        )

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.drive_motor_encoder.getPosition(),
            Rotation2d(self.getAbsoluteEncoderPosition())
        )

    def getAbsoluteEncoderPosition(self) -> float:
        #currently uses getAbsolutePosition() - offset, could use voltage measurement instead???
        return self.abs_turn_encoder.getAbsolutePosition() - self.abs_encoder_offset

    def setDesiredState(self, desiredState:SwerveModuleState):
        encoderRotation = Rotation2d(self.getAbsoluteEncoderPosition())

        #create optimized desired state
        state = SwerveModuleState.optimize(desiredState, encoderRotation)

        #make it drive slow when it has to turn a lot
        state.speed *= (state.angle - encoderRotation).cos()

        driveOutput = self.drivePID.calculate(
            self.drive_motor_encoder.getVelocity(), state.speed
        )
        driveFeedForward = self.driveFF.calculate(state.speed)

        #YAY PID
        turnOutput = self.turnPID.calculate(
            self.getAbsoluteEncoderPosition(), state.angle.radians()
        )
        turnFeedForward = self.turnFF.calculate(self.turnPID.getSetpoint().velocity)

        self.drive_motor.setVoltage(driveOutput + driveFeedForward)
        self.turn_motor.setVoltage(turnOutput + turnFeedForward)

        