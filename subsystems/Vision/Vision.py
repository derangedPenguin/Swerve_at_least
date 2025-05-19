from commands2 import Subsystem

import ntcore
import wpilib.cameraserver as camServer

class Vision(Subsystem):
    def __init__(self) -> None:
        super().__init__()