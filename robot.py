import wpilib

import commands2

from robotcontainer import RobotContainer

class MyRobot(wpilib.TimedRobot):

    def robotInit(self) -> None:
        self.m_robotcontainer = RobotContainer()

    def robotPeriodic(self) -> None:
        commands2.CommandScheduler.getInstance().run()
    