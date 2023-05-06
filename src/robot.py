import commands2
from subsystems.swervedrive import SwerveDrive

class Robot(commands2.TimedCommandRobot):
    """
    Hold the robot periodic code
    """ 

    swerve_drive = SwerveDrive()