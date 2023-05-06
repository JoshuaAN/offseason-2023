from wpimath.kinematics import SwerveDrive2Odometry
from swervemodule import SwerveModule
from constants import DriveConstants, ElectricalConstants
from typing import List 

class SwerveDrive:
    """
    Swerve drive code
    """

    front_left: SwerveModule
    front_right: SwerveModule
    rear_left: SwerveModule
    rear_right: SwerveModule

    modules: List[SwerveModule]

    odometry: SwerveDrive2Odometry

    def __init__(self):
        self.front_left = SwerveModule(DriveConstants.kFL,
                                       ElectricalConstants.kDriveMotorPorts[DriveConstants.kFL],
                                       ElectricalConstants.kSteerMotorPorts[DriveConstants.kFL],
                                       ElectricalConstants.kAbsEncoderPorts[DriveConstants.kFL])
        self.front_right = SwerveModule(DriveConstants.kFR,
                                       ElectricalConstants.kDriveMotorPorts[DriveConstants.kFR],
                                       ElectricalConstants.kSteerMotorPorts[DriveConstants.kFR],
                                       ElectricalConstants.kAbsEncoderPorts[DriveConstants.kFR])
        self.rear_left = SwerveModule(DriveConstants.kRL,
                                       ElectricalConstants.kDriveMotorPorts[DriveConstants.kRL],
                                       ElectricalConstants.kSteerMotorPorts[DriveConstants.kRL],
                                       ElectricalConstants.kAbsEncoderPorts[DriveConstants.kRL])
        self.rear_right = SwerveModule(DriveConstants.kRR,
                                       ElectricalConstants.kDriveMotorPorts[DriveConstants.kRR],
                                       ElectricalConstants.kSteerMotorPorts[DriveConstants.kRR],
                                       ElectricalConstants.kAbsEncoderPorts[DriveConstants.kRR])

        self.modules = [self.front_left, self.front_right, self.rear_left, self.rear_right]