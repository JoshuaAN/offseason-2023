from typing import List 

from wpimath.kinematics import SwerveDrive2Odometry

from wpimath.geometry import Rotation2d

import ctre

from subsystems.swervemodule import SwerveModule
from constants import DriveConstants, ElectricalConstants

class SwerveDrive:
    """
    Swerve drive code
    """

    gyro: ctre.PigeonIMU

    front_left: SwerveModule
    front_right: SwerveModule
    rear_left: SwerveModule
    rear_right: SwerveModule

    modules: List[SwerveModule]

    odometry: SwerveDrive2Odometry

    def __init__(self):
        self.gyro = ctre.PigeonIMU(ElectricalConstants.kPigeonPort)

        self.front_left = SwerveModule(DriveConstants.kFL,
                                       ElectricalConstants.kDriveMotorPorts[DriveConstants.kFL],
                                       ElectricalConstants.kSteerMotorPorts[DriveConstants.kFL],
                                       ElectricalConstants.kAbsEncoderPorts[DriveConstants.kFL],
                                       Rotation2d())
        self.front_right = SwerveModule(DriveConstants.kFR,
                                       ElectricalConstants.kDriveMotorPorts[DriveConstants.kFR],
                                       ElectricalConstants.kSteerMotorPorts[DriveConstants.kFR],
                                       ElectricalConstants.kAbsEncoderPorts[DriveConstants.kFR],
                                       Rotation2d())
        self.rear_left = SwerveModule(DriveConstants.kRL,
                                       ElectricalConstants.kDriveMotorPorts[DriveConstants.kRL],
                                       ElectricalConstants.kSteerMotorPorts[DriveConstants.kRL],
                                       ElectricalConstants.kAbsEncoderPorts[DriveConstants.kRL],
                                       Rotation2d())
        self.rear_right = SwerveModule(DriveConstants.kRR,
                                       ElectricalConstants.kDriveMotorPorts[DriveConstants.kRR],
                                       ElectricalConstants.kSteerMotorPorts[DriveConstants.kRR],
                                       ElectricalConstants.kAbsEncoderPorts[DriveConstants.kRR],
                                       Rotation2d())

        self.modules = [self.front_left, self.front_right, self.rear_left, self.rear_right]

    def drive():
        print("NOT DONE (UNFINISHED)")