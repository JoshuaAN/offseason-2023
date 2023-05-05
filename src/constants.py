
import math

class ElectricalConstants:
    """
    A list of electrical constants for the robot
    """
    
    #  --------------- [fl, fr, rl, rr]
    kDriveMotorPorts = [28, 26, 10, 12]
    kSteerMotorPorts = [29, 27, 11, 13]
    kAbsEncoderPorts = [43, 33, 45, 31]

    kPigeonPort = 2

class DriveConstants:
    """
    A list of swerve drive constants
    """
    
    # swerve drive module indexes
    kFL = 0
    kFR = 1
    kRL = 2
    kRR = 3
    
class ModuleConstants:
    """
    A list of swerve module constants
    """
    
    kDriveP: float  = 0.025
    kDriveI: float = 0.0
    kDriveD: float = 0.0
    kDriveFF: float = 0.25

    kSteerP: float = 0.0075
    kSteerI: float = 0.0
    kSteerD: float = 0.0
    kSteerFF: float = 0.0

    # Unit: meters per second
    kMaxVelocity: float = 3.0

    # Unit: meters
    kWheelDiameter: float = 0.0762 # 3 in

    # Gear reduction (unitless) between the drive motor and the wheel
    kDriveGearRatio: float = 5.5

    # The drive encoder reports in RPM by default. Calculate the conversion factor
    # to make it report in meters per second.
    kDriveConversionFactor: float = (kWheelDiameter * math.pi) / kDriveGearRatio

    # Gear reduction (unitless) between the steering motor and the module azimuth
    # Stage 1 - REV Ultraplanetary nominal "4:1" stage, actual ratio 29:84
    # Stage 2 - REV Ultraplanetary nominal "3:1" stage, actual ratio 21:76
    # Stage 3 - 14:62
    kTurnPositionConversionFactor: float = 46.42

    # Unit: volts
    kNominalVoltage : int = 12

    # Unit: amps
    kDriveCurrentLimit: int = 60
    kSteerCurrentLimit: int = 25