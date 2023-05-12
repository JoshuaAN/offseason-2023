import math

from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
from wpilib import DutyCycleEncoder, SmartDashboard
from commands2 import SubsystemBase

from rev import CANSparkMax, CANSparkMaxLowLevel, SparkMaxRelativeEncoder, SparkMaxPIDController

from constants import ModuleConstants

class SwerveModule(SubsystemBase):
    """
    Swerve module level code
    """
    module_id: int

    drive_motor: CANSparkMax
    steer_motor: CANSparkMax
    
    drive_encoder: SparkMaxRelativeEncoder
    steer_encoder: SparkMaxRelativeEncoder
    
    throughbore_encoder: DutyCycleEncoder
    
    absolute_offset: Rotation2d
    relative_offset: Rotation2d
    
    drive_controller: SparkMaxPIDController
    steer_controller: SparkMaxPIDController
    
    def __init__(self, 
                 module_id: int,
                 drive_motor_id: int, 
                 steer_motor_id: int,
                 steer_encoder_id: int,
                 absolute_offset: Rotation2d):
        self.module_id = module_id

        self.drive_motor = CANSparkMax(drive_motor_id, CANSparkMax.MotorType.kBrushless)
        self.steer_motor = CANSparkMax(steer_motor_id, CANSparkMax.MotorType.kBrushless)
        
        self.drive_motor.restoreFactoryDefaults()
        self.steer_motor.restoreFactoryDefaults()
        self.drive_motor.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self.steer_motor.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self.drive_motor.setInverted(False)
        self.steer_motor.setInverted(False)
        self.drive_motor.enableVoltageCompensation(ModuleConstants.kNominalVoltage)
        self.steer_motor.enableVoltageCompensation(ModuleConstants.kNominalVoltage)
        self.drive_motor.setSmartCurrentLimit(ModuleConstants.kDriveCurrentLimit)
        self.steer_motor.setSmartCurrentLimit(ModuleConstants.kSteerCurrentLimit)
        
        self.drive_encoder = self.drive_motor.getEncoder()
        self.steer_encoder = self.steer_motor.getEncoder()
        
        self.drive_encoder.setVelocityConversionFactor(ModuleConstants.kDriveConversionFactor / 60.0)
        self.drive_encoder.setPositionConversionFactor(ModuleConstants.kDriveConversionFactor)
        
        self.steer_encoder.setPositionConversionFactor(360.0 / ModuleConstants.kTurnPositionConversionFactor)

        self.drive_controller = self.drive_motor.getPIDController()
        self.steer_controller = self.steer_motor.getPIDController()

        self.drive_controller.setP(ModuleConstants.kDriveP)
        self.drive_controller.setI(ModuleConstants.kDriveI)
        self.drive_controller.setD(ModuleConstants.kDriveD)
        self.drive_controller.setFF(ModuleConstants.kDriveFF)

        self.steer_controller.setP(ModuleConstants.kSteerP)
        self.steer_controller.setI(ModuleConstants.kSteerI)
        self.steer_controller.setD(ModuleConstants.kSteerD)
        self.steer_controller.setFF(ModuleConstants.kSteerFF)

        self.drive_motor.burnFlash()
        self.steer_motor.burnFlash()

        self.throughbore_encoder = DutyCycleEncoder(steer_encoder_id)

        self.absolute_offset = absolute_offset
        self.relative_offset = Rotation2d()

        self.sync_encoders()
        
        self.steer_id = self.steer_motor.getDeviceId()
        abs_deg = self.get_absolute_position().getDegrees()
        rel_deg = self.get_relative_position().getDegrees()

        SmartDashboard.putNumber(f"Initial absolute position: {self.steer_id}", abs_deg)
        SmartDashboard.putNumber(f"Initial relative position: {self.steer_id}", rel_deg)

    def get_absolute_position(self) -> Rotation2d:
        abs_pos = Rotation2d.fromDegrees(-self.throughbore_encoder.getAbsolutePosition() * 360.0)
        return abs_pos - self.absolute_offset

    def get_relative_position(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.steer_encoder.getPosition()) - self.relative_offset
    
    def sync_encoders(self) -> None:
        deg = self.steer_encoder.getPosition() - self.get_absolute_position().degrees
        self.relative_offset = Rotation2d.fromDegrees(deg)
        
    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(speed = self.drive_encoder.getVelocity(), angle = self.get_relative_position())
    
    # Units: meters
    def get_drive_distance(self) -> float: 
        return self.drive_encoder.getPosition()

    def reset_distance(self) -> None:
        self.drive_encoder.setPosition(0.0)
        
    def set_desired_state(self, state: SwerveModuleState) -> None:
        currentAngle = self.get_relative_position().degrees
        delta = self.deltaAdjustedAngle(state.angle.degrees, currentAngle)
        driveOutput = state.speed

        if math.abs(delta) > 90:
            driveOutput *= -1
            delta -= math.copysign(180, delta)

        adjustedAngle = delta + currentAngle

        SmartDashboard.putNumber(f"Commanded Velocity: {self.steer_id}", driveOutput)
        SmartDashboard.putNumber(f"Commanded position: {self.steer_id}", adjustedAngle)

        self.steer_controller.setReference(adjustedAngle + self.relative_offset.degrees, CANSparkMaxLowLevel.ControlType.kPosition)
        self.drive_controller.setReference(driveOutput, CANSparkMaxLowLevel.ControlType.kVelocity)

    # Compute motor angular setpoint from desired and current angles.
    def delta_adjusted_angle(self, target_angle: float, current_angle: float) -> float:
        return ((target_angle - current_angle + 180) % 360 + 360) % 360 - 180
        
    def periodic(self) -> None:
        SmartDashboard.putNumber(f"Drive velocity: {self.steer_id}", self.getState().speed)
        SmartDashboard.putNumber(f"Relative position: {self.steer_id}", self.get_relative_position().degrees)
        SmartDashboard.putNumber(f"Absolute position: {self.steer_id}", self.get_absolute_position().degrees)
        SmartDashboard.putNumber(f"Drive position: {self.steer_id}", self.getDriveDistance())
        
        