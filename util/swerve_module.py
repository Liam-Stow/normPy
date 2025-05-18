from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.hardware.cancoder import CANcoder
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.base_status_signal import BaseStatusSignal
import phoenix6.controls as controls
from wpimath import units
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d

class SwerveModule:
    def __init__(self, drive_id: int, steer_id: int, steer_encoder_id: int):
        # Make devices
        self.drive_motor = TalonFX(drive_id)
        self.steer_motor = TalonFX(steer_id)
        self.steer_encoder = CANcoder(steer_encoder_id)

        # Configure devices
        drive_config = TalonFXConfiguration()
        self.drive_motor.configurator.apply(drive_config)

        steer_config = TalonFXConfiguration()
        self.steer_motor.configurator.apply(steer_config)

        steer_encoder_config = CANcoderConfiguration()
        self.steer_encoder.configurator.apply(steer_encoder_config)

    def set_target_state(self, target_state: SwerveModuleState):
        current_angle = Rotation2d(self.get_angle_from_cancoder())

        target_state.optimize(current_angle)
        target_state.cosineScale(current_angle)

        self.set_target_rotation(target_state.angle)
        self.set_target_speed(target_state.speed)


    def sync_sensors(self):
        self.steer_motor.set_position(self.get_angle_from_cancoder())

    def send_to_dashboard(self):
        pass

    def set_target_rotation(self, angle: Rotation2d):
        rotations = angle.degrees() / 360.0
        self.set_target_angle(rotations)

    def set_target_angle(self, angle: units.turns):
        self.steer_motor.set_control(controls.PositionTorqueCurrentFOC(angle))

    def set_target_speed(self, speed: units.turns_per_second):
        self.drive_motor.set_control(controls.VelocityTorqueCurrentFOC(speed))

    def drive_straight_volts(self, volts: units.volts):
        self.drive_motor.set_control(controls.VoltageOut(volts))

    def stop(self):
        self.drive_motor.set_control(controls.VoltageOut(0))

    # def set_break_mode(self, break_mode: bool):
    #     pass

    # def get_state_from_internal_encoders(self) -> SwerveModuleState:
    #     pass
    
    # def get_angle_from_internal_encoder(self) -> units.turns:
    #     pass

    def get_angle_from_cancoder(self) -> units.turns:
        angle_signal = self.steer_encoder.get_absolute_position()
        velocity_signal = self.steer_encoder.get_velocity()
        compensated_angle = BaseStatusSignal.get_latency_compensated_value(angle_signal, velocity_signal)
        return compensated_angle

    # def get_wheel_velocity(self) -> units.turns_per_second:
    #     pass

    # def get_linear_velocity(self) -> units.meters_per_second:
    #     pass

    # def get_drive_voltage(self) -> units.volts:
    #     pass

    # def get_driven_rotations(self) -> units.turns:
    #     pass

    
