from math import pi
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.hardware.cancoder import CANcoder
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.base_status_signal import BaseStatusSignal
import phoenix6.controls as controls
import phoenix6.signals as signals
import wpilib
import wpilib.simulation
from wpimath import units
from wpimath.kinematics import SwerveModuleState
from wpimath.kinematics import SwerveModulePosition
from wpimath.geometry import Rotation2d
from wpimath.system import plant

class SwerveModule:
    # Constants
    drive_gearing = 6.75
    steer_gearing = 150.0 / 7.0
    wheel_radius_m = 0.0481098886  # meters
    wheel_circumference_m = 2 * pi * wheel_radius_m

    # Simulation
    drive_moi = 0.01  # Moment of Inertia in kg*m^2
    steer_moi = 0.00001  # Moment of Inertia in kg*m^2
    drive_system = plant.LinearSystemId().DCMotorSystem(plant.DCMotor.krakenX60FOC(), drive_moi, drive_gearing)
    steer_system = plant.LinearSystemId().DCMotorSystem(plant.DCMotor.falcon500FOC(), steer_moi, steer_gearing)
    drive_physics_sim = wpilib.simulation.DCMotorSim(drive_system, plant.DCMotor.krakenX60FOC().withReduction(drive_gearing))
    steer_physics_sim = wpilib.simulation.DCMotorSim(steer_system, plant.DCMotor.falcon500FOC().withReduction(steer_gearing))

    def __init__(self, drive_id: int, steer_id: int, steer_encoder_id: int):
        # Make devices
        self.drive_motor = TalonFX(drive_id)
        self.steer_motor = TalonFX(steer_id)
        self.steer_encoder = CANcoder(steer_encoder_id)

        # Configure devices
        drive_config = TalonFXConfiguration()
        drive_config.feedback.feedback_sensor_source = signals.FeedbackSensorSourceValue.ROTOR_SENSOR
        drive_config.closed_loop_general.continuous_wrap = False
        drive_config.feedback.sensor_to_mechanism_ratio = self.drive_gearing
        drive_config.slot0.k_p = 0.01 
        drive_config.slot0.k_i = 0.0
        drive_config.slot0.k_d = 0.0
        drive_config.current_limits.supply_current_limit_enable = True
        drive_config.current_limits.stator_current_limit_enable = True
        drive_config.current_limits.supply_current_lower_limit = 40.0  # Amps
        drive_config.current_limits.supply_current_limit = 60.0  # Amps
        drive_config.current_limits.supply_current_lower_time = 0.1  # Seconds
        drive_config.current_limits.stator_current_limit = 80.0  # Amps
        drive_config.slot0.k_s = 0.0
        drive_config.slot0.k_v = 0.0 
        drive_config.slot0.k_a = 0.0
        drive_config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        self.drive_motor.configurator.apply(drive_config)

        steer_config = TalonFXConfiguration()
        steer_config.feedback.sensor_to_mechanism_ratio = self.steer_gearing
        steer_config.closed_loop_general.continuous_wrap = True
        steer_config.slot0.k_p = 60.0
        steer_config.slot0.k_i = 0.0
        steer_config.slot0.k_d = 0.0 
        steer_config.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        steer_config.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        steer_config.current_limits.supply_current_limit_enable = True
        steer_config.current_limits.stator_current_limit_enable = True
        steer_config.current_limits.supply_current_lower_limit = 30.0  # Amps
        steer_config.current_limits.supply_current_limit = 40.0  # Amps
        steer_config.current_limits.supply_current_lower_time = 0.1  # Seconds
        steer_config.current_limits.stator_current_limit = 120.0  # Amps
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

    def get_state_from_internal_encoders(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.get_linear_velocity(), self.get_rotation_from_internal_encoder()
        )

    def get_state_from_cancoder(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.get_linear_velocity(), self.get_rotation_from_cancoder()
        )
    
    def get_position_from_cancoder(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.get_driven_distance(), self.get_rotation_from_cancoder())
    
    def get_angle_from_internal_encoder(self) -> units.turns:
        return self.steer_motor.get_position().value
    
    def get_rotation_from_internal_encoder(self) -> Rotation2d:
        return Rotation2d(self.get_angle_from_internal_encoder())

    def get_angle_from_cancoder(self) -> units.turns:
        angle_signal = self.steer_encoder.get_absolute_position()
        velocity_signal = self.steer_encoder.get_velocity()
        compensated_angle = BaseStatusSignal.get_latency_compensated_value(angle_signal, velocity_signal)
        return compensated_angle
    
    def get_rotation_from_cancoder(self) -> Rotation2d:
        return Rotation2d(self.get_angle_from_cancoder())

    # def get_wheel_velocity(self) -> units.turns_per_second:
    #     pass

    def get_linear_velocity(self) -> units.meters_per_second:
        return self.drive_motor.get_velocity().value * self.wheel_circumference_m

    # def get_drive_voltage(self) -> units.volts:
    #     pass

    def get_driven_rotations(self) -> units.turns:
        return self.drive_motor.get_position().value

    def get_driven_distance(self) -> units.meters:
        return self.get_driven_rotations() * self.wheel_circumference_m

    def update_sim(self):
        drive_sim_state = self.drive_motor.sim_state
        steer_sim_state = self.steer_motor.sim_state
        steer_encoder_sim_state = self.steer_encoder.sim_state

        # Sim the drive motor
        self.drive_physics_sim.setInputVoltage(drive_sim_state.motor_voltage * 12.0)
        self.drive_physics_sim.update(0.02)
        drive_sim_state.set_raw_rotor_position(self.drive_physics_sim.getAngularPositionRotations() * self.drive_gearing)
        rpm = self.drive_physics_sim.getAngularVelocityRPM()
        rps = rpm / 60
        drive_sim_state.set_rotor_velocity(rps * self.drive_gearing)

        # Sim the steer motor
        self.steer_physics_sim.setInputVoltage(steer_sim_state.motor_voltage * 12.0)
        self.steer_physics_sim.update(0.02)
        steer_sim_state.set_raw_rotor_position(self.steer_physics_sim.getAngularPositionRotations() * self.steer_gearing)
        rpm = self.steer_physics_sim.getAngularVelocityRPM()
        rps = rpm / 60
        steer_sim_state.set_rotor_velocity(rps * self.steer_gearing)

        # Sim the steer encoder
        steer_encoder_sim_state.set_velocity(rps)
        steer_encoder_sim_state.set_raw_position(self.steer_physics_sim.getAngularPositionRotations())
