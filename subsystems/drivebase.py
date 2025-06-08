import math
from typing import Callable

from commands2 import Command, Subsystem
from commands2.button import CommandXboxController
from phoenix6.hardware.pigeon2 import Pigeon2
from wpilib import DriverStation, SmartDashboard
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics
from wpimath.units import radiansToDegrees, rotationsToRadians
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.controller import PIDController
from pathplannerlib.util import DriveFeedforwards # type: ignore

from constants import can
from util.swerve_module import SwerveModule


class Drivebase(Subsystem):
    JOYSTICK_DEADBAND = 0.05
    MAX_VELOCITY_MPS = 5.0
    MAX_ANGULAR_VELOCITY_TPS = 0.8
    MAX_JOYSTICK_ACCEL = 5.0
    MAX_ANGULAR_JOYSTICK_ACCEL = 3.0
    TRANSLATION_SCALING = 2.0
    ROTATION_SCALING = 1.0
    CONFIG_PATH = "drivebase/Config/"

    def __init__(self):
        # Coponents: gyro and modules
        self.gyro = Pigeon2(can.drivebaseGyro)
        self.front_left_module = SwerveModule(
            can.drivebaseFrontLeftDrive,
            can.drivebaseFrontLeftSteer,
            can.drivebaseFrontLeftEncoder,
            "FL",
        )
        self.front_right_module = SwerveModule(
            can.drivebaseFrontRightDrive,
            can.drivebaseFrontRightSteer,
            can.drivebaseFrontRightEncoder,
            "FR",
        )
        self.back_left_module = SwerveModule(
            can.drivebaseBackLeftDrive,
            can.drivebaseBackLeftSteer,
            can.drivebaseBackLeftEncoder,
            "BL",
        )
        self.back_right_module = SwerveModule(
            can.drivebaseBackRightDrive,
            can.drivebaseBackRightSteer,
            can.drivebaseBackRightEncoder,
            "BR",
        )

        # Kinematics
        self.front_left_location = Translation2d(0.381, 0.381)
        self.front_right_location = Translation2d(0.381, -0.381)
        self.back_left_location = Translation2d(-0.381, 0.381)
        self.back_right_location = Translation2d(-0.381, -0.381)
        self.kinematics = SwerveDrive4Kinematics(
            self.front_left_location,
            self.front_right_location,
            self.back_left_location,
            self.back_right_location,
        )

        # PID for teleop drive (not pathplanner)
        self.translation_pid = PIDController(4, 0, 0.15)
        self.rotation_pid = PIDController(4, 0, 0.15)

        # Driving filters config
        self.tuned_max_joystick_accel = self.MAX_JOYSTICK_ACCEL
        self.tuned_max_angular_joystick_accel = self.MAX_ANGULAR_JOYSTICK_ACCEL
        self.x_stick_limiter = SlewRateLimiter(self.tuned_max_joystick_accel)
        self.y_stick_limiter = SlewRateLimiter(self.tuned_max_joystick_accel)
        self.rot_stick_limiter = SlewRateLimiter(self.tuned_max_angular_joystick_accel)

        # Dashboard
        SmartDashboard.putNumber(self.CONFIG_PATH + "Joystick Deadband", self.JOYSTICK_DEADBAND)
        SmartDashboard.putNumber(self.CONFIG_PATH + "Max Velocity", self.MAX_VELOCITY_MPS)
        SmartDashboard.putNumber(self.CONFIG_PATH + "Max Angular Velocity", self.MAX_ANGULAR_VELOCITY_TPS)
        SmartDashboard.putNumber(self.CONFIG_PATH + "Max Joystick Accel", self.MAX_JOYSTICK_ACCEL)
        SmartDashboard.putNumber(self.CONFIG_PATH + "Max Joystick Angular Accel", self.MAX_ANGULAR_JOYSTICK_ACCEL)
        SmartDashboard.putNumber(self.CONFIG_PATH + "Translation Scaling", self.TRANSLATION_SCALING)
        SmartDashboard.putNumber(self.CONFIG_PATH + "Rotation Scaling", self.ROTATION_SCALING)

    def periodic(self) -> None:
        self.front_left_module.send_to_dashboard()
        self.front_right_module.send_to_dashboard()
        self.back_left_module.send_to_dashboard()
        self.back_right_module.send_to_dashboard()

    def simulationPeriodic(self) -> None:
        self.front_left_module.update_sim()
        self.front_right_module.update_sim()
        self.back_left_module.update_sim()
        self.back_right_module.update_sim()

        # Adjust gyro angle
        rot_speed_rad_per_sec = self.kinematics.toChassisSpeeds(
            self.get_module_states()
        ).omega
        change_in_rot_deg = (
            radiansToDegrees(rot_speed_rad_per_sec) * 0.02
        )  # 0.02 seconds per loop
        self.gyro.sim_state.add_yaw(change_in_rot_deg)

    def set_gyro_rotation(
        self, rotation: Rotation2d, flip_on_red_alliance: bool
    ) -> None:
        if (
            flip_on_red_alliance
            and DriverStation.getAlliance() == DriverStation.Alliance.kRed
        ):
            rotation = rotation.rotateBy(Rotation2d.fromRotations(0.5))
        self.gyro.set_yaw(rotation.degrees())

    def calc_drive_to_pose_speeds(self, current_pose: Pose2d, target_pose: Pose2d) -> ChassisSpeeds:
        # Calculate the desired speeds from the difference
        delta = current_pose - target_pose
        forward_speed_mps = self.translation_pid.calculate(delta.x)
        sideways_speed_mps = self.translation_pid.calculate(delta.y)
        rotation_speed_rps = self.rotation_pid.calculate(delta.rotation().radians())

        # Create and return the ChassisSpeeds object
        return ChassisSpeeds(
            forward_speed_mps,
            sideways_speed_mps,
            rotation_speed_rps,
        )


    def calc_joystick_speeds(self, controller: CommandXboxController):
        deadband = SmartDashboard.getNumber(self.CONFIG_PATH + "Joystick Deadband", self.JOYSTICK_DEADBAND)
        max_vel_mps = SmartDashboard.getNumber(self.CONFIG_PATH + "Max Velocity", self.MAX_VELOCITY_MPS)
        max_ang_vel_tps = SmartDashboard.getNumber(self.CONFIG_PATH + "Max Angular Velocity", self.MAX_ANGULAR_VELOCITY_TPS)
        max_joystick_accel = SmartDashboard.getNumber(self.CONFIG_PATH + "Max Joystick Accel", self.MAX_JOYSTICK_ACCEL)
        max_ang_joystick_accel = SmartDashboard.getNumber(self.CONFIG_PATH + "Max Joystick Angular Accel", self.MAX_ANGULAR_JOYSTICK_ACCEL)
        translation_scaling: float = SmartDashboard.getNumber(self.CONFIG_PATH + "Translation Scaling", self.TRANSLATION_SCALING)
        rotaiton_scaling: float = SmartDashboard.getNumber(self.CONFIG_PATH + "Rotation Scaling", self.ROTATION_SCALING)

        # Recreate slew rate limiters if limits have changed
        if max_joystick_accel != self.tuned_max_joystick_accel:
            self.x_stick_limiter = SlewRateLimiter(max_joystick_accel)
            self.y_stick_limiter = SlewRateLimiter(max_joystick_accel)
            self.tuned_max_joystick_accel = max_joystick_accel
        if max_ang_joystick_accel != self.tuned_max_angular_joystick_accel:
            self.rot_stick_limiter = SlewRateLimiter(max_ang_joystick_accel)
            self.tuned_max_angular_joystick_accel = max_ang_joystick_accel

        # Apply deadbands
        raw_translation_y = applyDeadband(-controller.getLeftY(), deadband)
        raw_translation_x = applyDeadband(-controller.getLeftX(), deadband)
        raw_rotation = applyDeadband(-controller.getRightX(), deadband)

        # Convert cartesian (x, y) translation stick coordinates to polar (R, theta) and scale R-value
        raw_translation_R = min(1.0, math.sqrt(pow(raw_translation_x, 2) + pow(raw_translation_y, 2)))
        translation_theta = math.atan2(raw_translation_y, raw_translation_x)
        scaled_translation_R = pow(raw_translation_R, translation_scaling)

        # Convert polar coordinates (with scaled R-value) back to cartesian; scale rotation as well
        scaled_translation_Y = scaled_translation_R * math.sin(translation_theta)
        scaled_translation_X = scaled_translation_R * math.cos(translation_theta)
        scaled_rotation = pow(raw_rotation, rotaiton_scaling)

        # Apply joystick rate limits and calculate speed
        forward_speed_mps = self.y_stick_limiter.calculate(scaled_translation_Y) * max_vel_mps
        sideways_speed_mps = self.x_stick_limiter.calculate(scaled_translation_X) * max_vel_mps
        rotation_speed_tps = self.rot_stick_limiter.calculate(scaled_rotation) * max_ang_vel_tps

        # Dashboard things
        path = "drivebase/Joystick Scaling/"
        SmartDashboard.putNumber(f"{path}rawTranslationY", raw_translation_y)
        SmartDashboard.putNumber(f"{path}rawTranslationX", raw_translation_x)
        SmartDashboard.putNumber(f"{path}rawTranslationR", raw_translation_R)
        SmartDashboard.putNumber(f"{path}translationTheta (rad)", translation_theta)
        SmartDashboard.putNumber(f"{path}scaledTranslationR", scaled_translation_R)
        SmartDashboard.putNumber(f"{path}scaledTranslationY", scaled_translation_Y)
        SmartDashboard.putNumber(f"{path}scaledTranslationX", scaled_translation_X)
        SmartDashboard.putNumber(f"{path}rawRotation", raw_rotation)
        SmartDashboard.putNumber(f"{path}scaledRotation", scaled_rotation)

        return ChassisSpeeds(
            forward_speed_mps,
            sideways_speed_mps,
            rotationsToRadians(rotation_speed_tps),
        )

    def joystick_drive(
        self, controller: CommandXboxController, field_oriented: bool = True
    ) -> Command:
        return self.drive(lambda: self.calc_joystick_speeds(controller), field_oriented)

    def drive(
        self, speeds: Callable[[], ChassisSpeeds], field_oriented: bool
    ) -> Command:
        return super().run(
            lambda: self.drive_implementation(speeds(), None, field_oriented)
        )

    def drive_implementation(
        self,
        speeds: ChassisSpeeds,
        feedforwards: DriveFeedforwards | None,
        field_oriented: bool,
    ):
        if field_oriented:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vx,
                speeds.vy,
                speeds.omega,
                self.get_gyro_rotation(flip_on_red_alliance=False),
            )

        speeds = ChassisSpeeds.discretize(speeds, 0.02)

        module_states = self.kinematics.toSwerveModuleStates(speeds)
        max_vel_mps = SmartDashboard.getNumber(
            self.CONFIG_PATH + "Max Velocity", self.MAX_VELOCITY_MPS
        )
        self.kinematics.desaturateWheelSpeeds(module_states, max_vel_mps)

        [fl, fr, bl, br] = module_states
        self.front_left_module.set_target_state(fl)
        self.front_right_module.set_target_state(fr)
        self.back_left_module.set_target_state(bl)
        self.back_right_module.set_target_state(br)

    def point_wheels_left(self) -> Command:
        def loop():
            self.front_left_module.set_target_angle(0.25)
            self.front_right_module.set_target_angle(0.25)
            self.back_left_module.set_target_angle(0.25)
            self.back_right_module.set_target_angle(0.25)

        return super().run(loop)

    def point_wheels_straight(self) -> Command:
        def loop():
            self.front_left_module.set_target_angle(0)
            self.front_right_module.set_target_angle(0)
            self.back_left_module.set_target_angle(0)
            self.back_right_module.set_target_angle(0)

        return super().run(loop)

    def get_gyro_rotation(self, flip_on_red_alliance: bool) -> Rotation2d:
        if (
            flip_on_red_alliance
            and DriverStation.getAlliance() == DriverStation.Alliance.kRed
        ):
            return self.gyro.getRotation2d().rotateBy(Rotation2d.fromRotations(0.5))
        else:
            return self.gyro.getRotation2d()

    def get_module_positions(
        self,
    ) -> tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ]:
        return (
            self.front_left_module.get_position_from_cancoder(),
            self.front_right_module.get_position_from_cancoder(),
            self.back_left_module.get_position_from_cancoder(),
            self.back_right_module.get_position_from_cancoder(),
        )

    def get_module_states(
        self,
    ) -> tuple[
        SwerveModuleState,
        SwerveModuleState,
        SwerveModuleState,
        SwerveModuleState,
    ]:
        return (
            self.front_left_module.get_state_from_cancoder(),
            self.front_right_module.get_state_from_cancoder(),
            self.back_left_module.get_state_from_cancoder(),
            self.back_right_module.get_state_from_cancoder(),
        )

    def get_robot_relative_speeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.get_module_states())
