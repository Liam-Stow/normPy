from typing import Callable

from commands2 import Command, Subsystem
from commands2.button import CommandXboxController
from phoenix6.hardware.pigeon2 import Pigeon2
from wpilib import DriverStation, Field2d, SmartDashboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics
from wpimath.filter import SlewRateLimiter
from wpimath import applyDeadband
from constants import can
from util.swerve_module import SwerveModule
from wpimath.units import radiansToDegrees
import math

class Drivebase(Subsystem):
    gyro = Pigeon2(can.drivebaseGyro)

    JOYSTICK_DEADBAND = 0.05
    MAX_VELOCITY_MPS = 5.0
    MAX_ANGULAR_VELOCITY_TPS = 0.8
    MAX_JOYSTICK_ACCEL = 5.0
    MAX_ANGULAR_JOYSTICK_ACCEL = 3.0
    TRANSLATION_SCALING = 2.0
    ROTATION_SCALING = 1.0

    front_left_module = SwerveModule(
        can.drivebaseFrontLeftDrive,
        can.drivebaseFrontLeftSteer,
        can.drivebaseFrontLeftEncoder,
        "FL",
    )
    front_right_module = SwerveModule(
        can.drivebaseFrontRightDrive,
        can.drivebaseFrontRightSteer,
        can.drivebaseFrontRightEncoder,
        "FR",
    )
    back_left_module = SwerveModule(
        can.drivebaseBackLeftDrive,
        can.drivebaseBackLeftSteer,
        can.drivebaseBackLeftEncoder,
        "BL",
    )
    back_right_module = SwerveModule(
        can.drivebaseBackRightDrive,
        can.drivebaseBackRightSteer,
        can.drivebaseBackRightEncoder,
        "BR",
    )

    front_left_location = Translation2d(0.381, 0.381)
    front_right_location = Translation2d(0.381, -0.381)
    back_left_location = Translation2d(-0.381, 0.381)
    back_right_location = Translation2d(-0.381, -0.381)

    kinematics = SwerveDrive4Kinematics(
        front_left_location,
        front_right_location,
        back_left_location,
        back_right_location,
    )

    max_speed_mps = 5.0  # m/s
    field_display = Field2d()

    def __init__(self):
        module_positions = (
            self.front_left_module.get_position_from_cancoder(),
            self.front_right_module.get_position_from_cancoder(),
            self.back_left_module.get_position_from_cancoder(),
            self.back_right_module.get_position_from_cancoder(),
        )
        self.pose_estimator = SwerveDrive4PoseEstimator(
            self.kinematics, Rotation2d(0), module_positions, Pose2d()
        )
        self.tuned_max_joystick_accel = self.MAX_JOYSTICK_ACCEL
        self.tuned_max_angular_joystick_accel = self.MAX_ANGULAR_JOYSTICK_ACCEL
        self.x_stick_limiter = SlewRateLimiter(self.tuned_max_joystick_accel)
        self.y_stick_limiter = SlewRateLimiter(self.tuned_max_joystick_accel)
        self.rot_stick_limiter = SlewRateLimiter(self.tuned_max_angular_joystick_accel)

        SmartDashboard.putData("drivebase/Field", self.field_display)

    def periodic(self) -> None:
        self.front_left_module.send_to_dashboard()
        self.front_right_module.send_to_dashboard()
        self.back_left_module.send_to_dashboard()
        self.back_right_module.send_to_dashboard()
        self.update_odometry()

    def simulationPeriodic(self) -> None:
        self.front_left_module.update_sim()
        self.front_right_module.update_sim()
        self.back_left_module.update_sim()
        self.back_right_module.update_sim()

        # Adjust gyro angle
        module_states = (
             self.front_left_module.get_state_from_internal_encoders(),
             self.front_right_module.get_state_from_internal_encoders(),
             self.back_left_module.get_state_from_internal_encoders(),
             self.back_right_module.get_state_from_internal_encoders(),
        )
        rot_speed_rad_per_sec = self.kinematics.toChassisSpeeds(module_states).omega
        change_in_rot_deg = radiansToDegrees(rot_speed_rad_per_sec) * 0.02 # 0.02 seconds per loop
        self.gyro.sim_state.add_yaw(change_in_rot_deg)

    def update_odometry(self) -> None:
        module_positions = (
            self.front_left_module.get_position_from_cancoder(),
            self.front_right_module.get_position_from_cancoder(),
            self.back_left_module.get_position_from_cancoder(),
            self.back_right_module.get_position_from_cancoder(),
        )

        rotation = self.get_gyro_rotation()
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            rotation -= Rotation2d.fromRotations(0.5)
        self.pose_estimator.update(rotation, module_positions)

        self.field_display.setRobotPose(self.pose_estimator.getEstimatedPosition())

    def calc_joystick_speeds(self, controller: CommandXboxController):

        configPath = "Drivebase/Config/"
        deadband = SmartDashboard.getNumber(
            configPath + "Joystick Deadband", self.JOYSTICK_DEADBAND
        )
        max_vel_mps = SmartDashboard.getNumber(
            configPath + "Max Velocity", self.MAX_VELOCITY_MPS
        )
        max_ang_vel_tps = SmartDashboard.getNumber(
            configPath + "Max Angular Velocity", self.MAX_ANGULAR_VELOCITY_TPS
        )
        max_joystick_accel = SmartDashboard.getNumber(
            configPath + "Max Joystick Accel", self.MAX_JOYSTICK_ACCEL
        )
        max_ang_joystick_accel = SmartDashboard.getNumber(
            configPath + "Max Joystick Angular Accel", self.MAX_ANGULAR_JOYSTICK_ACCEL
        )
        translation_scaling = SmartDashboard.getNumber(
            configPath + "Translation Scaling", self.TRANSLATION_SCALING
        )
        rotaiton_scaling = SmartDashboard.getNumber(
            configPath + "Rotation Scaling", self.ROTATION_SCALING
        )

        # # Recreate slew rate limiters if limits have changed
        # if max_joystick_accel != self.tuned_max_joystick_accel:
        #     self.x_stick_limiter = SlewRateLimiter(max_joystick_accel)
        #     self.y_stick_limiter = SlewRateLimiter(max_joystick_accel)
        #     self.tuned_max_joystick_accel = max_joystick_accel
        # if max_ang_joystick_accel != self.tuned_max_angular_joystick_accel:
        #     self.rot_stick_limiter = SlewRateLimiter(max_ang_joystick_accel)
        #     self.tuned_max_angular_joystick_accel = max_ang_joystick_accel

        # # Apply deadbands
        # raw_translation_y = applyDeadband(-controller.getLeftY(), deadband)
        # raw_translation_x = applyDeadband(-controller.getLeftX(), deadband)
        # raw_rotation = applyDeadband(-controller.getRightX(), deadband)

        # # Convert cartesian (x, y) translation stick coordinates to polar (R, theta) and scale R-value
        # rawTranslationR = min(1.0, math.sqrt(pow(raw_translation_x, 2) + pow(raw_translation_y, 2)))
        # translationTheta = math.atan2(raw_translation_y, raw_translation_x)
        # scaledTranslationR = pow(rawTranslationR, translation_scaling)

        # # Convert polar coordinates (with scaled R-value) back to cartesian; scale rotation as well
        # scaledTranslationY = scaledTranslationR * math.sin(translationTheta)
        # scaledTranslationX = scaledTranslationR * math.cos(translationTheta)

        # scaledRotation = pow(raw_rotation, rotaiton_scaling)

        # Apply joystick rate limits and calculate speed
        # forwardSpeed = self.y_stick_limiter.calculate(scaledTranslationY) * max_vel_mps
        # sidewaysSpeed = self.x_stick_limiter.calculate(scaledTranslationX) * max_vel_mps
        # rotationSpeed = self.rot_stick_limiter.calculate(scaledRotation) * max_ang_vel_tps

        # Dashboard things
        # SmartDashboard.putNumber(
        #     "Drivebase/Joystick Scaling/rawTranslationY", raw_translation_y
        # )
        # SmartDashboard.putNumber(
        #     "Drivebase/Joystick Scaling/rawTranslationX", raw_translation_x
        # )
        # SmartDashboard.putNumber(
        #     "Drivebase/Joystick Scaling/rawTranslationR", rawTranslationR
        # )
        # SmartDashboard.putNumber(
        #     "Drivebase/Joystick Scaling/translationTheta (rad)", translationTheta
        # )
        # SmartDashboard.putNumber(
        #     "Drivebase/Joystick Scaling/scaledTranslationR", scaledTranslationR
        # )
        # SmartDashboard.putNumber(
        #     "Drivebase/Joystick Scaling/scaledTranslationY", scaledTranslationY
        # )
        # SmartDashboard.putNumber(
        #     "Drivebase/Joystick Scaling/scaledTranslationX", scaledTranslationX
        # )
        # SmartDashboard.putNumber("Drivebase/Joystick Scaling/rawRotation", raw_rotation)
        # SmartDashboard.putNumber(
        #     "Drivebase/Joystick Scaling/scaledRotation", scaledRotation
        # )

        # return ChassisSpeeds(forwardSpeed, sidewaysSpeed, rotationSpeed)
        return ChassisSpeeds(-controller.getLeftY() * max_vel_mps, controller.getLeftX() * max_vel_mps, controller.getRightX() * max_ang_vel_tps)

    def joystick_drive(
        self, controller: CommandXboxController, field_oriented: bool = True
    ) -> Command:
        return self.drive(lambda: self.calc_joystick_speeds(controller), field_oriented)

    def drive(
        self, speeds: Callable[[], ChassisSpeeds], field_oriented: bool
    ) -> Command:
        def drive_loop(speeds: ChassisSpeeds):
            speeds = (
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds.vx, speeds.vy, speeds.omega, self.get_gyro_rotation()
                )
                if field_oriented
                else speeds
            )
            # speeds = ChassisSpeeds.discretize(speeds, 0.02)

            module_states = self.kinematics.toSwerveModuleStates(speeds)
            self.kinematics.desaturateWheelSpeeds(module_states, self.max_speed_mps)

            [fl, fr, bl, br] = module_states
            self.front_left_module.set_target_state(fl)
            self.front_right_module.set_target_state(fr)
            self.back_left_module.set_target_state(bl)
            self.back_right_module.set_target_state(br)

        return super().run(lambda: drive_loop(speeds()))

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

    def get_gyro_rotation(self) -> Rotation2d:
        return self.gyro.getRotation2d()
