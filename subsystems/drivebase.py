from typing import Callable

from commands2 import Command, Subsystem
from commands2.button import CommandXboxController
from phoenix6.hardware.pigeon2 import Pigeon2
from wpilib import DriverStation, Field2d, SmartDashboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics

from constants import can
from util.swerve_module import SwerveModule


class Drivebase(Subsystem):
    gyro = Pigeon2(can.drivebaseGyro)

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

    def update_odometry(self) -> None:
        module_positions = (
            self.front_left_module.get_position_from_cancoder(),
            self.front_right_module.get_position_from_cancoder(),
            self.back_left_module.get_position_from_cancoder(),
            self.back_right_module.get_position_from_cancoder(),
        )

        alliance = DriverStation.getAlliance()
        if alliance == DriverStation.Alliance.kBlue:
            self.pose_estimator.update(self.get_gyro_rotation(), module_positions)
        else:
            self.pose_estimator.update(
                self.get_gyro_rotation() - Rotation2d.fromDegrees(180), module_positions
            )

        self.field_display.setRobotPose(self.pose_estimator.getEstimatedPosition())

    def calc_joystick_speeds(self, controller: CommandXboxController):
        pass

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
            speeds = ChassisSpeeds.discretize(speeds, 0.02)

            module_states = self.kinematics.toSwerveModuleStates(speeds)
            self.kinematics.desaturateWheelSpeeds(module_states, self.max_speed_mps)

            [fl, fr, bl, br] = module_states
            self.front_left_module.set_target_state(fl)
            self.front_right_module.set_target_state(fr)
            self.back_left_module.set_target_state(bl)
            self.back_right_module.set_target_state(br)

        return super().run(lambda: drive_loop(speeds()))

    def get_gyro_rotation(self) -> Rotation2d:
        return self.gyro.getRotation2d()
