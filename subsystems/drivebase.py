from util.swerve_module import SwerveModule
from constants import can
from commands2.button import CommandXboxController
from wpimath.kinematics import ChassisSpeeds
from typing import Callable
from commands2 import Command, Subsystem
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d
from phoenix6.hardware.pigeon2 import Pigeon2
from wpilib import SmartDashboard


class Drivebase(Subsystem):
    gyro = Pigeon2(can.drivebaseGyro)

    front_left_module = SwerveModule(can.drivebaseFrontLeftDrive, can.drivebaseFrontLeftSteer, can.drivebaseFrontLeftEncoder)
    front_right_module = SwerveModule(can.drivebaseFrontRightDrive, can.drivebaseFrontRightSteer, can.drivebaseFrontRightEncoder)
    back_left_module = SwerveModule(can.drivebaseBackLeftDrive, can.drivebaseBackLeftSteer, can.drivebaseBackLeftEncoder)
    back_right_module = SwerveModule(can.drivebaseBackRightDrive, can.drivebaseBackRightSteer, can.drivebaseBackRightEncoder)

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

    def __init__(self):
        pass

    def periodic(self) -> None:
        chassis_speeds = self.kinematics.toChassisSpeeds(
            (
                self.front_left_module.get_state_from_internal_encoders(),
                self.front_right_module.get_state_from_internal_encoders(),
                self.back_left_module.get_state_from_internal_encoders(),
                self.back_right_module.get_state_from_internal_encoders(),
            )
        )

        SmartDashboard.putNumber("drivebase/Chassis Speed VX", chassis_speeds.vx)
        SmartDashboard.putNumber("drivebase/Chassis Speed VY", chassis_speeds.vy)
        SmartDashboard.putNumber("drivebase/Chassis Speed Omega", chassis_speeds.omega)

    def simulationPeriodic(self) -> None:
        self.front_left_module.update_sim()
        self.front_right_module.update_sim()
        self.back_left_module.update_sim()
        self.back_right_module.update_sim()

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
