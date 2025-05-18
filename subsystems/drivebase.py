from util.swerve_module import SwerveModule
from constants import can
from commands2.button import CommandXboxController
from wpimath.kinematics import ChassisSpeeds
from typing import Callable
from commands2 import Command, Subsystem
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d

class Drivebase(Subsystem):
    def __init__(self):
        self.front_left_module = SwerveModule(can.frontLeftDriveMotor, can.frontLeftSteerMotor, can.frontLeftSteerEncoder)
        self.front_right_module = SwerveModule(can.frontRightDriveMotor, can.frontRightSteerMotor, can.frontRightSteerEncoder)
        self.back_left_module = SwerveModule(can.backLeftDriveMotor, can.backLeftSteerMotor, can.backLeftSteerEncoder)
        self.back_right_module = SwerveModule(can.backRightDriveMotor, can.backRightSteerMotor, can.backRightSteerEncoder)

        self.front_left_location = Translation2d(0.381, 0.381)
        self.front_right_location = Translation2d(0.381, -0.381)
        self.back_left_location = Translation2d(-0.381, 0.381)
        self.back_right_location = Translation2d(-0.381, -0.381)

        self.kinematics = SwerveDrive4Kinematics(self.front_left_location, self.front_right_location, self.back_left_location, self.back_right_location)
    
        
    def calc_joystick_speeds(self, controller: CommandXboxController):
        pass

    def drive(self, speeds: Callable[[],ChassisSpeeds], field_oriented: bool) -> Command:
        def drive_impl(speeds: ChassisSpeeds):
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vx, speeds.vy, speeds.omega, self.get_gyro_rotation()) if field_oriented else speeds
            speeds = ChassisSpeeds.discretize(speeds, 0.02)
        
        return super().run(lambda: drive_impl(speeds()))

    def get_gyro_rotation(self) -> Rotation2d:
        pass