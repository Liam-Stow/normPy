from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d
from wpimath.kinematics import SwerveModulePosition
from wpimath.units import seconds
from wpilib import Field2d, SmartDashboard
from subsystems.drivebase import Drivebase
from subsystems.vision import Vision

SwerveModulePositions4 = tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]

class PoseEstimator:
    def __init__(self, drivebase: Drivebase, vision: Vision):
        self.drivebase = drivebase
        self.vision = vision
        self.pose_estimator = SwerveDrive4PoseEstimator(
            drivebase.kinematics,
            drivebase.get_gyro_rotation(flip_on_red_alliance=False),
            drivebase.get_module_positions(),
            Pose2d()
        )
        self.field_display = Field2d()
        SmartDashboard.putData("pose estimator", self.field_display)

    def get_pose(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition()
    
    def update(self):
        self.pose_estimator.update(self.drivebase.get_gyro_rotation(flip_on_red_alliance=False), self.drivebase.get_module_positions())
        self.field_display.setRobotPose(self.get_pose())

    def set_pose(self, pose: Pose2d):
        self.pose_estimator.resetPosition(self.drivebase.get_gyro_rotation(flip_on_red_alliance=False), self.drivebase.get_module_positions(), pose)
        self.field_display.setRobotPose(pose)

    def add_vision_measurement(self, pose: Pose2d, timestamp_seconds: seconds):
        self.pose_estimator.addVisionMeasurement(pose, timestamp_seconds)
        self.field_display.setRobotPose(self.get_pose())

    def update_sim(self):
        self.vision.update_sim_pose(self.get_pose())