from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import SwerveModulePosition
from wpilib import Field2d, SmartDashboard
from subsystems.drivebase import Drivebase
from subsystems.vision import Vision
from wpimath.units import meters
from commands2 import Subsystem
from commands2.button import Trigger

SwerveModulePositions4 = tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]

class PoseEstimator(Subsystem):
    def __init__(self, drivebase: Drivebase, vision: Vision):
        self.drivebase = drivebase
        self.vision = vision
        self.pose_estimator = SwerveDrive4PoseEstimator(
            drivebase.kinematics,
            drivebase.get_gyro_rotation(flip_on_red_alliance=False),
            drivebase.get_module_positions(),
            Pose2d()
        )
        self.sim_pose_tracker = SwerveDrive4PoseEstimator(
            drivebase.kinematics,
            drivebase.get_gyro_rotation(flip_on_red_alliance=False),
            drivebase.get_module_positions(),
            Pose2d(),
            (0.2, 0.2, 0.2),
            (0.9, 0.9, 0.9)
        )

        self.field_display = Field2d()
        SmartDashboard.putData("pose estimator", self.field_display)
        self.estimate_display = self.field_display.getObject("estimated pose")
        self.usable_vision_display = self.field_display.getObject("usable vision estimate")
        self.discarded_vision_display = self.field_display.getObject("discarded vision estimate")
        self.simulated_robot_display = self.field_display.getObject("simulated robot")
        # self.target_pose_display = self.field_display.getObject("target pose")

    def periodic(self):
        # Add odometry measurement 
        self.pose_estimator.update(self.drivebase.get_gyro_rotation(flip_on_red_alliance=False), self.drivebase.get_module_positions())

        # Add vision measurement if available
        vision_estimate = self.vision.get_latest_pose_estimate()

        if vision_estimate is None:
            self.usable_vision_display.setPose(Pose2d())
            self.discarded_vision_display.setPose(Pose2d())
        elif not self.vision.is_estimate_usable(vision_estimate):
            self.usable_vision_display.setPose(Pose2d())
            self.discarded_vision_display.setPose(vision_estimate.estimatedPose.toPose2d())
        else:
            self.usable_vision_display.setPose(vision_estimate.estimatedPose.toPose2d())
            self.discarded_vision_display.setPose(Pose2d())
            translation_std_dev = self.vision.get_std_deviation(vision_estimate)
            rotation_std_dev = 0.9
            std_devs = (translation_std_dev, translation_std_dev, rotation_std_dev)
            self.pose_estimator.addVisionMeasurement(vision_estimate.estimatedPose.toPose2d(), vision_estimate.timestampSeconds, std_devs)

        self.estimate_display.setPose(self.get_pose())
    
    def get_pose(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition() 
    
    def is_at_pose(self, pose: Pose2d, translation_tolerance_m: meters = 0.1, rotation_tolerance: Rotation2d = Rotation2d(0.03)) -> bool:
        current_pose = self.get_pose()
        # self.target_pose_display.setPose(pose)
        return (
            (current_pose - pose).translation().norm() < translation_tolerance_m 
            and abs(current_pose.rotation().radians() - pose.rotation().radians()) < rotation_tolerance.radians()
        )

    def is_at_pose_trigger(self, pose: Pose2d, translation_tolerance_m: meters = 0.1, rotation_tolerance: Rotation2d = Rotation2d(0.03)) -> Trigger:
        return Trigger(lambda: self.is_at_pose(pose, translation_tolerance_m, rotation_tolerance))

    def set_pose(self, pose: Pose2d):
        self.pose_estimator.resetPosition(self.drivebase.get_gyro_rotation(flip_on_red_alliance=False), self.drivebase.get_module_positions(), pose)
        self.estimate_display.setPose(self.get_pose())

    def display_pose(self, name: str, pose: Pose2d):
        self.field_display.getObject(name).setPose(pose)
    
    def simulationPeriodic(self):
        self.sim_pose_tracker.update(self.drivebase.get_gyro_rotation(flip_on_red_alliance=False), self.drivebase.get_module_positions())
        self.vision.update_sim_pose(self.sim_pose_tracker.getEstimatedPosition())
        self.simulated_robot_display.setPose(self.sim_pose_tracker.getEstimatedPosition())