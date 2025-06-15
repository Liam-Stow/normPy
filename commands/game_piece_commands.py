from commands2 import Command, cmd
from commands2.button import Trigger
from subsystems.subsystems import intake_roller, intake_deploy, drivebase, pose_estimator, shooter, shooter_pivot
from wpimath.geometry import Pose2d, Rotation2d
from wpilib import DriverStation
from wpimath.units import rotationsToRadians

def intake_sequence() -> Command:
    return cmd.parallel(
        intake_deploy.deploy(),
        intake_roller.spin_in()
    )

def prepare_score() -> Command:
    blue_score_pose = Pose2d(2.8, 5.5, Rotation2d(rotationsToRadians(0.5)))
    red_score_pose = Pose2d(13.6, 5.5, Rotation2d(rotationsToRadians(0)))
    score_pose = red_score_pose if DriverStation.getAlliance() == DriverStation.Alliance.kRed else blue_score_pose
    calc_drive_to_score_speeds = lambda: drivebase.calc_drive_to_pose_speeds(pose_estimator.get_pose(), blue_score_pose)

    pose_estimator.display_pose("target pose", score_pose)

    return cmd.parallel(
        drivebase.drive(calc_drive_to_score_speeds, True),
        shooter.spin_up(2000),
        shooter_pivot.aim_low()
    )

def is_ready_to_score() -> Trigger:
    blue_score_pose = Pose2d(2.8, 5.5, Rotation2d(rotationsToRadians(0.5)))
    red_score_pose = Pose2d(13.6, 5.5, Rotation2d(rotationsToRadians(0)))
    score_pose = red_score_pose if DriverStation.getAlliance() == DriverStation.Alliance.kRed else blue_score_pose

    return pose_estimator.is_at_pose_trigger(score_pose).and_(shooter.at_target_speed()).and_(shooter_pivot.is_aimed_low())