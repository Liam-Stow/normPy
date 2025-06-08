import commands2
from subsystems.intake import Intake
from subsystems.shooter import Shooter
from subsystems.drivebase import Drivebase
from subsystems.vision import Vision
from util.pose_estimator import PoseEstimator
from wpilib import SmartDashboard, DriverStation
from wpilib.interfaces import GenericHID
from pathplannerlib.auto import AutoBuilder # type: ignore
from pathplannerlib.controller import PPHolonomicDriveController # type: ignore
from pathplannerlib.config import RobotConfig, PIDConstants # type: ignore
from wpimath.units import seconds
from wpimath.geometry import Pose2d

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.intake = Intake()
        self.shooter = Shooter()
        self.drivebase = Drivebase()
        self.vision = Vision()
        self.pose_estimator = PoseEstimator(self.drivebase, self.vision)
        self.driver_controller = commands2.button.CommandXboxController(0)

        self.drivebase.setDefaultCommand(
            self.drivebase.joystick_drive(self.driver_controller, True)
        )

        # Pathplanner config
        AutoBuilder.configure(
            self.pose_estimator.get_pose,
            self.pose_estimator.set_pose,
            self.drivebase.get_robot_relative_speeds,
            lambda speeds, feedforwards: self.drivebase.drive_implementation(
                speeds, feedforwards, False
            ),
            PPHolonomicDriveController(
                PIDConstants(3.2, 0, 0.3),# Translation PID
                PIDConstants(1.5, 0, 0),  # Rotation PID
            ),
            RobotConfig.fromGUISettings(),
            self.should_flip_path,
            self.drivebase,
        )

        self.auto_chooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.auto_chooser)

        self.configure_button_bindings()
        self.configure_driver_feedback()

    def configure_button_bindings(self):
        self.driver_controller.a().whileTrue(
            self.intake.deploy().andThen(self.intake.spin_in())
        ).onFalse(self.intake.retract())

        self.driver_controller.b().whileTrue(self.drivebase.drive(lambda: self.drivebase.calc_drive_to_pose_speeds(self.pose_estimator.get_pose(), Pose2d(1, 1, 0)),True))
        self.driver_controller.x().onTrue(self.shooter.spin_up(2000))
        self.driver_controller.y().onTrue(self.shooter.spin_up(0))
        self.driver_controller.back().onTrue(self.shooter.aim_high())
        self.driver_controller.start().onTrue(self.shooter.aim_low())

    def configure_driver_feedback(self):
        self.shooter.at_target_speed().onTrue(self.rumble(seconds=0.5))

    def get_autonomous_command(self) -> commands2.Command:
        return self.auto_chooser.getSelected()

    def rumble(self, seconds: seconds) -> commands2.Command:
        return (
            commands2.cmd.runOnce(
                lambda: self.driver_controller.setRumble(
                    GenericHID.RumbleType.kBothRumble, 1
                )
            )
            .andThen(commands2.cmd.waitSeconds(seconds))
            .andThen(
                commands2.cmd.runOnce(
                    lambda: self.driver_controller.setRumble(
                        GenericHID.RumbleType.kBothRumble, 0
                    )
                )
            )
        )
    
    def should_flip_path(self) -> bool:
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
