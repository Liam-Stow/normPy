import commands2
from subsystems.subsystems import drivebase, shooter, pose_estimator, feeder
from wpilib import SmartDashboard, DriverStation
from wpilib.interfaces import GenericHID
from pathplannerlib.auto import AutoBuilder # type: ignore
from pathplannerlib.controller import PPHolonomicDriveController # type: ignore
from pathplannerlib.config import RobotConfig, PIDConstants # type: ignore
from wpimath.units import seconds
from commands import game_piece_commands

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.driver_controller = commands2.button.CommandXboxController(0)

        drivebase.setDefaultCommand(
            drivebase.joystick_drive(self.driver_controller, True)
        )

        # Pathplanner config
        AutoBuilder.configure(
            pose_estimator.get_pose,
            pose_estimator.set_pose,
            drivebase.get_robot_relative_speeds,
            lambda speeds, feedforwards: drivebase.drive_implementation(
                speeds, feedforwards, False
            ),
            PPHolonomicDriveController(
                PIDConstants(3.2, 0, 0.3),# Translation PID
                PIDConstants(1.5, 0, 0),  # Rotation PID
            ),
            RobotConfig.fromGUISettings(),
            self.should_flip_path,
            drivebase,
        )

        self.auto_chooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.auto_chooser)

        self.configure_button_bindings()
        self.configure_driver_feedback()

    def configure_button_bindings(self):
        self.driver_controller.a().whileTrue(game_piece_commands.intake_sequence())
        self.driver_controller.b().and_(game_piece_commands.is_ready_to_score().negate()).whileTrue(game_piece_commands.prepare_score())
        self.driver_controller.b().and_(game_piece_commands.is_ready_to_score()).whileTrue(feeder.feed_to_shooter())

    def configure_driver_feedback(self):
        shooter.at_target_speed().onTrue(self.rumble(seconds=0.5))

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
