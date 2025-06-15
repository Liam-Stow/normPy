from commands2 import Command, cmd
from subsystems.intake_roller import IntakeRoller
from subsystems.intake_deploy import IntakeDeploy
from robotcontainer import RobotContainer

intake_deploy: IntakeDeploy = RobotContainer.intake_deploy
intake_roller: IntakeRoller = RobotContainer.intake_roller


def intake_sequence() -> Command:
    return cmd.parallel(
        intake_deploy.deploy(),
        intake_roller.spin_in()
    )