from commands2 import Command, cmd
from subsystems.subsystems import intake_roller, intake_deploy


def intake_sequence() -> Command:
    return cmd.parallel(
        intake_deploy.deploy(),
        intake_roller.spin_in()
    )