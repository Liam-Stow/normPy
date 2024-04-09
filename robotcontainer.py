import commands2
from subsystems.intake import Intake

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.intake = Intake()
        self.driverController = commands2.button.CommandXboxController(0)
        self.configureButtonBindings()

    def configureButtonBindings(self):
        self.driverController.a().onTrue(self.intake.cmdDeploy())
        self.driverController.b().onTrue(self.intake.cmdRetract())

    def getAutonomousCommand(self) -> commands2.Command:
        return commands2.cmd.none()
