import commands2
from subsystems.intake import Intake
from subsystems.shooter import Shooter

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
        self.driverController = commands2.button.CommandXboxController(0)
        self.configureButtonBindings()

    def configureButtonBindings(self):
        self.driverController.a().onTrue(self.intake.cmdDeploy())
        self.driverController.b().onTrue(self.intake.cmdRetract())
        self.driverController.x().onTrue(self.shooter.cmdSpinUp(2000))
        self.driverController.y().onTrue(self.shooter.cmdSpinUp(0))
        self.driverController.back().onTrue(self.shooter.cmdAimHigh())
        self.driverController.start().onTrue(self.shooter.cmdAimLow())

    def getAutonomousCommand(self) -> commands2.Command:
        return commands2.cmd.none()
