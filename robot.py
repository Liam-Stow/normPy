import commands2
import typing
from robotcontainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.autonomousCommand = self.container.get_autonomous_command()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""

    def testInit(self):
        """This function is called once each time the robot enters test mode."""
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self):
        """This function is called periodically during test mode."""
