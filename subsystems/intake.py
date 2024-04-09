import wpilib
import rev
import commands2
import commands2.cmd as cmd

import constants


class Intake(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.deploySolenoid = wpilib.DoubleSolenoid(
            constants.pnuematicsHub,
            wpilib.PneumaticsModuleType.REVPH,
            constants.intakeDeploy,
            constants.intakeRetract,
        )

        self.motor = rev.CANSparkMax(constants.intakeMotor, rev.CANSparkLowLevel.MotorType.kBrushless)

    def cmdDeploy(self) -> commands2.Command:
        """Grabs the hatch"""
        return cmd.runOnce(
            lambda: self.deploySolenoid.set(wpilib.DoubleSolenoid.Value.kForward), self
        )

    def cmdRetract(self) -> commands2.Command:
        """Releases the hatch"""
        return cmd.runOnce(
            lambda: self.deploySolenoid.set(wpilib.DoubleSolenoid.Value.kReverse), self
        )

    def periodic(self) -> None:
        if self.deploySolenoid.get() == wpilib.DoubleSolenoid.Value.kForward:
            wpilib.SmartDashboard.putString("intake/deployment", "deployed")
        else:
            wpilib.SmartDashboard.putString("intake/deployment", "retracted")
