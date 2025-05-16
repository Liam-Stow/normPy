import wpilib
import rev
import commands2
import commands2.cmd as cmd

import constants


class Intake(commands2.Subsystem):
    # Actuators
    deploySolenoid = wpilib.DoubleSolenoid(
        constants.can.pnuematicsHub,
        wpilib.PneumaticsModuleType.REVPH,
        constants.pnuematics.intakeDeploy,
        constants.pnuematics.intakeRetract,
    )
    motor = rev.SparkMax(
        constants.can.intakeMotor, rev.SparkLowLevel.MotorType.kBrushless
    )

    def __init__(self) -> None:
        super().__init__()

    def cmdDeploy(self) -> commands2.Command:
        """Deployes the intake"""
        return cmd.runOnce(
            lambda: self.deploySolenoid.set(wpilib.DoubleSolenoid.Value.kForward), self
        )

    def cmdRetract(self) -> commands2.Command:
        """Retracts the intake"""
        return cmd.runOnce(
            lambda: self.deploySolenoid.set(wpilib.DoubleSolenoid.Value.kReverse), self
        )

    def cmdRun(self) -> commands2.Command:
        return cmd.startEnd(lambda: self.motor.set(1), lambda: self.motor.set(0))

    def periodic(self) -> None:
        if self.deploySolenoid.get() == wpilib.DoubleSolenoid.Value.kForward:
            wpilib.SmartDashboard.putString("intake/deployment", "deployed")
        else:
            wpilib.SmartDashboard.putString("intake/deployment", "retracted")
