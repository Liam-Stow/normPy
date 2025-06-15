from commands2 import Subsystem, Command
from constants import can, pnuematics
from wpilib import DoubleSolenoid, PneumaticsModuleType, SmartDashboard

class IntakeDeploy(Subsystem):
    def __init__(self):
        self.deploy_solenoid = DoubleSolenoid(
            can.pnuematicsHub,
            PneumaticsModuleType.REVPH,
            pnuematics.intakeDeploy,
            pnuematics.intakeRetract,
        )

    def periodic(self) -> None:
        state = 'deployed' if self.deploy_solenoid.get() == DoubleSolenoid.Value.kForward else 'retracted'
        SmartDashboard.putString("intake deploy/deployment", state)

    def deploy(self) -> Command:
        return super().startEnd(
            lambda: self.deploy_solenoid.set(DoubleSolenoid.Value.kForward),
            lambda: self.deploy_solenoid.set(DoubleSolenoid.Value.kReverse)
        )