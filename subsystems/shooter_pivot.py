import commands2
from commands2.button import Trigger
import constants
from wpilib import DoubleSolenoid, PneumaticsModuleType, SmartDashboard

class ShooterPivot(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.solenoid = DoubleSolenoid(
            constants.can.pnuematicsHub,
            PneumaticsModuleType.REVPH,
            constants.pnuematics.shooterUp,
            constants.pnuematics.shooterDown,
        )

    def periodic(self) -> None:
        position = "low" if self.solenoid.get() == DoubleSolenoid.Value.kForward else "high"
        SmartDashboard.putString("shooter pivot/angle", position)

    def aim_low(self) -> commands2.Command:
        return super().runOnce(
            lambda: self.solenoid.set(DoubleSolenoid.Value.kForward)
        )

    def aim_high(self) -> commands2.Command:
        return super().runOnce(
            lambda: self.solenoid.set(DoubleSolenoid.Value.kReverse)
        )
    
    def is_aimed_low(self) -> Trigger:
        return Trigger(lambda: self.solenoid.get() == DoubleSolenoid.Value.kForward)