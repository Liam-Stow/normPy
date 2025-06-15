from commands2 import Command, Subsystem
from constants import can
from rev import SparkMax
from wpilib import SmartDashboard

class Feeder(Subsystem):
    def __init__(self):
        self.motor = SparkMax(can.feederMotor, SparkMax.MotorType.kBrushless)

    def feed_to_shooter(self) -> Command:
        return super().startEnd(
            lambda: self.motor.set(0.5),
            lambda: self.motor.set(0)
        )

    def feed_to_intake(self) -> Command:
        return super().startEnd(
            lambda: self.motor.set(-0.5),
            lambda: self.motor.set(0)
        )
    
    def periodic(self) -> None:
        SmartDashboard.putNumber("feeder/motor power", self.motor.get())
        