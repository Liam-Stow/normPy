import wpilib
import rev
import commands2
import commands2.cmd as cmd

import constants


class Shooter(commands2.Subsystem):
    # Actuators
    leftMotor = rev.CANSparkMax(
        constants.can.shooterLeftMotor, rev.CANSparkLowLevel.MotorType.kBrushless
    )
    rightMotor = rev.CANSparkMax(
        constants.can.shooterRightMotor, rev.CANSparkLowLevel.MotorType.kBrushless
    )
    angleSolenoid = wpilib.DoubleSolenoid(
        constants.can.pnuematicsHub,
        wpilib.PneumaticsModuleType.REVPH,
        constants.pnuematics.shooterUp,
        constants.pnuematics.shooterDown,
    )

    # Sensors
    leftEncoder = leftMotor.getEncoder()
    rightEncoder = rightMotor.getEncoder()

    # Controllers
    leftPID = leftMotor.getPIDController()
    rightPID = rightMotor.getPIDController()
    p = 1
    i = 0
    d = 0
    ff = 12.0 / 5000.0
    currentTargetRPM = 0

    def __init__(self) -> None:
        super().__init__()
        self.leftPID.setP(self.p)
        self.leftPID.setI(self.i)
        self.leftPID.setD(self.d)
        self.leftPID.setFF(self.ff)
        self.rightPID.setP(self.p)
        self.rightPID.setI(self.i)
        self.rightPID.setD(self.d)
        self.rightPID.setFF(self.ff)

    def periodic(self) -> None:
        if self.angleSolenoid.get() == wpilib.DoubleSolenoid.Value.kForward:
            wpilib.SmartDashboard.putString("shooter/angle", "low")
        else:
            wpilib.SmartDashboard.putString("shooter/angle", "high")

        wpilib.SmartDashboard.putNumber("shooter/left power", self.leftMotor.get())
        wpilib.SmartDashboard.putNumber("shooter/right power", self.rightMotor.get())

    def cmdSpinUp(self, targetRPM) -> commands2.Command:
        self.currentTargetRPM = targetRPM
        return super().run(
            lambda: self.leftPID.setReference(
                targetRPM, rev.CANSparkLowLevel.ControlType.kVelocity
            )
        ).until(self.isAtTargetSpeed)

    def cmdAimLow(self) -> commands2.Command:
        return super().runOnce(
            lambda: self.angleSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)
        )

    def cmdAimHigh(self) -> commands2.Command:
        return super().runOnce(
            lambda: self.angleSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
        )

    def isAtTargetSpeed(self) -> bool:
        return (
            self.leftEncoder.getVelocity() == self.currentTargetRPM
            and self.rightEncoder.getVelocity() == self.currentTargetRPM
        )
