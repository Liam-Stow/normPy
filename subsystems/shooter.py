import wpilib
import rev
import commands2
import commands2.cmd as cmd

import constants


class Shooter(commands2.Subsystem):
    # Actuators
    leftMotor = rev.SparkMax(
        constants.can.shooterLeftMotor, rev.SparkLowLevel.MotorType.kBrushless
    )
    rightMotor = rev.SparkMax(
        constants.can.shooterRightMotor, rev.SparkLowLevel.MotorType.kBrushless
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
    leftPID: rev.SparkClosedLoopController = leftMotor.getClosedLoopController()
    rightPID: rev.SparkClosedLoopController = rightMotor.getClosedLoopController()
    currentTargetRPM: float = 0.0

    def __init__(self) -> None:
        super().__init__()
        leftConfig = rev.SparkMaxConfig()
        leftConfig.closedLoop.P(1)
        leftConfig.closedLoop.I(0)
        leftConfig.closedLoop.D(0)
        leftConfig.closedLoop.velocityFF(12.0 / 5000.0)
        self.leftMotor.configure(
            leftConfig,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters,
        )

    def periodic(self) -> None:
        if self.angleSolenoid.get() == wpilib.DoubleSolenoid.Value.kForward:
            wpilib.SmartDashboard.putString("shooter/angle", "low")
        else:
            wpilib.SmartDashboard.putString("shooter/angle", "high")

        wpilib.SmartDashboard.putNumber("shooter/left power", self.leftMotor.get())
        wpilib.SmartDashboard.putNumber("shooter/right power", self.rightMotor.get())

    def setSpeed(self, targetRPM: float) -> None:
        """Sets the speed of the shooter motors"""
        err = None
        while err != rev.REVLibError.kOk:
            err = self.leftPID.setReference(targetRPM, rev.SparkLowLevel.ControlType.kVelocity)
        self.currentTargetRPM = targetRPM

    def cmdSpinUp(self, targetRPM) -> commands2.Command:
        return super().run(lambda: self.setSpeed(targetRPM)).until(self.isAtTargetSpeed)

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
