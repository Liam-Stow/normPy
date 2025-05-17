import wpilib
import rev
import commands2
import commands2.cmd as cmd
import wpilib.simulation
import wpimath.system.plant as plant
from math import pi
from wpimath.units import radiansPerSecondToRotationsPerMinute

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

    # Constants
    gearing = 1.0
    MOI_KGM2 = 0.01  # Moment of Inertia in kg*m^2

    # Controllers
    leftPID: rev.SparkClosedLoopController = leftMotor.getClosedLoopController()
    rightPID: rev.SparkClosedLoopController = rightMotor.getClosedLoopController()
    targetRPM: float = 0.0

    # Simulation
    leftMotorSim: rev.SparkMaxSim = rev.SparkMaxSim(leftMotor, plant.DCMotor.NEO())
    rightMotorSim: rev.SparkMaxSim = rev.SparkMaxSim(rightMotor, plant.DCMotor.NEO())
    linearSystem = plant.LinearSystemId.flywheelSystem(plant.DCMotor.NEO(), MOI_KGM2, gearing)
    physicsSim = wpilib.simulation.FlywheelSim(linearSystem, plant.DCMotor.NEO())

    def __init__(self) -> None:
        super().__init__()
        leftConfig = rev.SparkMaxConfig()
        leftConfig.closedLoop.P(0.002)
        leftConfig.closedLoop.I(0)
        leftConfig.closedLoop.D(0)
        leftConfig.closedLoop.velocityFF(1.0 / radiansPerSecondToRotationsPerMinute(plant.DCMotor.NEO().speed(0, 12)))
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

    def simulationPeriodic(self):
        self.physicsSim.setInputVoltage(self.leftMotorSim.getAppliedOutput() * 12.0)
        self.physicsSim.update(0.02)
        radsPerSec = self.physicsSim.getAngularVelocity()
        rpm = radiansPerSecondToRotationsPerMinute(radsPerSec)
        self.leftMotorSim.iterate(rpm, 12, 0.02)

    def setSpeed(self, targetRPM: float) -> None:
        """Sets the speed of the shooter motors"""
        err = None
        while err != rev.REVLibError.kOk:
            err = self.leftPID.setReference(targetRPM, rev.SparkLowLevel.ControlType.kVelocity)
        self.targetRPM = targetRPM

    def cmdSpinUp(self, targetRPM) -> commands2.Command:
        return super().runOnce(lambda: self.setSpeed(targetRPM))

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
            self.leftEncoder.getVelocity() == self.targetRPM
            and self.rightEncoder.getVelocity() == self.targetRPM
        )
