import wpilib
import rev
import commands2
import wpilib.simulation
import wpimath.system.plant as plant
import wpimath.units as units
from wpimath.units import radiansPerSecondToRotationsPerMinute

import constants

class Shooter(commands2.Subsystem):
    # Actuators
    left_motor = rev.SparkMax(
        constants.can.shooterLeftMotor, rev.SparkLowLevel.MotorType.kBrushless
    )
    right_motor = rev.SparkMax(
        constants.can.shooterRightMotor, rev.SparkLowLevel.MotorType.kBrushless
    )

    # Sensors
    left_encoder = left_motor.getEncoder()
    right_encoder = right_motor.getEncoder()

    # Constants
    gearing = 1.0
    MOI_KGM2 = 0.01  # Moment of Inertia in kg*m^2

    # Controllers
    left_PID: rev.SparkClosedLoopController = left_motor.getClosedLoopController()
    right_PID: rev.SparkClosedLoopController = right_motor.getClosedLoopController()
    target_RPM: float = 0.0

    # Simulation
    left_motor_sim: rev.SparkMaxSim = rev.SparkMaxSim(left_motor, plant.DCMotor.NEO())
    right_motor_sim: rev.SparkMaxSim = rev.SparkMaxSim(right_motor, plant.DCMotor.NEO())
    linear_system = plant.LinearSystemId.flywheelSystem(
        plant.DCMotor.NEO(), MOI_KGM2, gearing
    )
    physics_sim = wpilib.simulation.FlywheelSim(linear_system, plant.DCMotor.NEO())

    def __init__(self) -> None:
        super().__init__()
        left_config = rev.SparkMaxConfig()
        left_config.closedLoop.P(0.002)
        left_config.closedLoop.I(0)
        left_config.closedLoop.D(0)
        left_config.closedLoop.velocityFF(
            1.0 / radiansPerSecondToRotationsPerMinute(plant.DCMotor.NEO().speed(0, 12))
        )
        self.left_motor.configure(
            left_config,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters,
        )

        right_config = rev.SparkMaxConfig()
        right_config.follow(constants.can.shooterLeftMotor)
        self.right_motor.configure(
            right_config,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters,
        )

    def periodic(self) -> None:
        wpilib.SmartDashboard.putNumber(
            "shooter/left motor RPM",
            self.left_encoder.getVelocity(),
        )

    def simulationPeriodic(self):
        self.physics_sim.setInputVoltage(self.left_motor_sim.getAppliedOutput() * 12.0)
        self.physics_sim.update(0.02)
        radsPerSec = self.physics_sim.getAngularVelocity()
        rpm = radiansPerSecondToRotationsPerMinute(radsPerSec)
        self.left_motor_sim.iterate(rpm, 12, 0.02)
        self.right_motor_sim.iterate(rpm, 12, 0.02)

    def __set_speed(self, targetRPM: units.revolutions_per_minute) -> None:
        """Sets the speed of the shooter motors"""
        err = None
        while err != rev.REVLibError.kOk:
            err = self.left_PID.setReference(
                targetRPM, rev.SparkLowLevel.ControlType.kVelocity
            )
        self.target_RPM = targetRPM

    def spin_up(self, targetRPM: units.revolutions_per_minute) -> commands2.Command:
        return (
            super()
            .runOnce(lambda: self.__set_speed(targetRPM))
            .andThen(commands2.cmd.idle())
        )

    def at_target_speed(self) -> commands2.button.Trigger:
        return commands2.button.Trigger(
            lambda: abs(self.left_encoder.getVelocity() - self.target_RPM) < 50
            and abs(self.right_encoder.getVelocity() - self.target_RPM) < 50
        )
