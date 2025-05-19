import wpilib
import rev
import commands2
import commands2.cmd as cmd
import wpilib.simulation
import wpimath.system.plant as plant

from constants import can, pnuematics


class Intake(commands2.Subsystem):
    # Actuators
    deploySolenoid = wpilib.DoubleSolenoid(
        can.pnuematicsHub,
        wpilib.PneumaticsModuleType.REVPH,
        pnuematics.intakeDeploy,
        pnuematics.intakeRetract,
    )
    motor = rev.SparkMax(
        can.intakeMotor, rev.SparkLowLevel.MotorType.kBrushless
    )

    # Constants
    gearing = 1.0
    MOI_KGM2 = 0.00001  # Moment of Inertia in kg*m^2

    # Simulation
    motorSim: rev.SparkMaxSim = rev.SparkMaxSim(motor, plant.DCMotor.NEO())
    linearSystem = plant.LinearSystemId.DCMotorSystem(plant.DCMotor.NEO(), MOI_KGM2, gearing)
    physicsSim = wpilib.simulation.DCMotorSim(linearSystem, plant.DCMotor.NEO())

    def __init__(self) -> None:
        super().__init__()

    def cmdDeploy(self) -> commands2.Command:
        return cmd.runOnce(
            lambda: self.deploySolenoid.set(wpilib.DoubleSolenoid.Value.kForward), self
        )

    def cmdRetract(self) -> commands2.Command:
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

    def simulationPeriodic(self) -> None:
        self.physicsSim.setInputVoltage(self.motor.getAppliedOutput() * 12.0)
        self.physicsSim.update(0.02)
        rpm = self.physicsSim.getAngularVelocityRPM()
        self.motorSim.iterate(rpm, 12, 0.02)
