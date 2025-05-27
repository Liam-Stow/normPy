import wpilib
import rev
import commands2
import commands2.cmd as cmd
import wpilib.simulation
import wpimath.system.plant as plant

from constants import can, pnuematics


class Intake(commands2.Subsystem):
    # Actuators
    deploy_solenoid = wpilib.DoubleSolenoid(
        can.pnuematicsHub,
        wpilib.PneumaticsModuleType.REVPH,
        pnuematics.intakeDeploy,
        pnuematics.intakeRetract,
    )
    motor = rev.SparkMax(can.intakeMotor, rev.SparkLowLevel.MotorType.kBrushless)

    # Constants
    gearing = 1.0
    MOI_KGM2 = 0.00001  # Moment of Inertia in kg*m^2

    # Simulation
    motor_sim: rev.SparkMaxSim = rev.SparkMaxSim(motor, plant.DCMotor.NEO())
    linear_system = plant.LinearSystemId.DCMotorSystem(
        plant.DCMotor.NEO(), MOI_KGM2, gearing
    )
    physics_Sim = wpilib.simulation.DCMotorSim(linear_system, plant.DCMotor.NEO())

    def __init__(self) -> None:
        super().__init__()

    def deploy(self) -> commands2.Command:
        return cmd.runOnce(
            lambda: self.deploy_solenoid.set(wpilib.DoubleSolenoid.Value.kForward), self
        )

    def retract(self) -> commands2.Command:
        return cmd.runOnce(
            lambda: self.deploy_solenoid.set(wpilib.DoubleSolenoid.Value.kReverse), self
        )

    def spin_in(self) -> commands2.Command:
        return cmd.startEnd(lambda: self.motor.set(1), lambda: self.motor.set(0))

    def periodic(self) -> None:
        if self.deploy_solenoid.get() == wpilib.DoubleSolenoid.Value.kForward:
            wpilib.SmartDashboard.putString("intake/deployment", "deployed")
        else:
            wpilib.SmartDashboard.putString("intake/deployment", "retracted")

    def simulationPeriodic(self) -> None:
        self.physics_Sim.setInputVoltage(self.motor.getAppliedOutput() * 12.0)
        self.physics_Sim.update(0.02)
        rpm = self.physics_Sim.getAngularVelocityRPM()
        self.motor_sim.iterate(rpm, 12, 0.02)
