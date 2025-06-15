import commands2
import rev
import wpilib
import wpilib.simulation
import wpimath.system.plant as plant

from constants import can


class IntakeRoller(commands2.Subsystem):

    # Constants
    gearing = 1.0
    MOI_KGM2 = 0.00001  # Moment of Inertia in kg*m^2


    def __init__(self):
        # Actuators
        self.motor = rev.SparkMax(can.intakeMotor, rev.SparkLowLevel.MotorType.kBrushless)

        # Simulation
        self.motor_sim = rev.SparkMaxSim(self.motor, plant.DCMotor.NEO())
        self.linear_system = plant.LinearSystemId.DCMotorSystem(
            plant.DCMotor.NEO(), self.MOI_KGM2, self.gearing
        )
        self.physics_Sim = wpilib.simulation.DCMotorSim(self.linear_system, plant.DCMotor.NEO())

    def spin_in(self) -> commands2.Command:
        return super().startEnd(
            lambda: self.motor.set(1), 
            lambda: self.motor.set(0)
        )

    def spin_out(self) -> commands2.Command:
        return super().startEnd(
            lambda: self.motor.set(-1), 
            lambda: self.motor.set(0)
        )

    def periodic(self) -> None:
        wpilib.SmartDashboard.putNumber("intake roller/motor output", self.motor.getAppliedOutput())

    def simulationPeriodic(self) -> None:
        self.physics_Sim.setInputVoltage(self.motor.getAppliedOutput() * 12.0)
        self.physics_Sim.update(0.02)
        rpm = self.physics_Sim.getAngularVelocityRPM()
        self.motor_sim.iterate(rpm, 12, 0.02)
