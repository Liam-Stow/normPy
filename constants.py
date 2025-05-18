import types

# Pnuematics
pnuematics = types.SimpleNamespace()
pnuematics.intakeDeploy = 0
pnuematics.intakeRetract = 1
pnuematics.shooterUp = 2
pnuematics.shooterDown = 3

# CAN Bus
can = types.SimpleNamespace()
can.drivebaseFrontLeftDrive = 1
can.drivebaseFrontLeftSteer = 2
can.drivebaseFrontRightDrive = 3
can.drivebaseFrontRightSteer = 4
can.drivebaseBackLeftDrive = 5
can.drivebaseBackLeftSteer = 6
can.drivebaseBackRightDrive = 7
can.drivebaseBackRightSteer = 8
can.drivebaseFrontLeftEncoder = 9
can.drivebaseFrontRightEncoder = 10
can.drivebaseBackLeftEncoder = 11
can.drivebaseBackRightEncoder = 12
can.pnuematicsHub = 2
can.intakeMotor = 3
can.shooterLeftMotor = 4
can.shooterRightMotor = 5

# DIO
dio = types.SimpleNamespace()

# PWM
pwm = types.SimpleNamespace()