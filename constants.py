import types

# Pnuematics
pnuematics = types.SimpleNamespace()
pnuematics.intakeDeploy = 0
pnuematics.intakeRetract = 1
pnuematics.shooterUp = 2
pnuematics.shooterDown = 3

# CAN Bus
can = types.SimpleNamespace()
can.pnuematicsHub = 2
can.intakeMotor = 3
can.shooterLeftMotor = 4
can.shooterRightMotor = 5

# DIO
dio = types.SimpleNamespace()

# PWM
pwm = types.SimpleNamespace()