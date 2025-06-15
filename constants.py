from enum import IntEnum, unique

# Pnuematics
@unique
class pnuematics(IntEnum):
    intakeDeploy = 0
    intakeRetract = 1
    shooterUp = 2
    shooterDown = 3

# CAN Bus
@unique
class can(IntEnum):
    drivebaseFrontLeftDrive = 1
    drivebaseFrontLeftSteer = 2
    drivebaseFrontRightDrive = 3
    drivebaseFrontRightSteer = 4
    drivebaseBackLeftDrive = 5
    drivebaseBackLeftSteer = 6
    drivebaseBackRightDrive = 7
    drivebaseBackRightSteer = 8
    drivebaseFrontLeftEncoder = 9
    drivebaseFrontRightEncoder = 10
    drivebaseBackLeftEncoder = 11
    drivebaseBackRightEncoder = 12
    drivebaseGyro = 13
    pnuematicsHub = 14
    intakeMotor = 15
    shooterLeftMotor = 16
    shooterRightMotor = 17
    feederMotor = 18
