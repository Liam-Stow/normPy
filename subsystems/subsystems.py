from subsystems.intake_deploy import IntakeDeploy
from subsystems.intake_roller import IntakeRoller
from subsystems.shooter import Shooter
from subsystems.drivebase import Drivebase
from subsystems.vision import Vision
from subsystems.shooter_pivot import ShooterPivot
from subsystems.feeder import Feeder
from util.pose_estimator import PoseEstimator

intake_deploy: IntakeDeploy = IntakeDeploy()
intake_roller: IntakeRoller = IntakeRoller()
shooter: Shooter = Shooter()
shooter_pivot: ShooterPivot = ShooterPivot()
feeder: Feeder = Feeder()
drivebase: Drivebase = Drivebase()
vision: Vision = Vision()
pose_estimator: PoseEstimator = PoseEstimator(drivebase, vision)