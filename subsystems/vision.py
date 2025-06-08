from math import pi
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from photonlibpy.simulation import PhotonCameraSim, VisionSystemSim
from wpimath.geometry import Transform3d, Rotation3d, Pose2d
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

class Vision:
    def __init__(self):
        self.tag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)
        self.camera_name = "photonvision"
        self.camera = PhotonCamera(self.camera_name)
        self.bot_to_cam = Transform3d(-0.27, 0.27, 0.22, Rotation3d(0, 0.09, pi/4))
        self.pose_estimator = PhotonPoseEstimator(self.tag_layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, self.camera, self.bot_to_cam)
        self.pose_estimator.multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY

        self.camera_sim = PhotonCameraSim(self.camera)
        self.vision_sim = VisionSystemSim("Vision System Sim")

        self.vision_sim.addAprilTags(self.tag_layout)
        self.vision_sim.addCamera(self.camera_sim, self.bot_to_cam)

    def periodic(self):
        pass

    def update_sim_pose(self, pose: Pose2d):
        self.vision_sim.update(pose)

    