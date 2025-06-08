from math import pi
from statistics import mean
from commands2 import Subsystem
from photonlibpy import EstimatedRobotPose
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult
from photonlibpy.simulation import PhotonCameraSim, VisionSystemSim
from wpimath.geometry import Transform3d, Rotation3d, Pose2d
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from util.linear_interpolation_map import LinearInterpolationMap


class Vision(Subsystem):
    def __init__(self):
        self.tag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)
        self.camera_name = "photonvision"
        self.camera = PhotonCamera(self.camera_name)
        self.bot_to_cam = Transform3d(0, 0, 0, Rotation3d(0, 0, 0))
        self.pose_estimator = PhotonPoseEstimator(self.tag_layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, self.camera, self.bot_to_cam)
        self.pose_estimator.multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY
        self.latest_pose_estimate = None

        # Standard deviation map for pose estimates at various distances
        self.std_dev_map_meters = LinearInterpolationMap()
        self.std_dev_map_meters.insert(0.0, 0.0)
        self.std_dev_map_meters.insert(0.71, 0.002)
        self.std_dev_map_meters.insert(1.0, 0.006)
        self.std_dev_map_meters.insert(1.5, 0.02)
        self.std_dev_map_meters.insert(2.0, 0.068)
        self.std_dev_map_meters.insert(3.0, 0.230)
        
        # Simulation setup
        self.camera_sim = PhotonCameraSim(self.camera)
        self.vision_sim = VisionSystemSim("Vision System Sim")
        self.vision_sim.addAprilTags(self.tag_layout)
        self.vision_sim.addCamera(self.camera_sim, self.bot_to_cam)

    def periodic(self):
        results = self.camera.getAllUnreadResults()
        self.latest_pose_estimate = self.update_pose_estimator(results)

    def update_sim_pose(self, pose: Pose2d):
        self.vision_sim.update(pose)

    def update_pose_estimator(self, results: list[PhotonPipelineResult]) -> EstimatedRobotPose | None:
        estimated_pose = None
        for result in results:
            estimated_pose = self.pose_estimator.update(result)
        return estimated_pose

    def get_latest_pose_estimate(self) -> EstimatedRobotPose | None:
        return self.latest_pose_estimate
    
    def get_std_deviation(self, pose_estimate: EstimatedRobotPose) -> float:
        if (len(pose_estimate.targetsUsed) == 0):
            return 0.0
        
        average_distance_m = mean([target.bestCameraToTarget.translation().norm() for target in pose_estimate.targetsUsed])
        return self.std_dev_map_meters.interpolate(average_distance_m) or 1.0

    def is_estimate_usable(self, pose_estimate: EstimatedRobotPose) -> bool:
        if len(pose_estimate.targetsUsed) == 0:
            return False
        
        tag_count = len(pose_estimate.targetsUsed)
        if tag_count == 1:
            distance_meters = pose_estimate.targetsUsed[0].bestCameraToTarget.translation().norm()
            return distance_meters < 0.7 
        else: #if tag_count > 1
            min_distance_meters = min([target.bestCameraToTarget.translation().norm() for target in pose_estimate.targetsUsed])
            return min_distance_meters < 3.5