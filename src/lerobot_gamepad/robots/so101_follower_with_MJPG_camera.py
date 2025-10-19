from lerobot.robots.so101_follower import SO101Follower
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus
)
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.cameras.camera import Camera
from lerobot.cameras.configs import CameraConfig, Cv2Rotation

def make_cameras_from_configs_MJPG(camera_configs: dict[str, CameraConfig]) -> dict[str, Camera]:
    cameras = make_cameras_from_configs(camera_configs)
    for key, cfg in camera_configs.items():
        # TODO(Steven): Consider just using the make_device_from_device_class for all types
        if cfg.type == "opencv":
            from lerobot_gamepad.cameras.opencv_camera_MJPG import OpenCVCameraMJPG
            cameras[key] = OpenCVCameraMJPG(cfg)

    return cameras

class SO101FollowerWithMJPGCamera(SO101Follower):
    def __init__(self, config: SO101FollowerConfig):
        super().__init__(config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                "elbow_flex": Motor(3, "sts3215", norm_mode_body),
                "wrist_flex": Motor(4, "sts3215", norm_mode_body),
                "wrist_roll": Motor(5, "sts3215", norm_mode_body),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs_MJPG(config.cameras)