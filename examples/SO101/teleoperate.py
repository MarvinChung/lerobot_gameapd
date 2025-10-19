# !/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
import time
import argparse
from dataclasses import dataclass

import numpy as np
import torch

from lerobot_teleoperator_deltas_gamepad import (
    DeltasGamepad as GamepadTeleop,
    DeltasGamepadConfig as GamepadTeleopConfig,
)
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig, Cv2Rotation
from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import (
    MapDeltaActionToRobotActionStep,
    RobotAction,
    RobotActionProcessorStep,
    RobotObservation,
    RobotProcessorPipeline,
    TransitionKey,
    create_transition,
)
from lerobot.processor.converters import identity_transition, transition_to_robot_action
from lerobot.robots.robot import Robot
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so100_follower.robot_kinematic_processor import (
    EEBoundsAndSafety,
    EEReferenceAndDelta,
    GripperVelocityToJoint,
    InverseKinematicsRLStep,
)
from lerobot_gamepad.robots.so101_follower_with_MJPG_camera import SO101FollowerWithMJPGCamera as SO101Follower
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

# This Pacakge
from lerobot_gamepad import ASSET_ROOT

# -----------------------------
# Helper classes & functions
# -----------------------------
@dataclass
class LogRobotAction(RobotActionProcessorStep):
    def action(self, action: RobotAction) -> RobotAction:
        print(f"Robot action: {action}")
        return action

    def transform_features(self, features):
        # features[PipelineFeatureType.ACTION][ACTION] = PolicyFeature(
        #     type=FeatureType.ACTION, shape=(len(self.motor_names),)
        # )
        return features


def reset_follower_position(robot_arm: Robot, target_position: np.ndarray) -> None:
    """Smoothly reset robot arm to a target joint position."""
    current_position_dict = robot_arm.bus.sync_read("Present_Position")
    current_position = np.array(
        [current_position_dict[name] for name in current_position_dict],
        dtype=np.float32,
    )

    trajectory = torch.from_numpy(np.linspace(current_position, target_position, 50))
    for pose in trajectory:
        action_dict = dict(zip(current_position_dict, pose, strict=False))
        robot_arm.bus.sync_write("Goal_Position", action_dict)
        busy_wait(0.015)


# -----------------------------
# Main entry point
# -----------------------------
def main(args):
    # NOTE: The urdf is copied from the SO-ARM100 repo: https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
    urdf_path = os.path.join(ASSET_ROOT, "SO101/so101_new_calib.urdf")

    camera_config = {
        # If rotate 90 or 270, the width and height should reverse
        # "front": OpenCVCameraConfig(index_or_path=args.front_camera_port, width=480, height=640, fps=30, rotation=Cv2Rotation.ROTATE_90),
        "front": OpenCVCameraConfig(index_or_path=args.front_camera_port, width=640, height=480, fps=30),
        "wrist": OpenCVCameraConfig(index_or_path=args.wrist_camera_port, width=640, height=480, fps=30),
    }

    follower_config = SO101FollowerConfig(
        port=args.port,
        id=args.arm_id,
        cameras=camera_config,
        use_degrees=True,
    )
    gamepad_config = GamepadTeleopConfig(use_gripper=True)

    follower = SO101Follower(follower_config)
    gamepad = GamepadTeleop(gamepad_config)

    follower_kinematics_solver = RobotKinematics(
        urdf_path=urdf_path,
        target_frame_name="gripper_frame_link",
        joint_names=list(follower.bus.motors.keys()),
    )

   # Build the processing steps conditionally based on debug mode
    steps = [
        MapDeltaActionToRobotActionStep(),
        EEReferenceAndDelta(
            kinematics=follower_kinematics_solver,
            end_effector_step_sizes={
                "x": 0.006,
                "y": 0.01,
                "z": 0.005,
                "wx": 0.03490658503988659,
                "wy": 0.05235987755982988,
                "wz": 0.08726646259971647,
            },
            motor_names=list(follower.bus.motors.keys()),
            use_latched_reference=False,
            use_ik_solution=True,
        ),
        EEBoundsAndSafety(
            end_effector_bounds={
                "min": [0.115, -0.165, -0.00175],
                "max": [0.28, 0.16, 0.06],
            },
            max_ee_step_m=0.1,
        ),
        GripperVelocityToJoint(
            clip_max=30.0,
            speed_factor=0.075,
        ),
        InverseKinematicsRLStep(
            kinematics=follower_kinematics_solver,
            motor_names=list(follower.bus.motors.keys()),
            initial_guess_current_joints=False,
        ),
    ]

    # If debug mode is enabled, interleave LogRobotAction() for inspection
    if args.debug:
        debug_steps = []
        for step in steps:
            debug_steps.extend([LogRobotAction(), step])
        steps = debug_steps + [LogRobotAction()]

    # Build the full pipeline
    ee_to_follower_joints = RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ](
        steps,
        to_transition=identity_transition,
        to_output=transition_to_robot_action,
    )

    # Connect devices
    follower.connect()
    gamepad.connect()

    # Reset pose
    reset_pose = np.array(args.reset_pose, dtype=np.float32)
    start_time = time.perf_counter()
    reset_follower_position(follower, reset_pose)
    busy_wait(5.0 - (time.perf_counter() - start_time))

    # Init visualization
    init_rerun(session_name="so101_so101_EE_teleop")

    # Main teleop loop
    print("Starting teleop loop...")
    info = {}
    complementary_data = {}
    transition = create_transition(observation=follower.get_observation(), info=info, complementary_data=complementary_data)

    while True:
        t0 = time.perf_counter()
        robot_obs = follower.get_observation()
        raw_action = gamepad.get_action()

        transition[TransitionKey.OBSERVATION] = robot_obs
        transition[TransitionKey.ACTION] = raw_action

        follower_joints_act = ee_to_follower_joints(transition)
        follower.send_action(follower_joints_act)

        log_rerun_data(observation=robot_obs, action=follower_joints_act)
        busy_wait(max(1.0 / args.fps - (time.perf_counter() - t0), 0.0))


# -----------------------------
# Argument parser
# -----------------------------
def parse_args():
    parser = argparse.ArgumentParser(description="SO101 follower teleoperation with gamepad control.")
    parser.add_argument("--fps", type=int, default=10, help="Target loop frequency (Hz).")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="Serial port for follower arm.")
    parser.add_argument("--front_camera_port", type=str, default="/dev/video0", help="Serial port for  for front camera.")
    parser.add_argument("--wrist_camera_port", type=str, default="/dev/video2", help="Serial port for  for wrist camera.")
    parser.add_argument("--arm_id", type=str, default="R12253612", help="Id for follower arm.")
    parser.add_argument("--reset-pose", type=float, nargs=6, default=[0.00, 0.00, 0.00, 90.00, 90.00, 10.00], help="Initial joint angles [6 values].")
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable detailed logging of each robot action in the pipeline.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    main(args)