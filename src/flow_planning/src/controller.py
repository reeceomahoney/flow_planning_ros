#!/usr/bin/env python3

import random
from copy import deepcopy

import rospy
import torch
from scipy.signal import savgol_filter

from flow_planning.arm import CustomArmInterface


class FlowMatchingController:
    def __init__(self):
        rospy.init_node("flow_matching_controller")

        self.model = self._load_model()
        self.arm = CustomArmInterface()

        self.arm.move_to_neutral()
        self.initial_pose = deepcopy(self.arm.joint_ordered_angles())
        self.default_joint_pos = [ 0.0000, -0.5690,  0.0000, -2.8100,  0.0000,  3.0370,  0.7410]  # fmt: off
        self.goal = torch.tensor([0.5, 0.3, 0.2, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0]).to(
            "cuda"
        )

        rospy.loginfo("Ready to start control. Press Enter to begin.")
        input("Hit Enter to Start")

        self.control_loop()

    def _load_model(self):
        model_path = rospy.get_param("~model_path", "model.pt")
        model = torch.jit.load(model_path)
        model.eval()
        rospy.loginfo(f"Successfully loaded PyTorch model from {model_path}")
        return model

    def get_observation(self):
        # get current state
        joint_positions = self.arm.joint_ordered_angles()
        joint_positions = [
            x - y for x, y in zip(joint_positions, self.default_joint_pos)
        ]
        joint_positions += [-0.0407, -0.0355]  # gripper positions
        joint_velocities = list(self.arm.joint_velocities().values())
        joint_velocities += [0.0, 0.0]
        ee_pose = self.arm.endpoint_pose()
        # build observation
        obs = joint_positions + joint_velocities
        obs += ee_pose["position"].tolist()
        obs += ee_pose["ori_mat"][:, :2].flatten().tolist()
        return torch.Tensor(obs).unsqueeze(0).to("cuda")

    def control_loop(self):
        rospy.loginfo("Starting control loop")

        while not rospy.is_shutdown():
            try:
                obs = self.get_observation()
                self.goal[0] = random.uniform(0.35, 0.5)
                self.goal[1] = random.uniform(-0.5, 0.5)
                self.goal[2] = random.uniform(0.15, 0.5)
                with torch.no_grad():
                    print("Resampling goal...")
                    actions = self.model(obs, self.goal).squeeze().cpu()
                    actions[:, :7] += torch.tensor(self.default_joint_pos)
                    actions = actions.numpy()

                self.arm.follow_trajectory(actions[::10])

            except Exception as e:
                rospy.logerr(f"Error in control loop: {e}")
                break


if __name__ == "__main__":
    try:
        controller = FlowMatchingController()
    except rospy.ROSInterruptException:
        pass
