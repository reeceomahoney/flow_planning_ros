#!/usr/bin/env python3

from copy import deepcopy

import rospy
import torch
from franka_interface import ArmInterface


class FlowMatchingController:
    def __init__(self):
        rospy.init_node("flow_matching_controller")

        self.model = self._load_model()
        self.arm = ArmInterface()
        self.rate = rospy.Rate(30)

        self.arm.move_to_neutral()
        self.initial_pose = deepcopy(self.arm.joint_ordered_angles())

        rospy.loginfo("Ready to start control. Press Enter to begin.")
        input("Hit Enter to Start")

        self.control_loop()

    def _load_model(self):
        model_path = rospy.get_param("~model_path", "model.pt")
        try:
            model = torch.jit.load(model_path)
            model.eval()
            rospy.loginfo(f"Successfully loaded PyTorch model from {model_path}")
            return model
        except Exception as e:
            rospy.logerr(f"Failed to load PyTorch model: {e}")
            return None

    def get_observation(self):
        # get current state
        joint_positions = self.arm.joint_ordered_angles()
        joint_velocities = list(self.arm.joint_velocities().values())
        ee_pose = self.arm.endpoint_pose()
        # build observation
        obs = joint_positions + joint_velocities
        obs += ee_pose["position"].tolist()
        obs += ee_pose["ori_mat"][:, :2].flatten().tolist()
        return torch.Tensor(obs).unsqueeze(0)

    def control_loop(self):
        rospy.loginfo("Starting control loop")

        while not rospy.is_shutdown():
            try:
                obs = self.get_observation()
                with torch.no_grad():
                    joint_positions = self.model(obs).squeeze().numpy()

                self.arm.set_joint_positions(
                    dict(zip(self.arm.joint_names(), joint_positions))
                )
                self.rate.sleep()

            except Exception as e:
                rospy.logerr(f"Error in control loop: {e}")
                break


if __name__ == "__main__":
    try:
        controller = FlowMatchingController()
    except rospy.ROSInterruptException:
        pass
