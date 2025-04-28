#!/usr/bin/env python3

import os

import numpy as np
import pytorch_kinematics as pk
import rospy
import torch
from franka_interface import ArmInterface
from scipy.signal import savgol_filter
from torch import Tensor

from flow_planning.gp import CostGPTrajectory


class FlowMatchingController:
    def __init__(self):
        rospy.init_node("flow_matching_controller")

        self.model = self._load_model()
        self.arm = ArmInterface()
        self.rate = rospy.Rate(30)

        self.state_dim = 14
        self.sampling_steps = 10
        self.T = 64
        self.use_refinement = False
        self.use_guide = True
        self.device = "cuda"

        urdf_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "../data/franka_panda/panda.urdf",
        )
        self.urdf_chain = pk.build_serial_chain_from_urdf(
            open(urdf_path, mode="rb").read(), "panda_hand"
        ).to(device=self.device)
        self.gp = CostGPTrajectory(self.T, 1 / 30, 1)
        self.timesteps = torch.linspace(0, 1.0, self.sampling_steps + 1).to(self.device)

        self.goal = torch.tensor(
            [0.1118, 0.2624, 0.4228, -2.0830, -0.1448, 2.3247, 1.4210]
        ).to(self.device)
        self.goal = torch.cat([self.goal, torch.zeros(7).to(self.device)]).unsqueeze(0)

        self.arm.move_to_cartesian_pose([0.5, -0.3, 0.3])

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
        joint_positions = self.arm.joint_ordered_angles()
        joint_velocities = list(self.arm.joint_velocities().values())
        obs = joint_positions + joint_velocities
        return torch.Tensor(obs).unsqueeze(0).to(self.device)

    def control_loop(self):
        rospy.loginfo("Starting control loop")

        while not rospy.is_shutdown():
            try:
                obs = self.get_observation()
                with torch.no_grad():
                    print("Planning...")
                    actions = self.forward(obs)

                # filter
                for i in range(actions.shape[1]):
                    actions[:, i] = savgol_filter(
                        actions[:, i], window_length=41, polyorder=2
                    )

                # moving average variables
                curr_pos = np.array(self.arm.joint_ordered_angles())
                prev_actions = actions[0].copy()
                alpha = 0.2

                # execute actions
                for i in range(actions.shape[0]):
                    while np.linalg.norm(actions[i, :7] - curr_pos) > 0.1:
                        act = alpha * actions[i] + (1 - alpha) * prev_actions
                        self.arm.set_joint_positions_velocities(
                            act[:7].tolist(), act[7:].tolist()
                        )
                        prev_actions = act.copy()
                        self.rate.sleep()
                        curr_pos = np.array(self.arm.joint_ordered_angles())

                rospy.sleep(0.5)

            except Exception as e:
                rospy.logerr(f"Error in control loop: {e}")
                break

    def forward(self, obs: Tensor) -> Tensor:
        # sample noise
        x = torch.randn((1, self.T, self.state_dim)).to(self.device)
        data = {"obs": obs, "goal": self.goal}

        # inference
        for i in range(self.sampling_steps):
            x = self.model(x, data, i)
            if self.use_guide:
                guide_vals = self._guide_fn(x)
                x += (1 - self.timesteps[i]) * guide_vals

        # refinement step
        if self.use_refinement:
            midpoint = x[:, self.T // 2].clone()
            x_0 = torch.randn((1, self.T, self.state_dim)).to(self.device)
            x = 0.5 * x_0 + 0.5 * x
            x = torch.cat([x[:, : self.T // 2], x[:, self.T // 2 :]], dim=0)

            data = {k: torch.cat([v] * 2) for k, v in data.items()}
            data["obs"][1] = midpoint
            data["goal"][0] = midpoint

            for i in range(self.sampling_steps // 2):
                x = self.model(x, data, i + self.sampling_steps // 2)
                if self.use_guide:
                    guide_vals = self._guide_fn(x)
                    x += (1 - self.timesteps[i]) * guide_vals

            x = torch.cat([x[0], x[1]], dim=0)

        x = self.model.denormalize(x)
        return x.squeeze().cpu().numpy()

    @torch.enable_grad()
    def _guide_fn(self, x: Tensor) -> Tensor:
        # collision
        x = x.detach().clone().requires_grad_(True)
        pts = torch.tensor([0.5, 0, 0.2]).view(1, 1, -1).to(self.device)
        th = self.urdf_chain.forward_kinematics(x[0, :, :7], end_only=False)
        matrices = {k: v.get_matrix() for k, v in th.items()}  # type: ignore
        pos = {k: v[:, :3, 3] for k, v in matrices.items()}
        pos = torch.stack(list(pos.values()), dim=1)
        dists = torch.norm(pos - pts, dim=-1)
        collision_grad = torch.autograd.grad([dists.sum()], [x])[0].detach()

        # smoothness
        x = x.detach().clone().requires_grad_(True)
        cost = self.gp(x)
        smooth_grad = torch.autograd.grad([cost.sum()], [x])[0].detach()

        grad = 0.1 * collision_grad - 1e-6 * smooth_grad
        return 1.2 * grad


if __name__ == "__main__":
    try:
        controller = FlowMatchingController()
    except rospy.ROSInterruptException:
        pass
