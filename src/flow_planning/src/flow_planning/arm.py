import franka_dataflow
import numpy as np
import rospy
from franka_interface import ArmInterface
from franka_tools import JointTrajectoryActionClient


class CustomArmInterface(ArmInterface):
    def follow_trajectory(self, trajectory: np.ndarray, timeout=10.0):
        curr_controller = self._ctrl_manager.set_motion_controller(
            self._ctrl_manager.joint_trajectory_controller
        )

        traj_client = JointTrajectoryActionClient(joint_names=self.joint_names())
        traj_client.clear()

        traj_client.add_point(
            positions=[self._joint_angle[n] for n in self._joint_names], time=0.0001
        )

        for t in range(trajectory.shape[0]):
            traj_client.add_point(
                positions=trajectory[t, :7].tolist(),
                time=(t + 1) / (0.5 * 3),
                velocities=trajectory[t, 7:].tolist(),
            )

        fail_msg = "{}: {} limb failed to reach commanded joint positions.".format(
            self.__class__.__name__, self.name.capitalize()
        )

        def test_collision():
            if self.has_collided():
                rospy.logerr(" ".join(["Collision detected.", fail_msg]))
                return True
            return False

        traj_client.start()

        franka_dataflow.wait_for(
            test=lambda: test_collision() or traj_client.result() is not None,
            timeout=timeout,
            timeout_msg=fail_msg,
            rate=100,
            raise_on_error=False,
        )
        res = traj_client.result()
        if res is not None and res.error_code:
            rospy.loginfo("Trajectory Server Message: {}".format(res))

        rospy.sleep(0.5)

        self._ctrl_manager.set_motion_controller(curr_controller)

        rospy.loginfo(
            "{}: Trajectory controlling complete".format(self.__class__.__name__)
        )
