import numpy as np

from roboreg_base.monocular_depth_node import (
    MonocularDepthNode as MonocularDepthNodeBase,
)
from roboreg_idl.srv import RegHydraRobustICP

from .util.hydra_robust_icp import run_hydra_robust_icp_registration
from .util.interactive_segmentation import InteractiveSegmentation
from .util.robot_data import robot_description_to_robot_data


class MonocularDepthNode(MonocularDepthNodeBase):
    def __init__(self, node_name: str = "roboreg") -> None:
        super().__init__(node_name)

        self._hydra_icp_srv = self.create_service(
            RegHydraRobustICP, "register/hydra_robust_icp", self._on_hydra_icp
        )
        self._interactive_segmentation = InteractiveSegmentation(self)

    def _on_hydra_icp(
        self, req: RegHydraRobustICP.Request, res: RegHydraRobustICP.Response
    ) -> RegHydraRobustICP.Response:
        res.success = True
        try:
            robot_data = robot_description_to_robot_data(
                robot_description=self._robot_description,
                params=self._robot_data_params,
                logger=self.get_logger(),
            )
            images = [
                collectables["camera.image"].to_numpy()
                for collectables in self._data_collector.collectables_history
            ]
            segmentations = self._interactive_segmentation.segment(images)
            depths = [
                collectables["camera.depth"].to_numpy()
                for collectables in self._data_collector.collectables_history
            ]
            intrinsics = self._data_collector.collectables_history[0][
                "camera.depth.camera_info"
            ].to_numpy()
            joint_states = [
                collectables["joint_states"].to_numpy()
                for collectables in self._data_collector.collectables_history
            ]
            result = run_hydra_robust_icp_registration(
                request=req,
                robot_data=robot_data,
                intrinsics=intrinsics,
                joint_states=joint_states,
                masks=segmentations,
                depths=depths,
            )
            extrinsics = result.extrinsics.cpu().numpy()
            if np.isnan(extrinsics).any():
                raise ValueError("Registration failed: extrinsics contain NaN values.")
            self._extrinsics = extrinsics
            res.message = f"Optimization terminated after {result.iterations} iterations with status '{result.termination_reason}'."
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().error(res.message)
            return res
        return res
