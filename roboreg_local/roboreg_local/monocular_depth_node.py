import numpy as np
from roboreg.io import load_robot_data_from_ros_robot_description

from roboreg_base.monocular_depth_node import (
    MonocularDepthNode as MonocularDepthNodeBase,
)
from roboreg_idl.srv import RegHydraRobustICP

from .util.hydra_robust_icp import run_hydra_robust_icp
from .util.interactive_segmentation import InteractiveSegmentation


class MonocularDepthNode(MonocularDepthNodeBase):
    def __init__(self, node_name: str = "roboreg_monocular_local") -> None:
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
            robot_data = load_robot_data_from_ros_robot_description(
                urdf=self._robot_description,
                root_link_name=self._robot_data_params.root_link_name,
                end_link_name=self._robot_data_params.end_link_name,
                collision=self._robot_data_params.collision_meshes,
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
            result = run_hydra_robust_icp(
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
