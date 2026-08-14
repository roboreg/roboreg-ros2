import io

import numpy as np

from roboreg_base.stereo_depth_node import (
    StereoDepthNode as StereoDepthNodeBase,
)
from roboreg_idl.srv import RegHydraRobustICP

from .util.annotator import OpenCVAnnotator, annotations_to_csv
from .util.archive import build_archive
from .util.roboreg_client import RoboregCloudClient
from .util.urdf import format_robot_description


class StereoDepthNode(StereoDepthNodeBase):
    def __init__(self, node_name: str = "roboreg_stereo_cloud") -> None:
        super().__init__(node_name)

        self._hydra_icp_srv = self.create_service(
            RegHydraRobustICP, "register/hydra_robust_icp", self._on_hydra_icp
        )
        self._interactive_annotator = OpenCVAnnotator(self)
        self._roboreg_client = RoboregCloudClient(self)

    def _on_hydra_icp(
        self, req: RegHydraRobustICP.Request, res: RegHydraRobustICP.Response
    ) -> RegHydraRobustICP.Response:
        res.success = True
        try:
            images = [
                collectables["camera.left.image"].to_numpy()
                for collectables in self._data_collector.collectables_history
            ]
            annotations = []
            for image in images:
                annotations.append(
                    annotations_to_csv(self._interactive_annotator.annotate(image))
                )
                self._interactive_annotator.clear()
            formatted_urdf, mesh_path_mapping = format_robot_description(
                self._robot_description
            )
            archive = build_archive(
                urdf=formatted_urdf,
                mesh_paths_mapping=mesh_path_mapping,
                intrinsics=self._data_collector.collectables_history[0][
                    "camera.depth.camera_info"
                ].to_numpy(),
                joint_states=[
                    collectables["joint_states"].to_numpy()
                    for collectables in self._data_collector.collectables_history
                ],
                depths=[
                    collectables["camera.depth"].to_numpy()
                    for collectables in self._data_collector.collectables_history
                ],
                images=images,
                image_annotations=annotations,
            )
            response = self._roboreg_client.run_hydra_robust_icp(
                archive=archive,
                config={
                    "root_link_name": self._robot_data_params.root_link_name,
                    "end_link_name": self._robot_data_params.end_link_name,
                    "collision_meshes": self._robot_data_params.collision_meshes,
                    "dilation_kernel_size": req.dilation_kernel_size,
                    "erosion_kernel_size": req.erosion_kernel_size,
                    "max_correspondence_distance": req.max_correspondence_distance,
                    "max_inner_iterations": req.max_inner_iterations,
                    "max_outer_iterations": req.max_outer_iterations,
                    "reference_points_per_mesh": req.reference_points_per_mesh,
                    "rmse_change_tolerance": req.rmse_change_tolerance,
                    "use_mask_boundary": req.use_mask_boundary,
                    "z_max": req.z_max,
                    "z_min": req.z_min,
                },
            )
            extrinsics = np.loadtxt(io.BytesIO(response.content), delimiter=",")
            if np.isnan(extrinsics).any():
                raise ValueError("Registration failed: extrinsics contain NaN values.")
            res.message = f"Registration completed successfully with status code '{response.status_code}'."
            self._extrinsics = extrinsics
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().error(res.message)
            return res
        return res
