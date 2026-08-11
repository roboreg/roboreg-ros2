from typing import List

import numpy as np
from roboreg.core.robot import RobotData
from roboreg.registration.point_cloud.config import (
    DepthToPointCloudConfig,
    HydraConfig,
    HydraRobustICPConfig,
)
from roboreg.registration.point_cloud.request import HydraObservations, HydraRequest
from roboreg.registration.point_cloud.solver import HydraRobustICP
from roboreg.registration.result import RegistrationResult

from roboreg_idl.srv import RegHydraRobustICP


def run_hydra_robust_icp_registration(
    request: RegHydraRobustICP.Request,
    robot_data: RobotData,
    intrinsics: np.ndarray,
    joint_states: List[np.ndarray],
    masks: List[np.ndarray],
    depths: List[np.ndarray],
) -> RegistrationResult:
    registration = HydraRobustICP(
        config=HydraRobustICPConfig(
            hydra=HydraConfig(
                reference_points_per_mesh=request.reference_points_per_mesh,
                depth_to_point_cloud=DepthToPointCloudConfig(
                    z_min=request.z_min,
                    z_max=request.z_max,
                    use_mask_boundary=request.use_mask_boundary,
                    dilation_kernel_size=request.dilation_kernel_size,
                    erosion_kernel_size=request.erosion_kernel_size,
                ),
                max_correspondence_distance=request.max_correspondence_distance,
                rmse_change_tolerance=request.rmse_change_tolerance,
            ),
            max_outer_iterations=request.max_outer_iterations,
            max_inner_iterations=request.max_inner_iterations,
        ),
    )
    return registration(
        request=HydraRequest(
            intrinsics=intrinsics,
            robot_data=robot_data,
            observations=HydraObservations(
                joint_states=joint_states,
                masks=masks,
                depths=depths,
            ),
        )
    )
