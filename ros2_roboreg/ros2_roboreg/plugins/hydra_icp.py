from dataclasses import dataclass
from typing import List

import numpy as np
import torch
from roboreg.differentiable import TorchKinematics, TorchMeshContainer
from roboreg.hydra_icp import hydra_centroid_alignment, hydra_robust_icp
from roboreg.util.mask import mask_extract_extended_boundary
from roboreg.util.points import (
    clean_xyz,
    compute_vertex_normals,
    from_homogeneous,
    to_homogeneous,
)
from roboreg.util.transform import depth_to_xyz, generate_ht_optical


class HydraICP:
    @dataclass
    class _ProcessParams:
        with_boundary: bool = True
        dilation_kernel_size: int = 3
        erosion_kernel_size: int = 10

    @dataclass
    class _RegistrationParams:
        number_of_points: int = 5000
        max_distance: float = 0.1
        outer_max_iter: int = 100
        inner_max_iter: int = 3
        rmse_change: float = 1.0e-6

    @staticmethod
    def _depths_to_pcls(
        depths: List[np.ndarray],
        intrinsics: np.ndarray,
        z_min: float = 0.01,
        z_max: float = 4.0,
        device: torch.device = "cuda",
    ) -> List[np.ndarray]:
        intrinsics = torch.tensor(intrinsics, dtype=torch.float32, device=device)
        depths = torch.tensor(np.array(depths), dtype=torch.float32, device=device)
        xyzs = depth_to_xyz(
            depth=depths,
            intrinsics=intrinsics,
            z_min=z_min,
            z_max=z_max,
            conversion_factor=1.0,  # assume depth is in meters
        )

        # flatten BxHxWx3 -> Bx(H*W)x3
        height, width = xyzs.shape[-3:-1]
        xyzs = xyzs.view(-1, height * width, 3)
        xyzs = to_homogeneous(xyzs)
        ht_optical = generate_ht_optical(
            xyzs.shape[0], dtype=torch.float32, device=device
        )
        xyzs = torch.matmul(xyzs, ht_optical.transpose(-1, -2))
        xyzs = from_homogeneous(xyzs)

        # unflatten
        xyzs = xyzs.view(-1, height, width, 3)
        xyzs = xyzs.cpu().numpy()

        # return as list of numpy arrays
        return [xyz for xyz in xyzs]

    @staticmethod
    def _process_pcls(
        pcls: List[np.ndarray],
        params: _ProcessParams,
        masks: List[np.ndarray] = None,
        device: torch.device = "cuda",
    ) -> List[torch.Tensor]:
        processed_pcls = [
            torch.tensor(
                clean_xyz(
                    xyz=pcl,
                    mask=(
                        mask_extract_extended_boundary(
                            mask,
                            dilation_kernel=np.ones(
                                [params.dilation_kernel_size, params.dilation_kernel_size]
                            ),
                            erosion_kernel=np.ones(
                                [params.erosion_kernel_size, params.erosion_kernel_size]
                            ),
                        )
                        if params.with_boundary
                        else mask
                    ),
                ),
                dtype=torch.float32,
                device=device,
            )
            for pcl, mask in zip(pcls, masks)
        ]
        return processed_pcls

    @staticmethod
    def _register_hydra_icp(
        meshes: TorchMeshContainer,
        kinematics: TorchKinematics,
        joint_states: List[np.ndarray],
        pcls: List[torch.Tensor],
        params: _RegistrationParams,
    ) -> np.ndarray:
        batch_size = len(joint_states)
        if batch_size != meshes.batch_size:
            raise ValueError(
                "Batch size of joint states and mesh vertices must be the same."
            )

        # process data
        mesh_vertices = meshes.vertices.clone()
        joint_states = torch.tensor(
            np.array(joint_states), dtype=torch.float32, device=meshes.device
        )
        ht_lookup = kinematics.mesh_forward_kinematics(joint_states)
        for link_name, ht in ht_lookup.items():
            mesh_vertices[
                :,
                meshes.lower_vertex_index_lookup[
                    link_name
                ] : meshes.upper_vertex_index_lookup[link_name],
            ] = torch.matmul(
                mesh_vertices[
                    :,
                    meshes.lower_vertex_index_lookup[
                        link_name
                    ] : meshes.upper_vertex_index_lookup[link_name],
                ],
                ht.transpose(-1, -2),
            )

        # mesh vertices to list
        mesh_vertices = from_homogeneous(mesh_vertices)
        mesh_vertices = [mesh_vertices[i].contiguous() for i in range(batch_size)]
        mesh_normals = []
        for i in range(batch_size):
            mesh_normals.append(
                compute_vertex_normals(vertices=mesh_vertices[i], faces=meshes.faces)
            )

        # sample N points per mesh
        for i in range(batch_size):
            idx = torch.randperm(mesh_vertices[i].shape[0])[: params.number_of_points]
            mesh_vertices[i] = mesh_vertices[i][idx]
            mesh_normals[i] = mesh_normals[i][idx]

        ht_init = hydra_centroid_alignment(pcls, mesh_vertices)
        ht = hydra_robust_icp(
            ht_init,
            pcls,
            mesh_vertices,
            mesh_normals,
            max_distance=params.max_distance,
            outer_max_iter=params.outer_max_iter,
            inner_max_iter=params.inner_max_iter,
        )

        return ht.squeeze().cpu().numpy()
