import numpy as np
import torch
from roboreg import differentiable as rrd
from roboreg.util import overlay_mask

from .kinematics import KinematicsPlugin


class RenderPlugin:
    @staticmethod
    def render_meshes(
        meshes: rrd.TorchMeshContainer,
        kinematics: rrd.TorchKinematics,
        camera: rrd.VirtualCamera,
        renderer: rrd.NVDiffRastRenderer,
        joint_states: torch.FloatTensor,
    ) -> torch.Tensor:
        # apply forward kinematics
        mesh_vertices = KinematicsPlugin.mesh_forward_kinematics(
            kinematics=kinematics, meshes=meshes, joint_states=joint_states
        )

        # compute observations
        observed_vertices = torch.matmul(
            mesh_vertices,
            torch.matmul(
                torch.linalg.inv(
                    torch.matmul(
                        camera.extrinsics,
                        camera.ht_optical,
                    ),
                ).transpose(-1, -2),
                camera.perspective_projection.transpose(-1, -2),
            ),
        )

        # render
        return renderer.constant_color(
            observed_vertices,
            meshes.faces,
            camera.resolution,
        )

    @staticmethod
    def overlay_render(
        image: np.ndarray,
        render: np.ndarray,
        color: str = "b",
    ) -> np.ndarray:
        return overlay_mask(img=image, mask=render, mode=color, scale=1.0)
