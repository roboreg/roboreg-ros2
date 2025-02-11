import torch
from roboreg import differentiable as rrd


class KinematicsPlugin:
    @staticmethod
    def mesh_forward_kinematics(
        kinematics: rrd.TorchKinematics,
        meshes: rrd.TorchMeshContainer,
        joint_states: torch.FloatTensor,
    ) -> torch.FloatTensor:
        mesh_vertices = meshes.vertices.clone()
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
        return mesh_vertices
