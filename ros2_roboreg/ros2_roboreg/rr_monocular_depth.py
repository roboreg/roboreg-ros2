import copy
import os
import pathlib
from typing import List

import cv2
import numpy as np
import torch
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, qos_profile_system_default
from roboreg.hydra_icp import hydra_centroid_alignment, hydra_robust_icp
from roboreg.util import (
    clean_xyz,
    compute_vertex_normals,
    from_homogeneous,
    mask_boundary,
)
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_srvs.srv import Trigger

from ros2_roboreg.structs import MonocularDepthParams, MonocularDepthSample
from ros2_roboreg_idl.srv import CollectSample, Export, RegHydraICP

from .rr_server import RoboregServer


class RoboregMonocularDepth(RoboregServer):
    def __init__(self, node_name: str = "roboreg_monocular_depth") -> None:
        super().__init__(node_name)

    def _instantiate_synced_samples(self):
        self._synced_sample: MonocularDepthSample = None
        self._synced_samples: List[MonocularDepthSample] = []

    def _instantiate_server_params(self) -> None:
        self._params: MonocularDepthParams = MonocularDepthParams()

    def _declare_camera_topic_node_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topics.image.name", "camera/image_rect_color"),
                ("topics.image.qos_reliability", "RELIABLE"),
                ("topics.images.camera_info.name", "camera/camera_info"),
                ("topics.images.camera_info.qos_reliability", "RELIABLE"),
            ],
        )

    def _get_camera_topic_node_parameters(self) -> None:
        self._params.image_topic.name = (
            self.get_parameter("topics.image.name").get_parameter_value().string_value
        )
        self._params.image_topic.qos_reliability = (
            self.get_parameter("topics.image.qos_reliability")
            .get_parameter_value()
            .string_value
        )
        self._params.camera_info_topic.name = (
            self.get_parameter("topics.images.camera_info.name")
            .get_parameter_value()
            .string_value
        )
        self._params.camera_info_topic.qos_reliability = (
            self.get_parameter("topics.images.camera_info.qos_reliability")
            .get_parameter_value()
            .string_value
        )

    def _log_camera_topic_node_parameters(self) -> None:
        self.get_logger().info(f"*{' '*7}Image topic:")
        self.get_logger().info(f"*{' '*9}Name: {self._params.image_topic.name}")
        self.get_logger().info(
            f"*{' '*9}QoS reliability: {self._params.image_topic.qos_reliability}"
        )
        self.get_logger().info(f"*{' '*7}Camera info topic:")
        self.get_logger().info(f"*{' '*9}Name: {self._params.camera_info_topic.name}")
        self.get_logger().info(
            f"*{' '*9}QoS reliability: {self._params.camera_info_topic.qos_reliability}"
        )

    def _on_set_camera_topic_parameter(
        self, parameter: Parameter, result: SetParametersResult
    ) -> None:
        if parameter.name == "topics.image.name":
            self.get_logger().info(f"Setting image topic to {parameter.value}")
            self._params.image_topic.name = parameter.value
            self._create_camera_subscriptions()
            self._create_approximate_time_sync()
        elif parameter.name == "topics.image.camera_info.name":
            self.get_logger().info(f"Setting camera info topic to {parameter.value}")
            self._params.camera_info_topic.name = parameter.value
            self._create_camera_subscriptions()
            self._create_approximate_time_sync()
        else:
            return

    def _create_registration_services(self) -> None:
        self._register_hydra_icp_srv = self.create_service(
            RegHydraICP,
            "register/hydra",
            self._on_register_hydra_icp,
        )

    def _create_camera_subscriptions(self) -> None:
        if self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._params.image_topic.qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._image_sub = Subscriber(
            self,
            Image,
            self._params.image_topic.name,
            qos_profile=qos_profile,
        )

        if self._camera_info_sub is not None:
            self.destroy_subscription(self._camera_info_sub)
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._params.camera_info_topic.qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._camera_info_sub = Subscriber(
            self,
            CameraInfo,
            self._params.camera_info_topic.name,
            qos_profile=qos_profile,
        )

    def _create_approximate_time_sync(self):
        if self._approximate_time_sync is not None:
            self._approximate_time_sync = None
        self._approximate_time_sync = ApproximateTimeSynchronizer(
            [
                self._camera_info_sub,
                self._image_sub,
                self._joint_state_sub,
                self._depth_sub,
            ],
            queue_size=1,
            slop=self._params.filters.sync_accuracy,
        )

    def _on_sync(
        self,
        camera_info: CameraInfo,
        image: Image,
        joint_state: JointState,
        depth: Image,
    ):
        self._synced_sample.camera.camera_info = camera_info
        self._synced_sample.camera.image = image
        self._synced_sample.joint_state = joint_state
        self._synced_sample.depth = depth

    def _on_collect_sample(
        self, request: CollectSample.Request, response: CollectSample.Response
    ) -> CollectSample.Response:
        try:
            if (
                self._synced_sample.camera.camera_info is None
                or self._synced_sample.camera.image is None
                or self._synced_sample.joint_state is None
                or self._synced_sample.depth is None
            ):
                response.success = False
                response.n_collected = len(self._synced_samples)
                response.message = f"No data available yet. Topics might be wrongly configured. Data might not be synchronized, accuracy: {self._params.filters.sync_accuracy} s."
                self.get_logger().warn(response.message)
                return response

            # check if joint states changed from last data
            if len(self._synced_samples) > 1:
                if np.isclose(
                    self._synced_samples[-1].joint_state.position,
                    self._synced_sample.joint_state.position,
                    atol=self._params.filters.min_joint_position_change,
                ).all():
                    response.success = False
                    response.n_collected = len(self._synced_samples)
                    response.message = f"Joint states did not change. Minimum joint position change: {self._params.filters.min_joint_position_change} rad. Skipping data collection."
                    self.get_logger().warn(response.message)
                    return response

            # only allow joint states velocities close to zero
            if not np.isclose(
                self._synced_sample.joint_state.velocity,
                np.zeros_like(self._synced_sample.joint_state.velocity),
                atol=self._params.filters.max_joint_velocity,
            ).all():
                response.success = False
                response.n_collected = len(self._synced_samples)
                response.message = f"Joint states velocity greater zero. Maximum joint velocity: {self._params.filters.max_joint_velocity} rad/s. This may cause un-correlated data. Skipping data collection."
                self.get_logger().warn(response.message)
                return response

            # add data
            self._synced_samples.append(copy.deepcopy(self._synced_sample))
            response.success = True
            response.n_collected = len(self._synced_samples)
            response.message = f"Added data with time stamp: {self._synced_sample.joint_state.header.stamp}"
            self.get_logger().info(response.message)
            self._synced_sample.clear()
        except Exception as e:
            response.success = False
            response.n_collected = len(self._synced_samples)
            response.message = f"Failed service call with: {e}"
            self.get_logger().error(response.message)
        return response

    def _hydra_icp_impl(self, request: RegHydraICP.Request) -> torch.Tensor:
        # process data
        mesh_vertices = self._meshes.vertices.clone()
        name_idx_map = np.argsort(np.array(self._synced_samples[0].joint_state.name))
        joint_states = [
            synced_sample.joint_state.position[name_idx_map]
            for synced_sample in self._synced_samples
        ]
        joint_states = torch.tensor(
            np.array(joint_states),
            dtype=torch.float32,
            device=self._params.robot_model.device,
        )
        ht_lookup = self._kinematics.mesh_forward_kinematics(joint_states)
        for link_name, ht in ht_lookup.items():
            mesh_vertices[
                :,
                self._meshes.lower_vertex_index_lookup[
                    link_name
                ] : self._meshes.upper_vertex_index_lookup[link_name],
            ] = torch.matmul(
                mesh_vertices[
                    :,
                    self._meshes.lower_vertex_index_lookup[
                        link_name
                    ] : self._meshes.upper_vertex_index_lookup[link_name],
                ],
                ht.transpose(-1, -2),
            )

        # mesh vertices to list
        if self._meshes.batch_size != len(self._synced_samples):
            raise ValueError("Batch size mismatch.")
        batch_size = self._meshes.batch_size
        mesh_vertices = from_homogeneous(mesh_vertices)
        mesh_vertices = [mesh_vertices[i].contiguous() for i in range(batch_size)]
        mesh_normals = []
        for i in range(batch_size):
            mesh_normals.append(
                compute_vertex_normals(
                    vertices=mesh_vertices[i], faces=self._meshes.faces
                )
            )

        # clean observed vertices and turn into tensor
        observed_vertices = [
            torch.tensor(
                clean_xyz(
                    xyz=xyz,
                    mask=(
                        mask_boundary(
                            mask,
                            erosion_kernel=np.ones(
                                [
                                    request.erosion_kernel_size,
                                    request.erosion_kernel_size,
                                ]
                            ),
                        )
                        if request.with_erosion
                        else mask
                    ),
                ),
                dtype=torch.float32,
                device=self._params.robot_model.device,
            )
            for xyz, mask in zip(self._pcls, self._left_segmentations)
        ]

        # sample N points per mesh
        for i in range(batch_size):
            idx = torch.randperm(mesh_vertices[i].shape[0])[: request.number_of_points]
            mesh_vertices[i] = mesh_vertices[i][idx]
            mesh_normals[i] = mesh_normals[i][idx]

        HT_init = hydra_centroid_alignment(observed_vertices, mesh_vertices)
        HT = hydra_robust_icp(
            HT_init,
            observed_vertices,
            mesh_vertices,
            mesh_normals,
            max_distance=request.max_distance,
            outer_max_iter=request.outer_max_iter,
            inner_max_iter=request.inner_max_iter,
        )
        return HT

    def _on_register_hydra_icp(
        self, request: RegHydraICP.Request, response: RegHydraICP.Response
    ) -> Trigger.Response:
        if len(self._synced_samples) == 0:
            response.success = False
            response.message = "No data collected yet"
            return response
        self._instantiate_robot_model(batch_size=len(self._synced_samples))
        if not self._kinematics:
            response.success = False
            response.message = "No kinematics available"
            return response
        if not self._meshes:
            response.success = False
            response.message = "No meshes available"
            return response
        try:
            # generate segmentation masks
            self._detect_and_segment()
            if len(self._left_segmentations) != len(self._synced_samples):
                raise ValueError(
                    "Segmentation masks not generated for all left images."
                )
            if len(self._right_segmentations) != len(self._synced_samples):
                raise ValueError(
                    "Segmentation masks not generated for all right images."
                )
            self._obtain_pcl_from_depth()
            if len(self._pcls) != len(self._synced_samples):
                raise ValueError("Point clouds not generated for all depth images.")
            if len(self._pcls) != len(self._left_segmentations):
                raise ValueError("Point clouds and segmentations do not match.")
            self._hydra_icp_impl(request)
            response.success = True
            response.message = "Registration successful"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed service call with: {e}"
            self.get_logger().error(response.message)
        return response

    def _on_export_samples(
        self, request: Export.Request, response: Export.Response
    ) -> Export.Response:
        try:
            if len(self._synced_samples) == 0:
                response.success = False
                response.message = "No data collected yet"
                return response

            if len(self._pcls) != len(self._synced_samples):
                self._obtain_pcl_from_depth()  # generate point clouds in case
            if len(self._pcls) != len(self._left_segmentations):
                raise ValueError("Point clouds and segmentations do not match.")

            path = pathlib.Path(request.path)
            self.get_logger().info(f"Saving data to {path.absolute()}")

            def write_synced_data():
                def write_camera_info_to_yaml(camera_info_msg: CameraInfo, path: str):
                    import yaml

                    camera_info_dict = {
                        "frame_id": camera_info_msg.header.frame_id,
                        "height": camera_info_msg.height,
                        "width": camera_info_msg.width,
                        "distortion_model": camera_info_msg.distortion_model,
                        "d": camera_info_msg.d.tolist(),
                        "k": camera_info_msg.k.tolist(),
                        "r": camera_info_msg.r.tolist(),
                        "p": camera_info_msg.p.tolist(),
                        "binning_x": camera_info_msg.binning_x,
                        "binning_y": camera_info_msg.binning_y,
                        "roi": {
                            "x_offset": camera_info_msg.roi.x_offset,
                            "y_offset": camera_info_msg.roi.y_offset,
                            "height": camera_info_msg.roi.height,
                            "width": camera_info_msg.roi.width,
                            "do_rectify": camera_info_msg.roi.do_rectify,
                        },
                    }
                    with open(path, "w") as file:
                        yaml.dump(camera_info_dict, file)

                # save camera infos
                for camera_info in self._synced_samples[0]:
                    write_camera_info_to_yaml(
                        camera_info,
                        os.path.join(
                            path, f"camera_info_{camera_info.header.frame_id}.yaml"
                        ),
                    )

                # save segmentation labels
                self._detector.write(
                    path=os.path.join(path, "sam2_right_labels.csv"),
                    samples=self._detector.samples,
                    labels=self._detector.labels,
                )

                # log time stamps to csv
                with open(os.path.join(path, "time_stamps.csv"), "w") as f:
                    f.write("idx,sec,nanosec\n")

                    for idx, synced_data in enumerate(self._synced_samples):
                        # log time stamps
                        f.write(
                            f"{idx},{synced_data.joint_state.header.stamp.sec},{synced_data.joint_state.header.stamp.nanosec}\n"
                        )

                        # save images
                        for image in synced_data.images:
                            image_np = self._bridge.imgmsg_to_cv2(
                                image, desired_encoding="passthrough"
                            )
                            cv2.imwrite(
                                os.path.join(
                                    path,
                                    f"image_{image.header.frame_id}_{idx}.png",
                                ),
                                image_np,
                            )

                        # convert to numpy
                        name_idx_map = np.argsort(
                            np.array(synced_data.joint_state.name)
                        )
                        joint_position = synced_data.joint_state.position[name_idx_map]
                        joint_position_np = np.array(joint_position)
                        depth_np = self._bridge.imgmsg_to_cv2(
                            synced_data.depth, desired_encoding="passthrough"
                        )

                        # save
                        np.save(
                            os.path.join(path, f"joint_states_{idx}.npy"),
                            joint_position_np,
                        )
                        np.save(
                            os.path.join(path, f"depth_{idx}.npy"),
                            depth_np,
                        )
                        np.save(
                            os.path.join(path, f"xyz_{idx}.npy"),
                            self._pcls[idx],
                        )
                self._synced_samples.clear()

            if path.exists():
                try:
                    write_synced_data()
                except Exception as e:
                    response.success = False
                    response.message = f"Could not write data to {path.absolute()}"
                    self.get_logger().error(response.message)
                    self.get_logger().error(e)
                    return response
                response.success = True
                response.message = f"Wrote data to {path.absolute()}"
                return response

            if request.mkdir:
                path.mkdir(parents=True, exist_ok=True)
                try:
                    write_synced_data()
                except Exception as e:
                    response.success = False
                    response.message = f"Could not write data to {path.absolute()}"
                    self.get_logger().error(response.message)
                    self.get_logger().error(e)
                    return response
                response.success = True
                response.message = (
                    f"Created directory {path.absolute()} and wrote data to it"
                )
                return response

            response.success = False
            response.message = f"Path {path.absolute()} does not exist and was not created as per request"
        except Exception as e:
            response.success = False
            response.message = f"Failed service call with: {e}"
            self.get_logger().error(response.message)
        return response
