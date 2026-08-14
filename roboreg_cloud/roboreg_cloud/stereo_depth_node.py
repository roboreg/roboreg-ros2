from roboreg_base.monocular_depth_node import StereoDepthNode as StereoDepthNodeBase
from roboreg_idl.srv import RegHydraRobustICP

from .util.annotator import OpenCVAnnotator
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

    def _on_hydra_icp(
        self, req: RegHydraRobustICP.Request, res: RegHydraRobustICP.Response
    ) -> RegHydraRobustICP.Response:
        res.success = True
        try:
            raise NotImplementedError("Hydra ICP registration is not implemented yet.")
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().error(res.message)
            return res
        return res
