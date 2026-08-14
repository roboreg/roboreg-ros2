import json
from dataclasses import dataclass

import requests
from rclpy.node import Node


class RoboregCloudClient:
    @dataclass(frozen=True)
    class _Params:
        base_url: str
        timeout: float

    def __init__(
        self,
        node: Node,
    ) -> None:
        self._node = node
        self._declare_parameters()
        self._params = self._get_parameters()
        self._session = requests.Session()

    def run_hydra_robust_icp(self, archive: bytes, config: dict) -> bytes:
        response = self._session.post(
            f"{self._params.base_url}/localize/hydra",
            files={"archive": ("archive.zip", archive, "application/zip")},
            data={"config": json.dumps(config)},
            timeout=self._params.timeout,
        )
        response.raise_for_status()
        return response.content

    def _declare_parameters(self) -> None:
        self._node.declare_parameter("cloud_client.base_url", "http://localhost:8000")
        self._node.declare_parameter("cloud_client.timeout", 120.0)

    def _get_parameters(self) -> _Params:
        return self._Params(
            base_url=self._node.get_parameter("cloud_client.base_url")
            .get_parameter_value()
            .string_value.rstrip("/"),
            timeout=self._node.get_parameter("cloud_client.timeout")
            .get_parameter_value()
            .double_value,
        )
