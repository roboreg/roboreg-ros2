import pathlib
from abc import ABC
from collections import OrderedDict
from copy import deepcopy
from typing import List

from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from std_srvs.srv import Trigger

from ros2_roboreg_idl.srv import CollectData, Export

from .collectables import Collectable


class Server(ABC):
    def __init__(self, node: Node) -> None:
        self._node = node

        # data collection
        self._collectables: OrderedDict[str, Collectable] = {}
        self._collectables_history: List[OrderedDict[str, Collectable]] = []

        # subscribers
        self._subscribers: OrderedDict[str, Subscriber] = {}

        # synchronizer
        self._approx_time_sync: ApproximateTimeSynchronizer = None

        # services
        self._collect_data_srv = self._node.create_service(
            CollectData, "~/collect_data", self._on_collect_data
        )
        self._clear_data_srv = self._node.create_service(
            Trigger, "~/clear_data", self._on_clear_data
        )
        self._save_data_srv = self._node.create_service(
            Export, "~/export/data", self._on_save_data
        )

    @property
    def node(self) -> Node:
        return self._node

    @property
    def subscribers(self) -> OrderedDict[str, Subscriber]:
        return self._subscribers

    @subscribers.setter
    def subscribers(self, subscribers: OrderedDict[str, Subscriber]) -> None:
        self._subscribers = subscribers

    def initialize(self, accuracy: float = 0.1) -> None:
        self._approx_time_sync: ApproximateTimeSynchronizer = (
            ApproximateTimeSynchronizer(
                self._subscribers.values(), queue_size=10, slop=accuracy
            )
        )
        self._approx_time_sync.registerCallback(self._on_sync)

    def _on_sync(self, *msgs: List) -> None:
        self._collectables = OrderedDict(
            {
                name: Collectable.from_message(msg)
                for name, msg in zip(self._subscribers.keys(), msgs)
            }
        )

    def _on_collect_data(
        self, _: CollectData.Request, res: CollectData.Response
    ) -> CollectData.Response:
        try:
            if self._collectables is None:
                warning = "No new data available."
                self._node.get_logger().warn(warning)
                res.success = False
                res.n_collected = len(self._collectables_history)
                res.message = warning
                return res
            self._collectables_history.append(deepcopy(self._collectables))
            self._collectables = None
            res.success = True
            res.n_collected = len(self._collectables_history)
            res.message = f"Collected {res.n_collected} data points."
            self._node.get_logger().info(res.message)
        except Exception as e:
            res.success = False
            res.n_collected = len(self._collectables_history)
            res.message = str(e)
            self._node.get_logger().error(res.message)
        return res

    def _on_clear_data(
        self, _: Trigger.Request, res: Trigger.Response
    ) -> Trigger.Response:
        self._collectables = None
        self._collectables_history.clear()
        res.success = True
        res.message = "Cleared all collected data."
        self._node.get_logger().info(res.message)
        return res

    def _on_save_data(
        self, req: Export.Request, res: Export.Response
    ) -> Export.Response:
        try:
            for idx, collectables in enumerate(self._collectables_history):
                for name, collectable in collectables.items():
                    collectable.to_disk(
                        path=pathlib.Path(req.path),
                        filename=f"{name}_{idx}",
                        mkdir=req.mkdir,
                    )
            res.success = True
        except Exception as e:
            res.success = False
            res.message = str(e)
            self._node.get_logger().error(res.message)
        return res
