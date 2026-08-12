import json

import requests


class RoboregCloudClient:
    def __init__(self, base_url: str, timeout: float = 120.0) -> None:
        self._base_url = base_url.rstrip("/")
        self._timeout = timeout
        self._session = requests.Session()

    def localize_hydra(self, archive: bytes, config: dict) -> bytes:
        response = self._session.post(
            f"{self._base_url}/localize/hydra",
            files={"archive": ("archive.zip", archive, "application/zip")},
            data={"config": json.dumps(config)},
            timeout=self._timeout,
        )
        response.raise_for_status()
        return response.content
