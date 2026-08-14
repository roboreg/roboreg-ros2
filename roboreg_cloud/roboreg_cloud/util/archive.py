import io
import zipfile
from pathlib import Path

import cv2
import numpy as np


def build_archive(
    *,
    urdf: str,
    mesh_paths_mapping: dict[str, Path],
    intrinsics: np.ndarray,
    joint_states: list[np.ndarray],
    depths: list[np.ndarray],
    images: list[np.ndarray],
    image_annotations: list[str],  # "x,y,label" rows, label 1=positive/0=negative
) -> bytes:
    if not len(joint_states) == len(depths) == len(images) == len(image_annotations):
        raise ValueError(
            f"Number of samples do not match. Got {len(joint_states)} joint state "
            f"samples, {len(depths)} depth samples, {len(images)} image "
            f"samples, and {len(image_annotations)} image sample labels."
        )

    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        zf.writestr("description/robot.urdf", urdf)
        for relative_path, absolute_path in mesh_paths_mapping.items():
            zf.write(absolute_path, arcname=f"description/{relative_path}")

        with zf.open("samples/intrinsics.csv", "w") as f:
            np.savetxt(f, intrinsics, delimiter=",")

        for i, (js, depth, image, samples) in enumerate(
            zip(joint_states, depths, images, image_annotations)
        ):
            with zf.open(f"samples/joint_states_{i}.npy", "w") as f:
                np.save(f, js)
            with zf.open(f"samples/depth_{i}.npy", "w") as f:
                np.save(f, depth)

            ok, encoded = cv2.imencode(".png", image)
            zf.writestr(f"samples/image_{i}.png", encoded.tobytes())
            zf.writestr(f"samples/image_{i}_annotations.csv", samples)
    return buf.getvalue()
