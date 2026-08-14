import struct

import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage


def decode_depth(msg: CompressedImage) -> np.ndarray[np.float32]:
    r"""Decodes a compressed depth image message.

    Args:
        msg [CompressedImage]: Compressed depth image message.

    Returns:
        np.ndarray[np.float32]: Depth image in meters.
    """
    depth_fmt, compr_type = msg.format.split(";")
    # Remove whitespace
    depth_fmt = depth_fmt.strip()
    compr_type = compr_type.strip()

    # Check for correct compression type
    if compr_type != "compressedDepth":
        raise Exception(
            "Compression type is not 'compressedDepth'."
            "You probably subscribed to the wrong topic."
        )

    # Remove header from raw data
    depth_header_size = 12
    raw_data = msg.data[depth_header_size:]

    # Decode the image with OpenCV
    depth_img_raw = cv2.imdecode(
        np.frombuffer(raw_data, np.uint8), cv2.IMREAD_UNCHANGED
    )
    if depth_img_raw is None:
        raise Exception(
            "Could not decode compressed depth image."
            "You may need to change 'depth_header_size'!"
        )

    if depth_fmt == "16UC1":
        # Convert millimeters to meters as float32
        depth_img_meters = depth_img_raw.astype(np.float32) / 1000.0
        depth_img_meters[depth_img_raw == 0] = 0
    elif depth_fmt == "32FC1":
        # Parse quantization parameters from header
        raw_header = msg.data[:depth_header_size]
        [compfmt, depthQuantA, depthQuantB] = struct.unpack("iff", raw_header)

        # Scale the depth image according to quantization values
        depth_img_meters = depthQuantA / (
            depth_img_raw.astype(np.float32) - depthQuantB
        )

        # Set invalid values (raw depth == 0) to 0 in the final output
        depth_img_meters[depth_img_raw == 0] = 0
    else:
        raise Exception(f"Unsupported depth format {depth_fmt}.")

    return depth_img_meters.astype(np.float32)
