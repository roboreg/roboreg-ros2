from pathlib import Path

from ament_index_python import get_package_share_directory
from urdf_parser_py.urdf import URDF, Mesh


def resolve_package_uris(robot_description: str) -> tuple[str, dict[str, Path]]:
    robot = URDF.from_xml_string(robot_description)
    resolved: dict[str, Path] = {}
    for link in robot.links:
        for holder in (link.visual, link.collision):
            if holder is None or not isinstance(holder.geometry, Mesh):
                continue
            uri = holder.geometry.filename
            if not uri.startswith("package://"):
                continue
            pkg, _, rel = uri.removeprefix("package://").partition("/")
            path = Path(get_package_share_directory(pkg)) / rel
            holder.geometry.filename = rel
            resolved[rel] = path
    return robot.to_xml_string(), resolved
