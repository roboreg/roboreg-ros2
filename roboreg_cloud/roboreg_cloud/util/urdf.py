from pathlib import Path

from ament_index_python import get_package_share_directory
from urdf_parser_py.urdf import URDF, Mesh


def format_robot_description(robot_description: str) -> tuple[str, dict[str, Path]]:
    r"""Format mesh URIs in URDF to relative paths.

    E.g., converts package://package_name/path/to/mesh.stl to path/to/mesh.stl, then
    returns a dictionary mapping the relative path to the absolute path of the mesh file,
    such that path/to/mesh.stl -> /absolute/path/to/package_name/path/to/mesh.stl.

    Args:
        robot_description (str): URDF string with mesh URIs to resolve.

    Returns:
        tuple[str, dict[str, Path]]: Tuple containing the URDF string with relative mesh paths
            and a dictionary mapping relative mesh paths to their absolute paths.
    """
    if robot_description == "":
        raise ValueError("Robot description is empty. Please provide a valid URDF.")
    formatted_urdf = URDF.from_xml_string(robot_description)
    mesh_path_mapping: dict[str, Path] = {}
    for link in formatted_urdf.links:
        for holder in (link.visual, link.collision):
            if holder is None or not isinstance(holder.geometry, Mesh):
                continue
            uri = holder.geometry.filename
            if not uri.startswith("package://"):
                continue
            pkg, _, rel = uri.removeprefix("package://").partition("/")
            path = Path(get_package_share_directory(pkg)) / rel
            holder.geometry.filename = rel
            mesh_path_mapping[rel] = path
    return formatted_urdf.to_xml_string(), mesh_path_mapping
