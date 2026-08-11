from rclpy.impl.rcutils_logger import RcutilsLogger
from roboreg.core.robot import RobotData
from roboreg.io import URDFParser, apply_mesh_origins, load_meshes, simplify_meshes

from roboreg_base.parameters import RobotDataParams


def robot_description_to_robot_data(
    robot_description: str, params: RobotDataParams, logger: RcutilsLogger
) -> RobotData:
    if robot_description == "":
        raise ValueError("Robot description is empty. Please provide a valid URDF.")
    urdf_parser = URDFParser(robot_description)
    if params.root_link_name == "":
        params.root_link_name = urdf_parser.link_names_with_meshes(
            collision=params.collision_meshes
        )[0]
        logger.info(
            f"No root link name specified. Using first link with mesh: {params.root_link_name}"
        )
    if params.end_link_name == "":
        params.end_link_name = urdf_parser.link_names_with_meshes(
            collision=params.collision_meshes
        )[-1]
        logger.info(
            f"No end link name specified. Using last link with mesh: {params.end_link_name}"
        )

    # parse data from URDF
    mesh_paths = urdf_parser.mesh_paths_from_ros_registry(
        root_link_name=params.root_link_name,
        end_link_name=params.end_link_name,
        collision=params.collision_meshes,
    )

    mesh_origins = urdf_parser.mesh_origins(
        root_link_name=params.root_link_name,
        end_link_name=params.end_link_name,
        collision=params.collision_meshes,
    )

    # load and preprocess meshes
    meshes = load_meshes(paths=mesh_paths)
    meshes = simplify_meshes(
        meshes=meshes,
        target_reduction=0.0,
    )
    meshes = apply_mesh_origins(meshes=meshes, origins=mesh_origins)

    # instantiate robot data
    return RobotData(
        meshes=meshes,
        urdf=urdf_parser.urdf,
        root_link_name=params.root_link_name,
        end_link_name=params.end_link_name,
    )
