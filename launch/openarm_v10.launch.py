import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def screw_extractor_spawner(context: LaunchContext,
                            arm_type, ee_type, bimanual,
                            base_link, ee_link, use_body_frame,
                            output_path, output_format):
    arm_type_str = context.perform_substitution(arm_type)
    ee_type_str = context.perform_substitution(ee_type)
    bimanual_str = context.perform_substitution(bimanual)

    # xacro → URDF
    xacro_path = os.path.join(
        get_package_share_directory("openarm_description"),
        "urdf", "robot", f"{arm_type_str}.urdf.xacro"
    )
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "ee_type": ee_type_str,
            "bimanual": bimanual_str,
        }
    ).toprettyxml(indent="  ")

    return [Node(
        package="urdf_to_screw_list",
        executable="urdf_to_screw_list_node",
        name="urdf_to_screw_list_node",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "base_link": context.perform_substitution(base_link),
            "ee_link": context.perform_substitution(ee_link),
            "use_body_frame": ParameterValue(use_body_frame, value_type=bool),
            "output_path": context.perform_substitution(output_path),
            "output_format": context.perform_substitution(output_format),
            "verbose": True
        }]
    )]


def generate_launch_description():
    arm_type_arg = DeclareLaunchArgument("arm_type", description="Type of arm (e.g., v10)")
    ee_type_arg = DeclareLaunchArgument("ee_type", default_value="openarm_hand")
    bimanual_arg = DeclareLaunchArgument("bimanual", default_value="false")

    base_link_arg   = DeclareLaunchArgument("base_link", default_value="openarm_link0")
    ee_link_arg     = DeclareLaunchArgument("ee_link", default_value="openarm_hand_tcp")
    use_body_arg    = DeclareLaunchArgument("use_body_frame", default_value="false")
    output_path_arg = DeclareLaunchArgument("output_path", default_value="")
    output_fmt_arg  = DeclareLaunchArgument("output_format", default_value="yaml")  # yaml|txt|cpp

    arm_type      = LaunchConfiguration("arm_type")
    ee_type       = LaunchConfiguration("ee_type")
    bimanual      = LaunchConfiguration("bimanual")
    base_link     = LaunchConfiguration("base_link")
    ee_link       = LaunchConfiguration("ee_link")
    use_body_frame= LaunchConfiguration("use_body_frame")
    output_path   = LaunchConfiguration("output_path")
    output_format = LaunchConfiguration("output_format")

    return LaunchDescription([
        arm_type_arg, ee_type_arg, bimanual_arg,
        base_link_arg, ee_link_arg, use_body_arg,
        output_path_arg, output_fmt_arg,
        OpaqueFunction(
            function=screw_extractor_spawner,
            args=[arm_type, ee_type, bimanual,
                  base_link, ee_link, use_body_frame,
                  output_path, output_format]
        ),
    ])
