from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Arguments ---
    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("urdf_to_screw_list"),
            "config",
            "tm5-700.urdf.xacro"
        ]),
        description="Path to the TM5-700 URDF/Xacro file."
    )
    base_link_arg   = DeclareLaunchArgument("base_link", default_value="base_link",
                                            description="Base link name (space frame).")
    ee_link_arg     = DeclareLaunchArgument("ee_link", default_value="tool0",
                                            description="End-effector/TCP link name.")
    use_body_frame_arg    = DeclareLaunchArgument("use_body_frame", default_value="false",
                                            description="true → body screws B, false → space screws S.")
    home_pose_as_pos_quat_arg = DeclareLaunchArgument("home_pose_as_pos_quat", default_value="true",
                                            description="true → use position + quaternion (wxyz), false → use matrix.")
    output_path_arg = DeclareLaunchArgument("output_path", default_value="",
                                            description="File path to save the screw list (make sure directory exists).")
    output_fmt_arg  = DeclareLaunchArgument("output_format", default_value="yaml",
                                            description="Output format: currently only supports yaml.")

    # --- LaunchConfigurations ---
    urdf_path      = LaunchConfiguration("urdf_path")
    base_link      = LaunchConfiguration("base_link")
    ee_link        = LaunchConfiguration("ee_link")
    use_body_frame = LaunchConfiguration("use_body_frame")
    home_pose_as_pos_quat = LaunchConfiguration("home_pose_as_pos_quat")
    output_path    = LaunchConfiguration("output_path")
    output_format  = LaunchConfiguration("output_format")

    # --- Build robot_description by running xacro in-shell ---
    robot_description = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ", urdf_path]),
        value_type=str
    )

    # --- One-shot screw export node ---
    screw_node = Node(
        package="urdf_to_screw_list",
        executable="urdf_to_screw_list_node",
        name="tm5_700_screw_exporter",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "base_link": base_link,
            "ee_link": ee_link,
            "use_body_frame": ParameterValue(use_body_frame, value_type=bool),
            "home_pose_as_pos_quat": ParameterValue(home_pose_as_pos_quat, value_type=bool),
            "output_path": output_path,
            "output_format": output_format,
            "verbose": True,
        }]
    )

    return LaunchDescription([
        urdf_path_arg,
        base_link_arg,
        ee_link_arg,
        use_body_frame_arg,
        home_pose_as_pos_quat_arg,
        output_path_arg,
        output_fmt_arg,
        screw_node
    ])
