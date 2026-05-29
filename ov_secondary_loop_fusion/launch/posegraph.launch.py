from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ov_secondary_loop_fusion",
                executable="loop_fusion_node",
                name="loop_fusion_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "config_file": PathJoinSubstitution(
                            [FindPackageShare("ov_secondary_loop_fusion"), "config", "master_config.yaml"]
                        ),
                        "vocabulary_file": PathJoinSubstitution(
                            [FindPackageShare("ov_secondary_loop_fusion"), "data", "brief_k10L6.bin"]
                        ),
                        "brief_pattern_file": PathJoinSubstitution(
                            [FindPackageShare("ov_secondary_loop_fusion"), "data", "brief_pattern.yml"]
                        ),
                    }
                ],
            )
        ]
    )
