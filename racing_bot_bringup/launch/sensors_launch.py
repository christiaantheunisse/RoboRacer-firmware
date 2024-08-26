import os
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter, SetRemap
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch.actions import LogInfo, GroupAction
from launch.substitutions import (
    TextSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    NotSubstitution,
    AndSubstitution,
    OrSubstitution,
    EqualsSubstitution,
    PythonExpression,
)
from launch.substitution import Substitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def are_equal_substition(left, right) -> Substitution:
    """Use instead of launch.substitutions.EqualsSubstition, since ROS2 Humble doesn't have the aforementioned class.
    example:
        are_equal_substition(slam_mode, "localization")
    INSTEAD OF:
        EqualsSubstitution(slam_mode, "localization")
    """
    return PythonExpression(["'", left, "' == '", right, "'"])


def generate_launch_description():
    use_ekf_launch_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value=TextSubstitution(text="true"),
        description="Use the extended kalman filter for the odometry data."
        "This will also activate and use the imu if available.",
    )

    use_ekf = LaunchConfiguration("use_ekf")

    # To start the `pigpiod package`, necessary for I2C
    start_pigpiod = ExecuteProcess(
        cmd=["sudo", "pigpiod"],
        name="Start pigpiod",
    )

    # Sensors
    hat_node = Node(
        package="racing_bot_hat",
        executable="hat_node",
    )
    encoder_node = Node(
        package="racing_bot_encoder",
        executable="encoder_node",
        parameters=[
            PathJoinSubstitution([FindPackageShare("racing_bot_encoder"), "config", "encoder_node.yaml"]),
        ],
    )
    imu_node = Node(
        package="racing_bot_imu",
        executable="imu_node",
        parameters=[
            PathJoinSubstitution([FindPackageShare("racing_bot_imu"), "config", "imu_node.yaml"]),
        ],
    )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("sllidar_ros2"), "launch", "sllidar_a1_launch.py"])
        ),
    )

    # State estimation:
    #  SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("racing_bot_bringup"), "launch/partial_launches", "slam_launch.py"])
        ),
        launch_arguments={
            "publish_tf": NotSubstitution(use_ekf),
            "time_interval": "2.0",
        }.items(),
    )

    #  Odometry
    odometry_node_w_ekf = Node(
        package="racing_bot_odometry",
        executable="odometry_node",
        parameters=[
            PathJoinSubstitution([FindPackageShare("racing_bot_odometry"), "config", "odometry_node.yaml"]),
            {
                "do_broadcast_transform": NotSubstitution(use_ekf)
            },  # Use either this or ekf `base_link` to `odom` transform
        ],
        condition=IfCondition(use_ekf),
    )
    odometry_node_wo_ekf = Node(
        package="racing_bot_odometry",
        executable="odometry_node",
        parameters=[
            PathJoinSubstitution([FindPackageShare("racing_bot_odometry"), "config", "odometry_node.yaml"]),
            {"do_broadcast_transform": NotSubstitution(use_ekf)},  # Use either this or ekf transform (set in ekf.yaml)
        ],
        condition=UnlessCondition(use_ekf),
        remappings=[("odometry/wheel_encoders", "odometry/filtered")],
    )

    #  EKF
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        parameters=[PathJoinSubstitution([FindPackageShare("racing_bot_bringup"), "config", "ekf_one.yaml"])],
        condition=IfCondition(use_ekf),
    )

    # CLI: ros2 run tf2_ros static_transform_publisher --x 0. --frame-id map --child-frame-id test_traject
    static_trans_base_link_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=("--x 0.085 --yaw 3.14 --frame-id base_link --child-frame-id laser").split(" "),
    )
    static_trans_base_link_to_imu_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=("--x 0.025 --y -0.038 --yaw -1.57080 --frame-id base_link --child-frame-id imu_link").split(" "),
    )

    return LaunchDescription(
        [
            # arguments
            use_ekf_launch_arg,
            # commands
            start_pigpiod,
            # nodes
            hat_node,
            encoder_node,
            odometry_node_w_ekf,
            odometry_node_wo_ekf,
            imu_node,
            ekf_node,
            # launch files
            lidar_launch,
            slam_launch,
            # static transforms
            static_trans_base_link_to_laser,
            static_trans_base_link_to_imu_link,
        ]
    )
