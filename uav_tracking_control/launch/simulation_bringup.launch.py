#!/usr/bin/env python3
"""一键启动仿真环境：PX4 SITL、Gazebo、图像桥接、识别、跟踪、任务控制等所有节点."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    px4_dir_arg = DeclareLaunchArgument(
        "px4_dir",
        default_value=EnvironmentVariable(
            "PX4_AUTOPILOT_DIR",
            default_value=os.path.expanduser("~/PX4-Autopilot"),
        ),
        description="Path to a PX4-Autopilot checkout",
    )
    px4_dir = LaunchConfiguration("px4_dir")

    # ---------- 环境变量 ----------
    # 为 PX4 和小车生成设置 Gazebo 资源路径
    turtlebot3_description = get_package_share_directory(
        "turtlebot3_description")
    turtlebot3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    turtlebot3_paths = os.pathsep.join((
        turtlebot3_description,
        os.path.join(turtlebot3_gazebo, "models"),
    ))
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    new_gz_path = f"{turtlebot3_paths}:{existing_gz_path}" if existing_gz_path else turtlebot3_paths
    set_gz_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", new_gz_path)

    # ---------- 启动 PX4 SITL ----------
    px4_sitl = ExecuteProcess(
        cmd=["make", "px4_sitl", "gz_x500_mono_cam_down", "-d"],
        cwd=px4_dir,
        output="screen",
        emulate_tty=True,
    )

    # ---------- 启动 MicroXRCEAgent ----------
    microxrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen",
        emulate_tty=True,
    )

    # ---------- 生成 TurtleBot3 小车 ----------
    create_turtlebot = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_sim", "create",
            "-world", "default",
            "-file", os.path.join(
                turtlebot3_gazebo,
                "models",
                "turtlebot3_burger",
                "model.sdf",
            ),
            "-name", "turtlebot3_burger",
            "-x", "2.0",
            "-y", "0.0",
            "-z", "0.1",
        ],
        output="screen",
        emulate_tty=True,
    )

    # ---------- 图像桥接 ----------
    image_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image"
            "@sensor_msgs/msg/Image@gz.msgs.Image"
        ],
        output="screen",
    )

    # ---------- 识别节点 ----------
    sim_recognition = Node(
        package="infer_track",
        executable="sim_recognition_node",
        output="screen",
    )

    # ---------- UDP 接收节点 ----------
    udp_yolo = Node(
        package="infer_track",
        executable="udp_yolo",
        output="screen",
    )

    # ---------- 坐标变换节点 ----------
    coordinate_transform = Node(
        package="infer_track",
        executable="coordinate_transform",
        output="screen",
    )

    # ---------- 跟踪控制节点 ----------
    tracking_node = Node(
        package="infer_track",
        executable="tracking_node",
        output="screen",
    )

     # ---------- 小车速度桥接 ----------
    cmd_vel_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"],
        output="screen",
    )

    # ---------- 裁判 UDP 桥接 ----------
    referee_udp_node = Node(
        package="infer_track",
        executable="referee_udp_node",
        output="screen",
    )

    # ---------- 任务状态机 ----------
    mission_manager = Node(
        package="infer_track",
        executable="mission_manager",
        output="screen",
    )

    mission_node = Node(
        package="uav_tracking_control",
        executable="mission_node",
        parameters=[os.path.join(
            get_package_share_directory("uav_tracking_control"),
            "config",
            "mission.yaml",
        )],
        output="screen",
    )

    # 返回启动描述（PX4 和 MicroXRCEAgent 先启动，其余节点随后启动）
    return LaunchDescription([
        px4_dir_arg,
        set_gz_path,
        px4_sitl,
        microxrce_agent,
        # 小车生成可以稍后执行，避免在 PX4 世界未完全加载时失败
        TimerAction(period=5.0, actions=[create_turtlebot]),
        # 图像桥接等节点可以立即启动，它们会自动等待话题
        image_bridge,
        sim_recognition,
        udp_yolo,
        coordinate_transform,
        tracking_node,
        cmd_vel_bridge,
        referee_udp_node,
        mission_manager,
        mission_node,
    ])
