import math
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_multiply(lhs: tuple[float, float, float, float], rhs: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _xacro_arg(name: str, value: object) -> str:
    return f"{name}:={value}"


def load_xacro_arguments() -> list[str]:
    config_path = Path(__file__).resolve().parents[1] / "config" / "sensors.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    params = config["robot_description"]["ros__parameters"]

    lidar_xyz = [float(v) for v in params["lidar_xyz"]]
    lidar_rpy = [float(v) for v in params["lidar_rpy"]]
    lidar_axis_rpy = [float(v) for v in params["lidar_axis_rpy"]]
    lidar_final_quat = quaternion_multiply(
        quaternion_from_rpy(*lidar_rpy),
        quaternion_from_rpy(*lidar_axis_rpy),
    )
    lidar_level_yaw = yaw_from_quaternion(*lidar_final_quat)

    return [
        _xacro_arg("base_frame", params["base_frame"]),
        _xacro_arg("base_footprint_frame", params["base_footprint_frame"]),
        _xacro_arg("lidar_mount_frame", params["lidar_mount_frame"]),
        _xacro_arg("lidar_frame", params["lidar_frame"]),
        _xacro_arg("lidar_level_frame", params.get("lidar_level_frame", "lidar_level_link")),
        _xacro_arg("imu_frame", params["imu_frame"]),
        _xacro_arg("lidar_x", lidar_xyz[0]),
        _xacro_arg("lidar_y", lidar_xyz[1]),
        _xacro_arg("lidar_z", lidar_xyz[2]),
        _xacro_arg("lidar_roll", lidar_rpy[0]),
        _xacro_arg("lidar_pitch", lidar_rpy[1]),
        _xacro_arg("lidar_yaw", lidar_rpy[2]),
        _xacro_arg("lidar_axis_roll", lidar_axis_rpy[0]),
        _xacro_arg("lidar_axis_pitch", lidar_axis_rpy[1]),
        _xacro_arg("lidar_axis_yaw", lidar_axis_rpy[2]),
        _xacro_arg("lidar_level_x", lidar_xyz[0]),
        _xacro_arg("lidar_level_y", lidar_xyz[1]),
        _xacro_arg("lidar_level_z", lidar_xyz[2]),
        _xacro_arg("lidar_level_yaw", lidar_level_yaw),
        _xacro_arg("imu_x", float(params["imu_xyz"][0])),
        _xacro_arg("imu_y", float(params["imu_xyz"][1])),
        _xacro_arg("imu_z", float(params["imu_xyz"][2])),
        _xacro_arg("imu_roll", float(params["imu_rpy"][0])),
        _xacro_arg("imu_pitch", float(params["imu_rpy"][1])),
        _xacro_arg("imu_yaw", float(params["imu_rpy"][2])),
    ]


def generate_launch_description():
    xacro_arguments = load_xacro_arguments()
    robot_description = ParameterValue(
        Command(
            ["xacro ", PathJoinSubstitution([FindPackageShare("robot_description"), "urdf", "robot.urdf.xacro"]), " "]
            + [f"{arg} " for arg in xacro_arguments]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            )
        ]
    )
