from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)

    motor_joints = [
        f"/model/{namespace}/joint/motor_port_joint",
        f"/model/{namespace}/joint/motor_stbd_joint",
    ]

    blueboat_arguments = (
    [f"{joint}/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double" for joint in motor_joints]
    + [f"{joint}/ang_vel@std_msgs/msg/Float64@gz.msgs.Double" for joint in motor_joints]
    + [
        f"{joint}/enable_deadband@std_msgs/msg/Bool@gz.msgs.Boolean"
        for joint in motor_joints
    ]
    + [
        f"/model/{namespace}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        f"/model/{namespace}/odometry_with_covariance@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance",
        f"/model/{namespace}/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V",
        f"/world/oceans_waves/model/{namespace}/link/camera_link/sensor/front_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
        f"/world/oceans_waves/model/{namespace}/link/camera_link/sensor/front_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
    ]
)

    blueboat_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=blueboat_arguments,
        output="screen",
    )
    # blueboat_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=blueboat_arguments,
    #     remappings=[
    #     (
    #         f"/world/oceans_waves/model/{namespace}/link/camera_link/sensor/front_camera/image",
    #         f"/model/{namespace}/camera/image",
    #     ),
    #     (
    #         f"/world/oceans_waves/model/{namespace}/link/camera_link/sensor/front_camera/camera_info",
    #         f"/model/{namespace}/camera/camera_info",
    #     ),
    #     ],
    #     output="screen",
    # )

    nodes = [blueboat_bridge]

    return nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "namespace",
            default_value="blueboat",
            description="Namespace",
        ),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])