from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video2',
            
        }]
    )


    tracker_node = Node(
        package='apriltag_tracker',
        executable='apriltag_tracker',
        name='apriltag_tracker',
        output='screen'
    )
    

    filter_node = Node(
        package='apriltag_tracker',
        executable='ekf_filter',
        name='ekf_filter',
        output='screen'
    )
    
    los_node = Node(
        package='apriltag_tracker',
        executable='los_planner',
        name='los_planner',
        output='screen'
    )
    
    
    cmdthrust_node = Node(
        package='apriltag_tracker',
        executable='cmd_thrust',
        name='cmd_thrust',
        output='screen'
    )


    viewer_node = Node(
        package='image_tools',
        executable='showimage',
        name='image_viewer',
        output='screen',
        remappings=[
            ('image', '/image_annotated')
        ]
    )

    return LaunchDescription([
        camera_node,
        tracker_node,
        filter_node,
        los_node,
        cmdthrust_node,
        viewer_node
    ])