from launch_ros.substitutions import FindPackageShare

robot_name = "simple_usv"

robot_description = {
    "model": "simple_usv",
    "model_path": "simple_usv",
}

spawn = {
    "x": 0.0,
    "y": 0.0,
    "z": -2.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}

use_sim_time = True