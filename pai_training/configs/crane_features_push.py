features = {
    "observation.state": {
        "dtype": "float32",
        "shape": (5,),
        "names": ["joint1", "joint2", "joint4", "joint3", "gripper"],
    },
    # 立方体の位置 (x, y)。/cube_pose から取る（講義版は関節状態の複製だった）
    "observation.environment_state": {
        "dtype": "float32",
        "shape": (2,),
        "names": ["cube_x", "cube_y"],
    },
    "action": {
        "dtype": "float32",
        "shape": (5,),
        "names": ["joint1", "joint2", "joint4", "joint3", "gripper"],
    },
}
# Below is used for the dataset conversion
robot_type = "crane_plus"
repo_id = "crane_plus_push"
root = "/home/ubuntu/dataset/lerobot_dataset/crane_plus_push"
task_name = "push"
