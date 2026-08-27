def mapping_launch_command():
    """返回不启动 RViz 的标准建图进程参数数组"""
    return [
        "ros2",
        "launch",
        "car_mapping",
        "mapping.launch.py",
        "use_rviz:=false",
    ]


def navigation_launch_command(map_path, mode):
    """返回指定地图和模式的标准导航进程参数数组"""
    return [
        "ros2",
        "launch",
        "car_navigation",
        "navigation.launch.py",
        f"map:={map_path}",
        f"navigation_mode:={mode}",
        "use_rviz:=false",
    ]
