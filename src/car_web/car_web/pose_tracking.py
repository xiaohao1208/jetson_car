import math


def normalize_angle(angle):
    """把平面角度限制到负π至π"""
    return math.atan2(math.sin(angle), math.cos(angle))


def compose_pose(first, second):
    """计算二维刚体变换first乘second"""
    cosine = math.cos(first["yaw"])
    sine = math.sin(first["yaw"])
    return {
        "x": first["x"] + cosine * second["x"] - sine * second["y"],
        "y": first["y"] + sine * second["x"] + cosine * second["y"],
        "yaw": normalize_angle(first["yaw"] + second["yaw"]),
    }


def inverse_pose(pose):
    """计算二维刚体变换的逆"""
    cosine = math.cos(pose["yaw"])
    sine = math.sin(pose["yaw"])
    return {
        "x": -cosine * pose["x"] - sine * pose["y"],
        "y": sine * pose["x"] - cosine * pose["y"],
        "yaw": normalize_angle(-pose["yaw"]),
    }


def map_odom_anchor(map_base, odom_base):
    """由同一时刻的地图位姿和里程计位姿计算地图到里程计变换"""
    return compose_pose(map_base, inverse_pose(odom_base))


def project_odom(anchor, odom_base):
    """用地图到里程计锚点把当前里程计位姿投影到地图"""
    return compose_pose(anchor, odom_base)
