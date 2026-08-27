ACTIVE_NAVIGATION_STATES = {
    "sending",
    "running",
    "pausing",
    "canceling",
}

UNFINISHED_NAVIGATION_STATES = {
    *ACTIVE_NAVIGATION_STATES,
    "paused",
}


def initial_navigation_state():
    """返回互不共享的导航任务初始状态"""
    return {
        "state": "idle",
        "message": "",
        "current_waypoint": 0,
        "total_waypoints": 0,
        "completed_waypoints": 0,
        "missed_waypoints": [],
    }
