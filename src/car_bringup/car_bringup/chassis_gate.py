class ChassisReadyGate:
    """统计连续有效的底盘状态帧"""

    def __init__(self, ready_count):
        """保存所需连续帧数并清零累计值"""
        self.ready_count = max(1, int(ready_count))
        self.consecutive_ready = 0

    def update(self, wifi_ready, agent_ready):
        """更新底盘连接状态并返回门控是否已经满足"""
        if not wifi_ready or not agent_ready:
            self.consecutive_ready = 0
            return False
        self.consecutive_ready += 1
        return self.consecutive_ready >= self.ready_count
