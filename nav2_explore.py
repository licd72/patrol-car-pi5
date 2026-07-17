import rclpy, numpy as np, math, time
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

class Nav2Explorer(Node):
    def __init__(self):
        super().__init__('nav2_explorer')
        self.map = None
        self.pose = (0.0, 0.0)
        self.goal_active = False
        self.no_frontier_count = 0
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, MAP_QOS)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(5.0, self.loop)
        self.get_logger().info('Nav2Explorer ready')

    def cb_map(self, m): self.map = m
    def cb_odom(self, m):
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y)

    def loop(self):
        if self.map is None:
            self.get_logger().info('waiting map...', throttle_duration_sec=10)
            return
        if self.goal_active:
            return
        
        f = self.find_frontier()
        if f is None:
            self.no_frontier_count += 1
            self.get_logger().info('no frontier (%d/6)' % self.no_frontier_count)
            if self.no_frontier_count >= 6:
                # 30秒无前沿，强制往前探索
                self.get_logger().info('auto-explore forward!')
                f = (self.pose[0] + 2.0, self.pose[1])
                self.no_frontier_count = 0
            else:
                return
        
        self.no_frontier_count = 0
        self.get_logger().info('GO to (%.2f, %.2f)' % (f[0], f[1]))
        self.goal_active = True
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(f[0])
        goal.pose.pose.position.y = float(f[1])
        goal.pose.pose.orientation.w = 1.0
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_done)

    def goal_done(self, future):
        self.goal_active = False
        try:
            result = future.result()
            if result.accepted:
                self.get_logger().info('goal accepted, navigating...')
            else:
                self.get_logger().info('goal rejected, will retry next cycle')
        except Exception as e:
            self.get_logger().info(f'goal failed: {e}, will retry')

    def find_frontier(self):
        d = np.array(self.map.data, dtype=np.int8).reshape(
            self.map.info.height, self.map.info.width)
        free = d == 0
        unk = d == -1
        fr = np.zeros_like(free, dtype=bool)
        fr[1:, :] |= free[1:, :] & unk[:-1, :]
        fr[:-1, :] |= free[:-1, :] & unk[1:, :]
        fr[:, 1:] |= free[:, 1:] & unk[:, :-1]
        fr[:, :-1] |= free[:, :-1] & unk[:, 1:]
        if not np.any(fr):
            return None
        pts = np.argwhere(fr)
        if len(pts) < 5:
            return None
        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        wx = ox + pts[:, 1] * res
        wy = oy + pts[:, 0] * res
        dists = np.sqrt((wx - self.pose[0])**2 + (wy - self.pose[1])**2)
        idx = int(np.argmin(dists))
        if dists[idx] < 0.5:
            return None
        return (float(wx[idx]), float(wy[idx]))

rclpy.init()
n = Nav2Explorer()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
finally:
    n.destroy_node()
    rclpy.shutdown()
