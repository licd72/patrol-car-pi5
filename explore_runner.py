import rclpy, numpy as np, math, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy

MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.scan = None; self.map = None; self.pose = (0.0, 0.0, 0.0)
        self.state = 'IDLE'; self._lt = 0; self._spin_count = 0
        self._avoid_start = 0; self._force_end = 0
        self.safe = 0.25; self.fwd = 0.12; self.turn_s = 0.3
        self.create_subscription(LaserScan,'/scan',self.cb_s,qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/map',self.cb_m,MAP_QOS)
        self.create_subscription(Odometry,'/odom',self.cb_o,10)
        self.cmd=self.create_publisher(Twist,'/cmd_vel',10)
        self.timer=self.create_timer(0.1,self.loop)
        self.get_logger().info('Explorer ready. Auto-start in 3s...')
        self._start_delay = time.time() + 3.0

    def cb_s(self, m): self.scan = m
    def cb_m(self, m): self.map = m
    def cb_o(self, m):
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                     2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w))

    def loop(self):
        if self.state == 'IDLE':
            if time.time() > self._start_delay and self.scan and self.map:
                self.state = 'EXPLORE'
                self.get_logger().info('Auto-start!')
            return
        now = time.time()
        if now - self._lt > 3:
            self._lt = now
            self.get_logger().info('%s spin=%d pose=(%.1f,%.1f)' % (self.state, self._spin_count, self.pose[0], self.pose[1]))
        
        obs = self._obs()
        if obs and self.state != 'AVOID':
            self.state = 'AVOID'; self._avoid_start = now
        elif not obs and self.state == 'AVOID':
            self.state = 'EXPLORE'
        if self.state == 'AVOID' and now - self._avoid_start > 5.0:
            self.get_logger().info('AVOID timeout, force EXPLORE')
            self.state = 'EXPLORE'
            
        if self.state == 'EXPLORE': self._explore()
        elif self.state == 'AVOID': self._avoid()

    def _explore(self):
        if time.time() < self._force_end: t = Twist(); t.linear.x = self.fwd; t.angular.z = 0.1; self.cmd.publish(t); return
        if self.map is None: return
        f = self._best_frontier()
        if f is None:
            self._spin_count += 1
            if self._spin_count > 50:
                self.get_logger().info('no frontier %ds, force forward!' % (self._spin_count // 10))
                t = Twist(); t.linear.x = self.fwd; t.angular.z = 0.1
                self.cmd.publish(t); self._force_end = time.time() + 2.0; return
                self.cmd.publish(t); self._spin_count = 0; return
            t = Twist(); t.angular.z = 0.3; self.cmd.publish(t); return
        self._spin_count = 0
        a = math.atan2(f[1] - self.pose[1], f[0] - self.pose[0])
        d = self._norm(a - self.pose[2])
        if abs(d) < 0.2:
            self.get_logger().info('GO to (%.2f,%.2f)' % (f[0], f[1]))
            t = Twist(); t.linear.x = self.fwd; t.angular.z = 0.3 * d
            self.cmd.publish(t)
        else:
            t = Twist(); t.angular.z = self.turn_s if d > 0 else -self.turn_s
            self.cmd.publish(t)

    def _best_frontier(self):
        if self.map is None: return None
        d = np.array(self.map.data, dtype=np.int8).reshape(self.map.info.height, self.map.info.width)
        free = d == 0; unk = d == -1; fr = np.zeros_like(free)
        fr[:-1,:] |= (free[:-1,:] & unk[1:,:]); fr[1:,:] |= (free[1:,:] & unk[:-1,:])
        fr[:,:-1] |= (free[:,:-1] & unk[:,1:]); fr[:,1:] |= (free[:,1:] & unk[:,:-1])
        if not np.any(fr): return None
        pts = np.argwhere(fr)
        if len(pts) < 5: return None
        # 选距离车体最近的前沿点
        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        wx = ox + pts[:,1] * res
        wy = oy + pts[:,0] * res
        dists = np.sqrt((wx - self.pose[0])**2 + (wy - self.pose[1])**2)
        idx = np.argmin(dists)
        if dists[idx] < 0.3: return None
        return (float(wx[idx]), float(wy[idx]))

    def _obs(self):
        if self.scan is None: return False
        r = self.scan.ranges; n = len(r); h = int(15 * n / 360); c = n // 2
        return any(0 < r[i] < self.safe for i in range(max(0, c - h), min(n, c + h)))

    def _avoid(self):
        r = self.scan.ranges; n = len(r)
        # 前方太近时先后退
        front = [x for x in r[n//2-10:n//2+10] if x > 0]
        if front and min(front) < 0.20:
            t = Twist(); t.linear.x = -0.08
            self.cmd.publish(t); return
        l = np.mean([x for x in r[n//4:3*n//8] if x > 0] or [10])
        ri = np.mean([x for x in r[5*n//8:3*n//4] if x > 0] or [10])
        t = Twist(); t.angular.z = 0.4 if ri > l else -0.4
        self.cmd.publish(t)

    def _norm(self, a):
        while a > math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

rclpy.init(); n = Explorer()
try: rclpy.spin(n)
except KeyboardInterrupt: pass
