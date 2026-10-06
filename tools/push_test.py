"""推力測試: 站著與前進時, 從側面 / 前方推機身一下 (0.1 s), 推力逐步加大, 找出會翻倒的推力邊界。
用法 (容器內, 以空地啟動): python3 /tools/push_test.py [推力 N ...]      預設 1 2 3 4 6 8 N

推力由 Gazebo 的 /apply_link_wrench 施加在 champ::base_link (需要 libgazebo_ros_force_system.so, launch 已載入)。
方向依推的當下機身朝向換算成世界座標。每次推完記錄 3 秒:
  最大傾斜、恢復時間 (傾斜回到 5° 以內並維持)、側向位移、是否翻倒 (傾斜 > 60°)。
翻倒後呼叫 /reset_world 把機器狗放回原點; 放回後站不起來就停止 (之後的結果無效)。
最後一行 SUMMARY 是各情境「不會翻倒的最大推力」, 給參數掃描或比較不同負載用。
"""
import math
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from gazebo_msgs.srv import ApplyLinkWrench
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

FORCES = [float(f) for f in sys.argv[1:]] or [1, 2, 3, 4, 6, 8]
PUSH_TIME, WATCH, FLIP_DEG, OK_DEG = 0.1, 3.0, 60.0, 5.0
SCENARIOS = [('站著-側推', 0.0, 'side'), ('站著-前推', 0.0, 'front'), ('前進0.1-側推', 0.1, 'side')]

rclpy.init()
n = rclpy.create_node('push_test')
st = {}


def on_gt(m):
    p = m.pose.pose; q = p.orientation
    st['tilt'] = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))
    st['yaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    st['xy'] = (p.position.x, p.position.y)


n.create_subscription(Odometry, '/demo/odom/ground_truth', on_gt, 50)
pub = n.create_publisher(Twist, '/cmd_vel', 10)
wrench = n.create_client(ApplyLinkWrench, '/apply_link_wrench')
reset = n.create_client(Empty, '/reset_world')


def spin_for(sec, v=None):
    t0 = time.time()
    while time.time() - t0 < sec:
        if v is not None:
            t = Twist(); t.linear.x = v; pub.publish(t)
        rclpy.spin_once(n, timeout_sec=0.02)


def call(cli, req):
    fut = cli.call_async(req)
    while not fut.done():
        rclpy.spin_once(n, timeout_sec=0.02)
    return fut.result()


def push(force, direction):
    yaw = st['yaw']
    ang = yaw + (math.pi / 2 if direction == 'side' else 0.0)       # 側推 = 從機身右側往左推
    req = ApplyLinkWrench.Request()
    req.link_name = 'champ::base_link'
    req.wrench.force.x, req.wrench.force.y = force * math.cos(ang), force * math.sin(ang)
    req.duration = Duration(sec=0, nanosec=int(PUSH_TIME * 1e9))
    return call(wrench, req).success


def upright_after_reset():
    call(reset, Empty.Request())
    spin_for(4.0, 0.0)
    return st['tilt'] < 15


for cli, name in ((wrench, '/apply_link_wrench'), (reset, '/reset_world')):
    if not cli.wait_for_service(timeout_sec=10):
        print(f'找不到 {name} 服務 (launch 需載入 libgazebo_ros_force_system.so)'); sys.exit(1)
while 'tilt' not in st:
    rclpy.spin_once(n, timeout_sec=0.1)

limits = {}
broken = False
print(f'推力 {FORCES} N, 每次 {PUSH_TIME:.1f} s; 每次推完觀察 {WATCH:.0f} s', flush=True)
for name, v, direction in SCENARIOS:
    print(f'\n== {name}', flush=True)
    limits[name] = 0.0
    for f in FORCES:
        spin_for(3.0, v)                         # 先穩定 (前進情境先走起來)
        x0, y0 = st['xy']; yaw0 = st['yaw']
        if not push(f, direction):
            print('  施力失敗'); broken = True; break
        t0 = time.time(); mx = 0.0; t_ok = None; flipped = False
        while time.time() - t0 < WATCH:
            spin_for(0.02, v)
            mx = max(mx, st['tilt'])
            if st['tilt'] > FLIP_DEG:
                flipped = True; break
            if st['tilt'] < OK_DEG:
                t_ok = t_ok or (time.time() - t0)
            else:
                t_ok = None
        dx, dy = st['xy'][0] - x0, st['xy'][1] - y0
        lateral = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
        rec = '翻倒' if flipped else (f'{t_ok:.2f} s' if t_ok is not None else f'>{WATCH:.0f} s')
        print(f'  {f:4.1f} N: 最大傾斜 {mx:5.1f}°  恢復 {rec:>7s}  側向位移 {lateral:+.3f} m', flush=True)
        if flipped:
            if not upright_after_reset():
                print('  reset 後站不起來, 停止測試'); broken = True
            break
        limits[name] = f
    if broken:
        break

pub.publish(Twist())
print('\nSUMMARY ' + ' '.join(f'{k}={v:.1f}N' for k, v in limits.items()) + (' (中途中止, 結果不完整)' if broken else ''))
