"""斜坡測試: 以固定速度直走上坡 (worlds/slope*.world), 量爬坡速度、扣掉坡度後的晃動、有沒有滑下來或翻倒。
用法 (容器內, 以斜坡場地啟動): python3 /tools/slope_test.py [前進速度 m/s] [最長秒數]     預設 0.15、60

gait_eval.py 量的是絕對 roll/pitch, 在坡上會把坡度也算成晃動, 所以這裡把 pitch 減掉 1 秒移動平均 (= 當下坡度) 再算 RMS。
分三段統計 (依 x 位置): 平地 (x < 1.0)、坡面、坡頂平台。
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

V = float(sys.argv[1]) if len(sys.argv) > 1 else 0.15
T_MAX = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
RAMP_X0 = 1.0
rclpy.init()
n = rclpy.create_node('slope_test')
buf = []


def cb(m):
    p = m.pose.pose; q = p.orientation
    buf.append((time.time(), p.position.x, p.position.y, p.position.z,
                math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))),
                math.degrees(math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))),
                math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))))


n.create_subscription(Odometry, '/demo/odom/ground_truth', cb, 50)
pub = n.create_publisher(Twist, '/cmd_vel', 10)
while not buf:
    rclpy.spin_once(n, timeout_sec=0.1)
t0 = time.time(); flipped = False; top_x = None
while time.time() - t0 < T_MAX:
    t = Twist(); t.linear.x = V; pub.publish(t)
    rclpy.spin_once(n, timeout_sec=0.02)
    if buf[-1][6] > 60:
        flipped = True; break
    if buf[-1][1] > RAMP_X0 + 2.6:          # 走到坡頂平台中段就結束
        break
pub.publish(Twist())

a = np.array(buf)
a = a[a[:, 0] >= t0]
w = max(1, int(len(a) / max(a[-1, 0] - a[0, 0], 1e-6)))      # 約 1 秒的樣本數
pitch_trend = np.convolve(a[:, 5], np.ones(w) / w, mode='same')
pitch_dev = a[:, 5] - pitch_trend
z_climb = a[:, 3].max() - a[0, 3]
print(f'指令前進 {V} m/s, 用時 {a[-1, 0] - a[0, 0]:.1f} s, 最遠 x={a[:, 1].max():.2f} m, 爬升 {z_climb:.3f} m'
      + ('  ← 翻倒' if flipped else ''))
for name, sel in [('平地', a[:, 1] < RAMP_X0), ('坡面', (a[:, 1] >= RAMP_X0) & (a[:, 3] < a[:, 3].max() - 0.01)),
                  ('坡頂', (a[:, 1] >= RAMP_X0) & (a[:, 3] >= a[:, 3].max() - 0.01))]:
    b = a[sel]; pd = pitch_dev[sel]
    if len(b) < 20:
        print(f'  {name}: 資料不足'); continue
    dt = b[-1, 0] - b[0, 0]
    v = math.hypot(b[-1, 1] - b[0, 1], b[-1, 2] - b[0, 2]) / max(dt, 1e-6)
    rms = math.sqrt(np.mean(b[:, 4] ** 2 + pd ** 2))
    print(f'  {name}: 速度 {v:.3f} m/s | 晃動 RMS {rms:4.1f}° (roll 最大 {np.abs(b[:, 4]).max():4.1f}°, '
          f'pitch 偏離坡度最大 {np.abs(pd).max():4.1f}°) | 平均坡度 {pitch_trend[sel].mean():+.1f}°', flush=True)
reached = a[:, 1].max() > RAMP_X0 + 2.0
print(f'SUMMARY reached_top={int(reached)} flipped={int(flipped)} climb={z_climb:.3f}')
