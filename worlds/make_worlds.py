#!/usr/bin/env python3
"""產生步態測試場地: python3 make_worlds.py            -> flat.world + slope.world (10°)
                     python3 make_worlds.py 5 15       -> 另外產生 slope_5.world、slope_15.world

物理設定 (步長 1 ms、ODE、地面摩擦 mu 100 / mu2 50) 與 ../DOGZILLA/sim_env/worlds 相同, 結果才能與主專案比較。
斜坡: 機器狗在原點朝 +x 生成, 前方 1 m 平地後接上坡 (長 1.5 m), 坡頂再接 1.5 m 平台。坡面寬 1.2 m。
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FRICTION = "<friction><ode><mu>100</mu><mu2>50</mu2></ode></friction>"


def world(models, camera):
    return f"""<?xml version="1.0"?>
<sdf version='1.6'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1</direction>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface>{FRICTION}</surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
        </visual>
      </link>
    </model>
{models}
    <gravity>0 0 -9.8</gravity>
    <physics name='default_physics' default='0' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene><ambient>0.4 0.4 0.4 1</ambient><background>0.7 0.7 0.7 1</background><shadows>1</shadows></scene>
    <gui fullscreen='0'>
      <camera name='user_camera'><pose>{camera}</pose><view_controller>orbit</view_controller></camera>
    </gui>
  </world>
</sdf>
"""


def box(name, x, z, sx, sy, sz, pitch=0.0, color='0.6 0.6 0.6 1'):
    g = f"<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>"
    return (f"    <model name='{name}'><static>1</static><pose>{x:.4f} 0 {z:.4f} 0 {pitch:.5f} 0</pose><link name='l'>\n"
            f"      <collision name='c'>{g}<surface>{FRICTION}</surface></collision>\n"
            f"      <visual name='v'>{g}<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>\n"
            f"    </link></model>")


def slope(deg, x0=1.0, length=1.5, plateau=1.5, width=1.2, t=0.05):
    th = math.radians(deg)
    top_x, top_z = x0 + length * math.cos(th), length * math.sin(th)
    # 坡面方塊: 上表面從 (x0, 0) 到 (top_x, top_z), 中心沿法線往下 t/2; 繞 y 軸轉 -th 讓 +x 端抬高
    cx = x0 + length / 2 * math.cos(th) + t / 2 * math.sin(th)
    cz = length / 2 * math.sin(th) - t / 2 * math.cos(th)
    ramp = box('ramp', cx, cz, length, width, t, pitch=-th, color='0.55 0.5 0.4 1')
    # 坡頂平台: 實心方塊, 頂面與坡頂同高
    plat = box('plateau', top_x + plateau / 2, top_z / 2, plateau, width, top_z, color='0.5 0.5 0.5 1')
    # 起點標線 (只有外觀)
    note = (f"    <!-- 斜坡 {deg}°: 坡腳 x={x0}, 坡頂 x={top_x:.3f} 高 {top_z:.3f} m, 平台到 x={top_x + plateau:.3f} -->")
    return '\n'.join([note, ramp, plat])


def write(name, text):
    with open(os.path.join(HERE, name), 'w') as f:
        f.write(text)
    print('寫入', name)


if __name__ == '__main__':
    write('flat.world', world("    <!-- 空地: 只有地面 -->", '-1.5 -1.5 1.0 0 0.4 0.785'))
    write('slope.world', world(slope(10), '-1.0 -2.5 1.5 0 0.4 0.9'))
    for a in sys.argv[1:]:
        write(f'slope_{a}.world', world(slope(float(a)), '-1.0 -2.5 1.5 0 0.4 0.9'))
