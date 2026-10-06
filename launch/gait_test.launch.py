"""DOGZILLA S1 步態測試 (容器內使用): Gazebo + CHAMP 步態控制, 不開 SLAM / Nav2 / RL。
測試程式直接送 /cmd_vel, 量的是步態控制器本身的表現。

    ros2 launch /launch/gait_test.launch.py                                  # 空地, 官方模型 (光達 14 g)
    ros2 launch /launch/gait_test.launch.py world:=/worlds/slope.world       # 10° 斜坡
    ros2 launch /launch/gait_test.launch.py lidar_mass:=0.17 lidar_z:=0.10   # 背上加 170 g 光達負載

Gazebo 流程與 ../DOGZILLA/sim_env/launch/dogzilla_sim.launch.py 相同 (暫停啟動、站姿生成、控制器設好才 unpause),
另外載入 libgazebo_ros_force_system.so, 提供 /apply_link_wrench 給推力測試 (tools/push_test.py) 使用。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    # 光達負載參數接在 xacro 路徑後面 (CHAMP 各節點都用 `xacro <description_path>` 產生模型)
    description = ['/description/dogzilla_s1_payload.urdf.xacro',
                   ' lidar_mass:=', LaunchConfiguration('lidar_mass'),
                   ' lidar_x:=', LaunchConfiguration('lidar_x'),
                   ' lidar_z:=', LaunchConfiguration('lidar_z'),
                   ' lidar_size:=', LaunchConfiguration('lidar_size')]
    champ_config = FindPackageShare('champ_config')

    champ_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('champ_bringup'), 'launch', 'bringup.launch.py'])),
        launch_arguments={
            'description_path': description,
            'joints_map_path': PathJoinSubstitution([champ_config, 'config/joints/joints.yaml']),
            'links_map_path': PathJoinSubstitution([champ_config, 'config/links/links.yaml']),
            'gait_config_path': '/config/gait_s1.yaml',
            'use_sim_time': 'true',
            'robot_name': 'champ',
            'gazebo': 'true',
            'rviz': 'false',
            'joint_controller_topic': 'joint_group_effort_controller/joint_trajectory',
            'hardware_connected': 'false',
            'publish_foot_contacts': 'false',
            'close_loop_odom': 'true',
        }.items(),
    )

    gz_params = '/ws/install/champ_gazebo/share/champ_gazebo/config/gazebo.yaml'
    gazebo = [
        ExecuteProcess(cmd=['gzserver', '-u', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
                            '-s', 'libgazebo_ros_force_system.so',
                            world, '--ros-args', '--params-file', gz_params], output='screen'),
        ExecuteProcess(cmd=['gzclient'], output='screen', condition=IfCondition(gui)),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'champ', '-topic', '/robot_description', '-robot_namespace', '',
                        '-x', '0.0', '-y', '0.0', '-z', '0.10', '-R', '0', '-P', '0', '-Y', '0.0']),
        ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive', 'joint_states_controller'],
                       output='screen'),
        ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive', 'joint_group_effort_controller'],
                       output='screen'),
        # S1 舵機只有 0.44 N·m, 控制器載入前腿是軟的; 以站姿暫停生成, 控制器啟動請求送出後才 unpause
        ExecuteProcess(output='screen', cmd=['bash', '-c',
            'until [ "$(ros2 control list_controllers 2>/dev/null | grep -c inactive)" -ge 2 ]; do sleep 0.5; done; '
            '(ros2 control switch_controllers --activate joint_states_controller joint_group_effort_controller &); '
            'sleep 1; ros2 service call /unpause_physics std_srvs/srv/Empty > /dev/null; sleep 3; '
            'n=$(ros2 control list_controllers 2>/dev/null | grep -v inactive | grep -c active); '
            'echo "[unpause] 開始物理模擬, active 控制器數 = $n (應為 2)"']),
    ]

    contact_sensor = Node(package='champ_gazebo', executable='contact_sensor', output='screen',
                          parameters=[{'use_sim_time': True}, PathJoinSubstitution([champ_config, 'config/links/links.yaml'])])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='/worlds/flat.world'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('lidar_mass', default_value='0.0', description='光達負載質量 (kg), 0 = 官方模型'),
        DeclareLaunchArgument('lidar_x', default_value='-0.017', description='負載前後位置 (m, 相對 base_link)'),
        DeclareLaunchArgument('lidar_z', default_value='0.09', description='負載中心高度 (m, 相對 base_link)'),
        DeclareLaunchArgument('lidar_size', default_value='0.07', description='負載方塊邊長 (m)'),
        GroupAction([champ_bringup]),   # Humble 的 include 參數會洩漏到上層, 用 GroupAction 限定
        *gazebo,
        contact_sensor,
    ])
