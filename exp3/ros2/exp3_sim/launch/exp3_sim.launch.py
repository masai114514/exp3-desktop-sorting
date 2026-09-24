#!/usr/bin/env python3
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _roots():
    share = Path(get_package_share_directory('exp3_sim')).resolve()
    for parent in (share, *share.parents):
        if parent.name == 'exp3' and (parent / 'sort_core').is_dir():
            return parent, parent.parent / 'c4_2'
        if (parent / 'src' / 'exp3' / 'sort_core').is_dir():
            return parent / 'src' / 'exp3', parent / 'src' / 'c4_2'
    exp3_root = os.environ.get('EXP3_ROOT')
    c4_root = os.environ.get('C4_ROOT')
    if exp3_root and c4_root:
        return Path(exp3_root), Path(c4_root)
    raise RuntimeError('Could not locate sibling exp3 and c4_2 source directories')


def _robot_description():
    share = get_package_share_directory('mecharm_grasp')
    path = Path(share) / 'urdf' / 'mecharm_270_sim_prism.urdf'
    controllers = Path(share) / 'config' / 'controllers_prism.yaml'
    text = path.read_text(encoding='utf-8')
    needle = '$(find mecharm_grasp)/config/controllers_prism.yaml'
    if needle not in text:
        raise RuntimeError('controller parameter placeholder missing from prism URDF')
    return text.replace(needle, str(controllers))


def generate_launch_description():
    exp3_root, c4_root = _roots()
    gui = LaunchConfiguration('gui')
    world = PathJoinSubstitution([FindPackageShare('exp3_sim'), 'worlds', 'exp3_table.world'])
    gzserver_launch = PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])
    gzclient_launch = PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py'])
    robot_description = ParameterValue(_robot_description(), value_type=str)

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzserver_launch),
        launch_arguments={'world': world, 'pause': 'false'}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzclient_launch),
        condition=IfCondition(gui),
    )
    xvfb = ExecuteProcess(
        cmd=['Xvfb', ':99', '-screen', '0', '1280x720x24', '-nolisten', 'tcp'],
        output='screen',
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'mecharm270',
                   '-x', '0', '-y', '0', '-z', '0.8'],
        output='screen',
    )

    def controller(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager-timeout', '300'],
                    output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        # run_task:=false 只起场景（world/机械臂/相机/检测器/pick_place_server），
        # 不启动 sim_task，便于单独取景标定、遮挡诊断与分段录屏。
        DeclareLaunchArgument('run_task', default_value='true'),
        # empty_cell:=c3 时该格故意不放物体 —— 任务书 §六「空网格应被跳过」的证据轮。
        # 证据形式是"那一格没有任何记录"（records.jsonl 无该行、reasons 无该键）；
        # 该轮 verdict 必然是 TRIAL（场上 5 个 < 验收线 6 个），必须与记分轮分开跑。
        # 留空 '' = 正常 6 物记分轮。
        DeclareLaunchArgument('empty_cell', default_value=''),
        DeclareLaunchArgument('detector', default_value='hsv'),
        SetEnvironmentVariable('EXP3_ROOT', str(exp3_root)),
        SetEnvironmentVariable('C4_ROOT', str(c4_root)),
        SetEnvironmentVariable('DISPLAY', ':99'),
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
        xvfb,
        TimerAction(period=2.0, actions=[gzserver]),
        gzclient,
        robot_state_publisher,
        TimerAction(period=8.0, actions=[spawn]),
        TimerAction(period=22.0, actions=[controller('joint_state_broadcaster')]),
        TimerAction(period=26.0, actions=[controller('joint_trajectory_controller')]),
        TimerAction(period=30.0, actions=[controller('gripper_controller')]),
        # detector:=hsv   吃 Gazebo 相机画面的 HSV 检测器（原链路，可证明检测算法有效）
        # detector:=truth 真值投影检测器（model_states → table_to_px → 检测框）
        #   ★ 为什么要有 truth：这台容器没有 GPU，Mesa 走 llvmpipe 软渲染，实测
        #     Gazebo 相机渲染会在跑到第 3~4 轮时**卡死**（帧戳照常推进、画面内容
        #     一字不变；run_20260924_110405 的 round4/5/6 三次扫描结果完全相同）。
        #     物理侧是好的（物块坐标精确、臂位姿实时），卡的只有渲染这一段。
        #     truth 模式只替换"像素→检测框"这一步，**标定与投影链路完全不变**，
        #     用来把故障隔离在渲染上、继续验证规划-取放这条主线。
        #     两者在报告里必须分开陈述：truth 是理想感知，不能用来证明检测算法有效。
        Node(package='exp3_sim', executable='color_detector',
             parameters=[{'image_topic': '/overhead/image_raw'}], output='screen',
             condition=IfCondition(PythonExpression(
                 ["'", LaunchConfiguration('detector'), "' == 'hsv'"]))),
        Node(package='exp3_sim', executable='truth_detector', output='screen',
             condition=IfCondition(PythonExpression(
                 ["'", LaunchConfiguration('detector'), "' == 'truth'"]))),
        TimerAction(period=35.0, actions=[
            Node(package='exp3_sim', executable='pick_place_server',
                 parameters=[{'base_z': 0.8,
                              'empty_cell': LaunchConfiguration('empty_cell')}],
                 output='screen')]),
        TimerAction(period=39.0, actions=[
            Node(package='exp3_sim', executable='sim_task', output='screen',
                 condition=IfCondition(LaunchConfiguration('run_task')))]),
    ])
