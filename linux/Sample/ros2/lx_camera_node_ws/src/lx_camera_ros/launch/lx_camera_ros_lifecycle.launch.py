from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
  enable_rviz = LaunchConfiguration('enable_rviz')
  autostart = LaunchConfiguration('autostart')

  camera_node = LifecycleNode(
      package="lx_camera_ros",
      executable="lx_camera_lifecycle_node",
      namespace="lx_camera_node",
      name="lx_camera_node",
      output="screen",
      emulate_tty=True,
      parameters=[
          {"ip": "192.168.100.82"},
          {"log_path": "/var/log/"},
          {"is_xyz": 1},
          {"is_depth": 1},
          {"is_amp": 1},
          {"is_rgb": 1},
          {"lx_work_mode": 0},
          {"lx_application": 0},
          {"lx_tof_unit": 1},
          {"x": 0.0},
          {"y": 0.0},
          {"z": 0.0},
          {"roll": 0.0},
          {"pitch": 0.0},
          {"yaw": 0.0},
          {"raw_param": 0},
          {"lx_2d_binning": 0},
          {"lx_2d_undistort": 0},
          {"lx_2d_undistort_scale": 51},
          {"lx_2d_auto_exposure": 0},
          {"lx_2d_auto_exposure_value": 11},
          {"lx_2d_exposure": 10001},
          {"lx_2d_gain": 101},
          {"lx_rgb_to_tof": 0},
          {"lx_3d_binning": 0},
          {"lx_mulit_mode": 0},
          {"lx_3d_undistort": 0},
          {"lx_3d_undistort_scale": 0},
          {"lx_hdr": 0},
          {"lx_3d_auto_exposure": 1},
          {"lx_3d_auto_exposure_value": 50},
          {"lx_3d_first_exposure": 1100},
          {"lx_3d_second_exposure": 200},
          {"lx_3d_gain": 11},
          {"lx_min_depth": 0},
          {"lx_max_depth": 8000},
      ])

  configure_event = EmitEvent(
      event=ChangeState(
          lifecycle_node_matcher=matches_action(camera_node),
          transition_id=Transition.TRANSITION_CONFIGURE),
      condition=IfCondition(autostart))

  activate_event = RegisterEventHandler(
      OnStateTransition(
          target_lifecycle_node=camera_node,
          goal_state='inactive',
          entities=[
              EmitEvent(
                  event=ChangeState(
                      lifecycle_node_matcher=matches_action(camera_node),
                      transition_id=Transition.TRANSITION_ACTIVATE))
          ]),
      condition=IfCondition(autostart))

  rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='lx_camera',
      output='screen',
      arguments=['-d', os.path.join(get_package_share_directory('lx_camera_ros'),
                                    'rviz', 'lx_camera.rviz')],
      condition=IfCondition(enable_rviz))

  return LaunchDescription([
      DeclareLaunchArgument(
          'enable_rviz',
          default_value='true',
          description='Whether to launch rviz2'),
      DeclareLaunchArgument(
          'autostart',
          default_value='false',
          description='Automatically configure and activate lifecycle node'),
      camera_node,
      configure_event,
      activate_event,
      rviz_node,
  ])
