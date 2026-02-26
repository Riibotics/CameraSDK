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
  topic_rgb = LaunchConfiguration('topic_rgb')
  topic_rgb_info = LaunchConfiguration('topic_rgb_info')
  topic_amp = LaunchConfiguration('topic_amp')
  topic_depth = LaunchConfiguration('topic_depth')
  topic_tof_info = LaunchConfiguration('topic_tof_info')
  topic_error = LaunchConfiguration('topic_error')
  topic_pallet = LaunchConfiguration('topic_pallet')
  topic_frame_rate = LaunchConfiguration('topic_frame_rate')
  topic_cloud = LaunchConfiguration('topic_cloud')
  topic_tf = LaunchConfiguration('topic_tf')

  camera_topic_remappings = [
      ('LxCamera_Rgb', topic_rgb),
      ('LxCamera_RgbInfo', topic_rgb_info),
      ('LxCamera_Amp', topic_amp),
      ('LxCamera_Depth', topic_depth),
      ('LxCamera_TofInfo', topic_tof_info),
      ('LxCamera_Error', topic_error),
      ('LxCamera_Pallet', topic_pallet),
      ('LxCamera_FrameRate', topic_frame_rate),
      ('LxCamera_Cloud', topic_cloud),
      ('LxCamera_TF', topic_tf),
  ]

  camera_node = LifecycleNode(
      package="lx_camera_ros",
      executable="lx_camera_lifecycle_node",
      namespace="lx_camera_node",
      name="lx_camera_node",
      output="screen",
      emulate_tty=True,
      remappings=camera_topic_remappings,
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
      DeclareLaunchArgument(
          'topic_rgb',
          default_value='LxCamera_Rgb',
          description='RGB image topic name'),
      DeclareLaunchArgument(
          'topic_rgb_info',
          default_value='LxCamera_RgbInfo',
          description='RGB camera info topic name'),
      DeclareLaunchArgument(
          'topic_amp',
          default_value='LxCamera_Amp',
          description='Amplitude image topic name'),
      DeclareLaunchArgument(
          'topic_depth',
          default_value='LxCamera_Depth',
          description='Depth image topic name'),
      DeclareLaunchArgument(
          'topic_tof_info',
          default_value='LxCamera_TofInfo',
          description='ToF camera info topic name'),
      DeclareLaunchArgument(
          'topic_error',
          default_value='LxCamera_Error',
          description='Error topic name'),
      DeclareLaunchArgument(
          'topic_pallet',
          default_value='LxCamera_Pallet',
          description='Pallet result topic name'),
      DeclareLaunchArgument(
          'topic_frame_rate',
          default_value='LxCamera_FrameRate',
          description='Frame-rate/temperature topic name'),
      DeclareLaunchArgument(
          'topic_cloud',
          default_value='LxCamera_Cloud',
          description='Point cloud topic name'),
      DeclareLaunchArgument(
          'topic_tf',
          default_value='LxCamera_TF',
          description='TF mirror topic name'),
      camera_node,
      configure_event,
      activate_event,
      rviz_node,
  ])
