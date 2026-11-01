"""SO-101 in lecture Humble/Fortress, using local world assets."""
import os
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node, SetParameter
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
from make_description import description

def generate_launch_description():
    desc=description()
    (ROOT/'generated.urdf').write_text(desc)
    return LaunchDescription([
        SetParameter(name='use_sim_time',value=True),
        ExecuteProcess(cmd=['ign','gazebo','-r','-s',os.environ.get('SO101_WORLD',str(ROOT.parent/'worlds/table_local.sdf'))],
                       additional_env={'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':os.environ.get('LD_LIBRARY_PATH','')},output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':desc}],output='screen'),
        Node(package='ros_gz_sim',executable='create',arguments=['-topic','/robot_description','-name','so101'],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock', *[f'/model/{name}/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose' for name in ['eraser','red_crayon','blue_crayon']], '/art_materials/camera@sensor_msgs/msg/Image[ignition.msgs.Image'],output='screen'),
        TimerAction(period=5.0,actions=[Node(package='controller_manager',executable='spawner',arguments=[name,'--controller-manager-timeout','90','--switch-timeout','30','--service-call-timeout','60'],output='screen') for name in ['joint_state_broadcaster','so101_arm_controller','so101_gripper_controller']]),
    ])
