import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():


    descpkg = 'rbe500_example_py'
 
    urdf = os.path.join(get_package_share_directory('open_manipulator_x_description'),'urdf','open_manipulator_x.urdf.xacro')
    rviz = os.path.join(get_package_share_directory('rbe500_example_py'),'description','car.rviz')
    world = os.path.join(get_package_share_directory("rbe500_example_py"), 'worlds', 'car_arena.world')


    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    subprocess.run(['killall', 'gzserver'])
    subprocess.run(['killall', 'gzclient'])

    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', 
            '-s', 'libgazebo_ros_factory.so'],
            output='screen')


    return LaunchDescription([

        gazebo,

        Node(
            package = descpkg,
            name = 'entity_spawner',
            executable = 'spawn_manipulator',
        ),

        Node(
            package='robot_state_publisher',
            name='robot_state_publisher',
            executable='robot_state_publisher',
            output={'both': 'log'},
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz],
            output={'both': 'log'}
        ),
        
        # Node(
        #     package = descpkg,
        #     name = 'map_publisher',
        #     executable = 'map',
        # ),

    ])

def main():

    generate_launch_description()

if __name__ == '__main__':
    main()
