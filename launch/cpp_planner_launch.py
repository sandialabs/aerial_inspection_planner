import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    run_rviz = True

    log_level_launch_arg = DeclareLaunchArgument(
        'log_level', default_value='info'
    )
    
    config = os.path.join(
      get_package_share_directory('aerial_inspection_planner'),
      'config',
      'cpp_planner.yaml'
      )

    planner_node = Node(
                        package='aerial_inspection_planner',
                        # namespace='',
                        executable='cpp_planner',
                        # name='',
                        # prefix=['valgrind --leak-check=full --track-origins=yes --show-leak-kinds=all'],
                        # prefix=['valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --verbose'],
                        # prefix=["valgrind --tool=callgrind --callgrind-out-file='callgrind.listener.%p' "],
                        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                        parameters=[config])
                    
    rviz_node = Node(
                    package='rviz2',
                    namespace='',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d' + os.path.join(
                        get_package_share_directory(
                            'aerial_inspection_planner'), 'config', 'config.rviz')]
                )
    ld = LaunchDescription([
            log_level_launch_arg,
            planner_node
        ])
    if (run_rviz):
        ld.add_entity(rviz_node)
    
        
    return ld