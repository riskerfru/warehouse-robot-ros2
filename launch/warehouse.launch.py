# =============================================================
#   WAREHOUSE ROBOT LAUNCH FILE
#   Starts all nodes with one command:
#   ros2 launch warehouse_robot warehouse.launch.py
# =============================================================

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():

    # API key argument
    api_key_arg = DeclareLaunchArgument(
        'api_key',
        default_value=os.getenv('ANTHROPIC_API_KEY', ''),
        description='Anthropic API key'
    )

    # Image Mapper Node
    image_mapper = Node(
        package='warehouse_robot',
        executable='image_mapper',
        name='image_mapper',
        output='screen',
        emulate_tty=True,
        env={'ANTHROPIC_API_KEY': os.getenv('ANTHROPIC_API_KEY', '')}
    )

    # Task Planner Node
    task_planner = Node(
        package='warehouse_robot',
        executable='task_planner',
        name='task_planner',
        output='screen',
        emulate_tty=True,
        env={'ANTHROPIC_API_KEY': os.getenv('ANTHROPIC_API_KEY', '')}
    )

    # Navigator Node
    navigator = Node(
        package='warehouse_robot',
        executable='navigator',
        name='navigator',
        output='screen',
        emulate_tty=True,
    )

    # Dashboard Server Node
    dashboard = Node(
        package='warehouse_robot',
        executable='dashboard',
        name='dashboard_server',
        output='screen',
        emulate_tty=True,
        env={'ANTHROPIC_API_KEY': os.getenv('ANTHROPIC_API_KEY', '')}
    )
       
       # pybullet_bridge = Node(
        #package='warehouse_robot',
        #executable='pybullet_bridge',
        #name='pybullet_bridge',
        #output='screen',
        #emulate_tty=True,
    #)


    return LaunchDescription([
        api_key_arg,
        image_mapper,
        task_planner,
        navigator,
        dashboard,
        
    ])