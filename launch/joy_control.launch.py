from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define launch arguments
    rname_arg = DeclareLaunchArgument('rname', default_value='a1')
    ctrl_level_arg = DeclareLaunchArgument('ctrl_level', default_value='highlevel')
    firmwork_arg = DeclareLaunchArgument('firmwork', default_value='3_2')

    # Define nodes
    lcm_server_node = Node(
        package='unitree_ros2_to_real_main',
        executable='lcm_server_3_2',
        name='lcm_server_node',
        output='screen',
        arguments=[LaunchConfiguration('rname'), LaunchConfiguration('ctrl_level')]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node'
    )

    joy_to_twist_node = Node(
        package='unitree_ros2_to_real_main',
        executable='joy_to_twist',
        name='joy_to_twist_node'
    )

    twist_driver_node = Node(
        package='unitree_ros2_to_real_main',
        executable='twist_driver',
        name='twist_driver_node',
        parameters=[
            {
                "start_walking": False,
                "using_imu_publisher": True
                //"using_low_publisher": True #Modify
            }
        ]
    )

    # Create launch description
    ld = LaunchDescription()
    ld.add_action(rname_arg)
    ld.add_action(ctrl_level_arg)
    ld.add_action(firmwork_arg)
    ld.add_action(lcm_server_node)
    ld.add_action(joy_node)
    ld.add_action(joy_to_twist_node)
    ld.add_action(twist_driver_node)
    
    return ld
