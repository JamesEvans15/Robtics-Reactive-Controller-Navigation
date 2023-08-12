from launch import LaunchDescription
from launch_ros.actions import *
from launch.actions import *
from launch.substitutions import *
from launch.event_handlers import *
from launch.events import *

from project_4.disc_robot import load_disc_robot

def generate_launch_description():

    '''
    bag_in = DeclareLaunchArgument("bag_in")
    run_bag = ExecuteProcess(cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_in')])

    bag_out = DeclareLaunchArgument("bag_out")
    rec_bag = ExecuteProcess(cmd=['ros2', 'bag', 'record', "/person_locations", "/people_count_current", "/people_count_total", '-o', LaunchConfiguration('bag_out')])

    node = Node(package='project3',
                executable='sub')
    node2 = Node(package='project3',
                 executable='pub')
    
    event_handler = OnProcessExit(target_action=run_bag,
                              on_exit=[EmitEvent(event=Shutdown())])
    
    terminate_at_end = RegisterEventHandler(event_handler)

    ld = LaunchDescription([ terminate_at_end, bag_in, run_bag, bag_out, rec_bag, node, node2 ])

    

    return ld
    '''

    robot = load_disc_robot('/home/parallels/ros2_ws/src/project_4/project_4/robots/normal.robot')

    node = Node(package='project_4',
                executable="sim")
    
    node2 = Node(package='project_4',
                 executable="vel")
    
    show_robot = Node(package="robot_state_publisher",
                      executable="robot_state_publisher",
                      parameters=[{"robot_description": robot['urdf']}])
    
    node3 = Node(package="project_4",
                 executable="nav")
    
    node4 = Node(package="project_4",
                 executable="translator")
    
    ld = LaunchDescription([node, show_robot, node3, node4 ])

    return ld
    