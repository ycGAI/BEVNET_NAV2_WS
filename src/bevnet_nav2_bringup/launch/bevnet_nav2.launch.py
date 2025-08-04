from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 获取包路径
    bringup_dir = FindPackageShare('bevnet_nav2_bringup')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    
    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file',
        default=PathJoinSubstitution([bringup_dir, 'config', 'nav2_params.yaml']))
    
    # 参数替换
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )
    
    # Controller Server with BEVNet controller
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav')
        ]
    )
    
    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params]
    )
    
    # Recovery Server
    recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[configured_params]
    )
    
    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params]
    )
    
    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params]
    )
    
    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'recoveries_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )
    
    # 路径可视化节点
    path_visualizer = Node(
        package='bevnet_nav2_core',
        executable='path_visualizer',
        name='path_visualizer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Costmap融合节点
    costmap_fusion = Node(
        package='bevnet_nav2_core',
        executable='costmap_fusion',
        name='costmap_fusion',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'fusion_method': 'weighted_average',
            'bevnet_weight': 0.7
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([bringup_dir, 'config', 'nav2_params.yaml']),
            description='Full path to the Nav2 parameters file'
        ),
        
        # Nav2 nodes
        controller_server,
        planner_server,
        recoveries_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        
        # BEVNet integration nodes
        path_visualizer,
        costmap_fusion
    ])