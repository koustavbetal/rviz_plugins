from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = FindPackageShare("polygon_editor")

    map_file = PathJoinSubstitution([pkg_share, 'maps', 'factory_hallway_edited.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'map_display.rviz'])
    coverage_params_file = PathJoinSubstitution([pkg_share, 'config', 'coverage_server.yaml'])

    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Full path to map yaml file'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='Full path to the RViz config file'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=coverage_params_file,
        description='Full path to coverage_server params yaml'
    )

    autostart_arg = DeclareLaunchArgument('autostart', default_value='true')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='')

    namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": LaunchConfiguration('map'),
            "use_sim_time": False
        }]
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ['map_server']
        }]
    )

    static_tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_to_odom",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )

    coverage_server = Node(
        package="opennav_coverage",
        executable="opennav_coverage",
        name="coverage_server",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        remappings=remappings,
    )

    lifecycle_manager_coverage = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_coverage",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ['coverage_server']
        }]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", LaunchConfiguration('rviz_config')]
    )

    ld = LaunchDescription()
    ld.add_action(map_yaml_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(params_file_arg)
    ld.add_action(autostart_arg)
    ld.add_action(namespace_arg)
    ld.add_action(map_server)
    ld.add_action(lifecycle_manager)
    ld.add_action(static_tf_map_to_odom)
    ld.add_action(coverage_server)
    ld.add_action(lifecycle_manager_coverage)
    ld.add_action(rviz)

    return ld