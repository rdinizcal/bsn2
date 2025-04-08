import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='patient',
            executable='patient',
            name='patient_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='sensor',
            executable='thermometer',
            name='thermometer_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='sensor',
            executable='oximeter',
            name='oximeter_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='sensor',
            executable='heart_rate',
            name='heart_rate_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='sensor',
            executable='ABPS',
            name='abps_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='sensor',
            executable='ABPD',
            name='ABPD_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='sensor',
            executable='glucose',
            name='glucose_node',
            output='screen'
        ),

    ])

