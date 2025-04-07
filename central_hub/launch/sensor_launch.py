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
            executable='SPO2',
            name='SPO2_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='sensor',
            executable='ABPS',
            name='ABPS_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='sensor',
            executable='ABPD',
            name='ABPD_node',
            output='Glucose'
        ),

    ])

