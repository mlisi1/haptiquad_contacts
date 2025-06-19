import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription

def generate_launch_description():

    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Launch rviz')
    use_gt_arg = DeclareLaunchArgument('use_gt', default_value='false', description='Wether to use or not values from simulation')

    use_gt = LaunchConfiguration("use_gt")
    rviz = LaunchConfiguration("rviz")


    self_pkg = get_package_share_directory('haptiquad_contacts')

    estimator_config = os.path.join(self_pkg, 'config', 'contact_estimator.yaml')
    estimator_config_gt = os.path.join(self_pkg, 'config', 'contact_estimator_gt.yaml')
    rviz_config = os.path.join(self_pkg, 'config', 'contacts.rviz')


    haptiquad_contacts = Node(
        package="haptiquad_contacts", executable="contact_estimator",
        emulate_tty = True,
        parameters=[estimator_config],
        condition=UnlessCondition(use_gt)
    )  

    haptiquad_contacts_gt = Node(
        package="haptiquad_contacts", executable="contact_estimator",
        emulate_tty = True,
        parameters=[estimator_config_gt],
        condition=IfCondition(use_gt)
    ) 


    model_publisher = Node(
        package="haptiquad_contacts", executable="model_publisher.py",
        emulate_tty = True,
        parameters=[{"model_alpha": 1.0}],
    ) 

    mujoco_contact_publisher = Node(
        package="haptiquad_contacts", executable="mujoco_contact_publisher.py",
        emulate_tty = True,
        parameters=[{"object_name": "base_collisionbox"}],
    ) 





    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(rviz)
    )


    return LaunchDescription(
        [   
            rviz_arg,
            use_gt_arg,
            haptiquad_contacts,
            haptiquad_contacts_gt,
            rviz_node,
            model_publisher,
            mujoco_contact_publisher
        ]
    )