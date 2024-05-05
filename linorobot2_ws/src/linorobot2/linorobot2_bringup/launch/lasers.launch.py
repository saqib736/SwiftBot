# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node


def generate_launch_description():
    ydlidar_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_bringup"), "config", "ydlidar.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sensor', 
            default_value='ydlidar',
            description='Sensor to launch'
        ),

        DeclareLaunchArgument(
            name='topic_name', 
            default_value='scan',
            description='Laser Topic Name'
        ),

        DeclareLaunchArgument(
            name='frame_id', 
            default_value='laser',
            description='Laser Frame ID'
        ),

        Node(
            condition=LaunchConfigurationEquals('sensor', 'ydlidar'),
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            remappings=[('scan', LaunchConfiguration('topic_name'))],
            parameters=[{ 
                'port': '/dev/ydlidar',
                'frame_id': LaunchConfiguration('frame_id'),
                'ignore_array': '',
                'baudrate': 128000,
                'lidar_type': 1,
                'device_type': 6,
                'sample_rate': 5,
                'abnormal_check_count': 4,
                'fixed_resolution': True,
                'reversion': False,
                'inverted': True,
                'auto_reconnect': True,
                'isSingleChannel': False,
                'intensity': False,
                'support_motor_dtr': True,
                'angle_max': 180.0,
                'angle_min': -180.0,
                'range_max': 10.0,
                'range_min': 0.12,
                'frequency': 5.0,
                'invalid_range_is_inf': False
            }]
        ),

        Node(
            condition=LaunchConfigurationEquals('sensor', 'rplidar'),
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            remappings=[('scan', LaunchConfiguration('topic_name'))],
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                'frame_id': LaunchConfiguration('frame_id'),
                'inverted': False,
                'angle_compensate': True,
            }],
        ),

        Node( 
            condition=LaunchConfigurationEquals('sensor', 'xv11'),
            name='xv_11_driver',
            package='xv_11_driver',
            executable='xv_11_driver',
            output='screen',
            remappings=[('scan', LaunchConfiguration('topic_name'))],
            parameters=[{
                'port': '/dev/ttyACM0',
                'baud_rate': 115200, 
                'frame_id': LaunchConfiguration('frame_id'),
                'firmware_version': 2
            }],
        ),

        Node(
            condition=LaunchConfigurationEquals('sensor', 'ldlidar'),
            package='ldlidar_sl_ros2',
     	    executable='ldlidar_sl_ros2_node',
      	    name='ldlidar_publisher_ld14',
      	    output='screen',
            parameters=[
		{'product_name': 'LDLiDAR_LD14'},
        	{'laser_scan_topic_name': 'scan'},
        	{'point_cloud_2d_topic_name': 'pointcloud2d'},
        	{'frame_id': 'laser'},
        	{'port_name': '/dev/ttyUSB0'},
        	{'serial_baudrate' : 115200},
       	 	{'laser_scan_dir': True},
       	 	{'enable_angle_crop_func': False},
        	{'angle_crop_min': 135.0},
       		{'angle_crop_max': 225.0}
      ]
        )
    ])

