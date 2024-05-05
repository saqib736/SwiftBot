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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    laser_filter_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'config', 'box_laser_filter.yaml']
    )

    return LaunchDescription([
    
        Node(
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
        ),

        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                laser_filter_config_path
            ],
            remappings=[
                ('/scan', '/base/scan/unfiltered'),
                ('/scan_filtered', '/base/scan')
            ]
        )
    ])

