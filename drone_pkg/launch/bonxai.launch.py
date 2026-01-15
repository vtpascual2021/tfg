from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='bonxai_server',
      executable='bonxai_server_node',
      name='bonxai_server',
      output='screen',
      parameters=[{
          'frame_id': 'drone0/map',
          'base_frame_id': 'base_link',
          'resolution': 0.5, # tamaño de voxel (m)
          'sensor_model/max_range': 20.0,
          'occupancy_min_z': 0.1,
          'occupancy_max_z': 1.0,
      }],
      remappings=[
          ('cloud_in', '/map'),
      ]
    )
])
