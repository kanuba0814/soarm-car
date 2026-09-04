from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
 return LaunchDescription([Node(package='soarm_room_search',executable='room_search_node',name='room_search',parameters=[{'localization_healthy':False,'map_id':''}])])
