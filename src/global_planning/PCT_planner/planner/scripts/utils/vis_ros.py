from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


def traj2ros(traj):
    """Convert trajectory to ROS 2 Path message"""
    path_msg = Path()
    path_msg.header = Header()
    path_msg.header.frame_id = "map"
    # Note: In ROS 2, header.stamp is automatically set by the publisher
    # If you need to set it manually, use: path_msg.header.stamp = node.get_clock().now().to_msg()

    for waypoint in traj:
        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(waypoint[0])
        pose.pose.position.y = float(waypoint[1])
        pose.pose.position.z = float(waypoint[2])
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)

    return path_msg
