#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker


def main():
    rospy.init_node("collision_visualization")

    try:
        length: float = rospy.get_param("collision/length")
        width: float = rospy.get_param("collision/width")
        height: float = rospy.get_param("collision/height")

        offset = rospy.get_param("collision/offset", [0.0, 0.0, 0.0])

        eef_frame = rospy.get_param("eef_frame")

    except KeyError as e:
        rospy.logerr(f"Missing required parameter: {e}")
        return

    marker_pub = rospy.Publisher(
        "visualization_marker", Marker, latch=True, queue_size=0
    )

    marker = Marker()
    marker.header.frame_id = eef_frame
    marker.header.stamp = rospy.Time.now()

    marker.ns = "attached_box"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.frame_locked = True

    marker.pose.position.x = length / 2 - offset[0]
    marker.pose.position.y = offset[1]
    marker.pose.position.z = offset[2]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = length
    marker.scale.y = width
    marker.scale.z = height

    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.5

    marker_pub.publish(marker)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
