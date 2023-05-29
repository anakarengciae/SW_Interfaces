#!/home/robotics/tools/miniconda3/envs/py38/bin/python
import rospy
from geometry_msgs.msg import PointStamped

def coordinates_callback(msg):
    # Process the received coordinates here
    x = msg.point.x
    y = msg.point.y
    timestamp = msg.header.stamp.to_sec()

    # Print the received coordinates and timestamp
    print("Received coordinates: (x={}, y={}) at timestamp: {}".format(x, y, timestamp))

def main():
    rospy.init_node('coordinates_subscriber')
    rospy.Subscriber('green_object_coordinates', PointStamped, coordinates_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
