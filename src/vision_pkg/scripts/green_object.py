#!/home/robotics/tools/miniconda3/envs/py38/bin/python
import rospy
import sys
import os
import yaml
from cv_bridge import CvBridge
import cv2
import numpy as np
import ctypes
from geometry_msgs.msg import PointStamped
import time

# Load the C++ library
lib = ctypes.CDLL('./green_object.so')

# Declare the argument and return types for the function
lib.multiplyCoordinates.argtypes = [ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
lib.multiplyCoordinates.restype = None

class DetectGreenObject:
    def __init__(self, image_folder):
        rospy.init_node('green_object_detection')
        self.image_folder = image_folder
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher('green_object_coordinates', PointStamped, queue_size=10)

    def process_images(self):
        image_files = os.listdir(self.image_folder)

        for file_name in image_files:
            file_path = os.path.join(self.image_folder, file_name)
            if os.path.isfile(file_path):
                self.detect_green_objects(file_path)

    def detect_green_objects(self, image_path):
        cv_image = cv2.imread(image_path)

        # Convert image to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for green color
        lower_green = np.array([35, 50, 50])  # Adjust these values according to your green object
        upper_green = np.array([65, 255, 255])  # Adjust these values according to your green object

        # Threshold the HSV image to get only green pixels
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)

        # Find contours of green objects
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Calculate x and y coordinates of green objects and draw rectangles
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cx = x + w // 2
            cy = y + h // 2

            # Declare the C integer variables
            cx_c = ctypes.c_int(cx)
            cy_c = ctypes.c_int(cy)

            # Call the C++ function to multiply the coordinates
            lib.multiplyCoordinates(ctypes.byref(cx_c), ctypes.byref(cy_c))

            # Retrieve the modified values
            cx_modified = cx_c.value
            cy_modified = cy_c.value

            # Generate a human-readable timestamp
            timestamp_str = time.strftime("%a %b %d %H:%M:%S %Z %Y", time.localtime(rospy.get_time()))

            # Publish the modified coordinates with timestamp
            point = PointStamped()
            point.header.stamp = rospy.Time.now()
            point.header.frame_id = "base_link"
            point.point.x = cx_modified
            point.point.y = cy_modified
            point.point.z = 0.0
            self.publisher.publish(point)

            coordinates_text = f"({cx}, {cy})"
            cv2.putText(cv_image, coordinates_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.circle(cv_image, (cx, cy), 3, (0, 0, 255), -1)  # Draw a red dot at the center of the object
            print("Detected green object at coordinates (x={}, y={}) in file: {}".format(cx, cy, image_path))
            print("Modified coordinates: (x={}, y={}) in file: {}".format(cx_modified, cy_modified, image_path))
            print("Timestamp:", timestamp_str)

        # Display the image with the detected green objects
        cv2.imshow("Image with Green Objects", cv_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def main():
    if len(sys.argv) < 2:
        print("Please provide the path to the image folder.")
        return

    image_folder = sys.argv[1]
    if not os.path.isdir(image_folder):
        print("The provided path is not a valid folder.")
        return

    detector = DetectGreenObject(image_folder)
    detector.process_images()

if __name__ == '__main__':
    main()
