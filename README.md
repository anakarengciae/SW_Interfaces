# Green Object Detection

This project focuses on detecting green objects in images using computer vision techniques. It provides a Python script that reads images from a specified folder, applies color thresholding to isolate green objects, and detects their coordinates. Additionally, it publishes the modified coordinates with timestamps as a ROS message.

## Prerequisites

- Python 3.x
- ROS (Robot Operating System)
- OpenCV
- cv_bridge
- numpy

## Installation

1. Clone the repository:
`git clone` https://github.com/anakarengciae/SW_Interfaces.git

2. Navigate to the repository directory:
`cd SW_Interfaces`

3. Build the C++ library:
`g++ -shared -o green_object.so green_object.cpp`


## Usage

1. Provide a folder path containing the images you want to process. Modify the `image_folder` variable in the `main()` function of the `green_object_detection.py` script.

2. Run the green object detection script:
`python green_object.py <image_folder_path>`

Replace `<image_folder_path>` with the actual path to your image folder.

3. The script will process each image in the folder, detect green objects, modify their coordinates using a C++ function, and display the image with the detected objects.

4. The modified coordinates with timestamps will be published as ROS messages with the topic name `green_object_coordinates`.

## Subscriber Connection
To connect the subscriber and receive green object coordinates, follow these steps:

1. Make sure the required dependencies and prerequisites are installed.

2. Modify the coordinates_callback function in the subscriber code (`green_object_subscriber.py`) according to your needs. This function is called whenever coordinates are received.

3. In a terminal, navigate to the project directory.

Run the following command to start the subscriber node:

`python gree_object_subscriber.py`

- Adjust the command based on your Python interpreter and file location.

4. The subscriber will now listen to the topic `green_object_coordinates` and print the received coordinates and timestamps.

## Customization

- Adjust the lower and upper bounds of the green color in the HSV color space (`lower_green` and `upper_green` variables) according to the specific green object you want to detect.

- Modify the C++ function `multiplyCoordinates()` in `green_object.cpp` to implement custom coordinate modifications.

- Customize the ROS topic name, frame ID, and message fields in the `DetectGreenObject` class to suit your needs.

## Currently working on the implementation and integration of the following components:
- Creation of a gRPC wrapper for converting ROS Topic coordinates to an RPC service.

- Development of a C# program for retrieving and displaying object coordinates from the RPC service.

- Utilization of grpc-Gateway to expose the wrapper's service as a REST API.

- Acquisition of object coordinates from the new REST API using Flask, followed by storing the data in a JSON file.

## Doxygen Documentation 
file:///C:/Documentacion_Doxygen/Salida_Doc/html/index.html

**Note:** The Doxygen documentation generation was unsuccessful during the testing. The issue might be related to the Docker environment and the folder path being unreachable. I apologize for any inconvenience caused.
