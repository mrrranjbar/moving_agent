#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def camera_feed():
    # Initialize the ROS node
    rospy.init_node('camera_node', anonymous=True)

    # Create a publisher to publish the camera frames as ROS image messages
    image_pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)

    # Create a CvBridge object to convert between OpenCV images and ROS images
    bridge = CvBridge()

    # Open the camera
    cap = cv2.VideoCapture(0)  # 0 represents the default camera index
 

    # Set the camera frame rate (adjust as needed)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Main loop
    rate = rospy.Rate(30)  # Publish at 30 Hz (adjust as needed)
    while not rospy.is_shutdown():
        # Read a frame from the camera
        ret, frame = cap.read()

        if ret:
            # Convert the OpenCV frame to a ROS image message
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            # Publish the ROS image message
            image_pub.publish(ros_image)

            # Display the frame in a window called "Camera Feed"
            cv2.imshow('Camera Feed', frame)

            # Wait for the 'q' key to be pressed to exit the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Sleep to maintain the desired frame rate
        rate.sleep()

    # Release the camera and close the window
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        camera_feed()
    except rospy.ROSInterruptException:
        pass
