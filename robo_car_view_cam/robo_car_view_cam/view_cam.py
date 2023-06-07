
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from std_msgs.msg import String, Int8

import cv2 # OpenCV library
import numpy as np
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from sensor_msgs.msg import Image, CompressedImage # Image is the message type
import time
import math

 
 
class ViewCam(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('robo_car_view_cam_node')
            
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(Image, '/cam/image_raw', self.read_camera_callback, 10)
        self.subscription # prevent unused variable warning
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        
        self.fps = 0.0
        self.current_time=time.time()
        self.previous_time = time.time()


    def read_camera_callback(self, msg):
        self.current_time=time.time()
        """
        Callback function.
        """

        # Convert ROS Image message to OpenCV image
        img = self.br.imgmsg_to_cv2(msg, "bgr8")  # current_frame = self.br.compressed_imgmsg_to_cv2(msg)
        # img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # threshold, img_bw = cv2.threshold(img_gray, 150, 255, cv2.THRESH_BINARY)

        # update framerate
        self.fps = 1/(self.current_time-self.previous_time)
        self.previous_time=self.current_time
        
        
        # Display image
        cv2.imshow("camera", img)
        
        # Display the message on the console
        # self.get_logger().info('fps=%d' % (int(self.fps)))

        cv2.waitKey(1)
  
    
  
  
def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    view_cam = ViewCam()

    # Spin the node so the callback function is called.
    rclpy.spin(view_cam)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    view_cam.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
  

if __name__ == '__main__':
    main()