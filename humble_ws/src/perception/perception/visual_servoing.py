import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np

class CameraBasedNavigation(Node):
    def __init__(self):
        super().__init__('visual_servoing')

        # subscribe to the available camera, which is front stereo camera left
        self.image_sub = self.create_subscription(
            Image, '/front_stereo_camera/left/image_raw', self.image_callback, 10)

        # publisher for control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        """
        Process image by doing color segmentation
        and publish movement commands according to 
        intensities on the left and right of the image
        """
        try:
            # convert ROS Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # image color segmentation 
        # values tuned using the hsv_threshold script
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([22, 43, 1])  
        upper = np.array([100, 157, 184])
        thresh = cv2.inRange(hsv, lower, upper)
        #thresh = cv2.bitwise_not(mask)  # Invert mask so brown is white (navigable)

        # split left and right regions
        # and sum the bottom half of pixel intensities
        height, width = thresh.shape
        left_region = np.sum(thresh[height//2:, :width//2])  
        right_region = np.sum(thresh[height//2:, width//2:])

        twist = Twist()
        twist.linear.x = 0.5  # always move forward

        # this value was tuned by trial and error
        if left_region > right_region:
            twist.angular.z = -0.4  # Turn right
        else:
            twist.angular.z = 0.4  # Turn left

        # publish velocity
        self.cmd_vel_pub.publish(twist)

        # show the processed image
        cv2.imshow("Processed Image", thresh)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraBasedNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
