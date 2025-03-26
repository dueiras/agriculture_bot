import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np

class CameraBasedNavigation(Node):
    def __init__(self):
        super().__init__('visual_navigation')

        # subscribe to the available camera, which is front stereo camera left
        self.image_sub = self.create_subscription(
            Image, '/front_stereo_camera/left/image_raw', self.image_callback, 10)
        
        # subscribe to global planner commands
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_nav', self.cmd_callback, 10)

        # publisher for control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # publisher for segmentation
        self.img_pub = self.create_publisher(Image, '/visual_planner/segmentation', 10)

        self.bridge = CvBridge()

        self.global_x = 0.1
        self.global_yaw = 0.0
        self.max_z = 0.8
        self.fixed_x_vel = 0.35

    def cmd_callback(self, msg):
        """
        Checks the global planner output command
        If the visual local planner identifies an obstacle
        that the global_planner will not avoid, then the
        local planner overides the global
        """
        self.global_x = msg.linear.x
        self.global_yaw = msg.angular.z

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

        # split left and right regions
        # and sum the bottom half of pixel intensities
        height, width = thresh.shape
        left_region = np.sum(thresh[height//2:, :width//2])  
        right_region = np.sum(thresh[height//2:, width//2:])

        twist = Twist()

        if(right_region > 0 and left_region > 0):
            z = np.log(right_region/left_region)
            if(z >= self.max_z):
                z = self.max_z
            elif(z <= -self.max_z):
                z = -self.max_z

            if((abs(z)>abs(self.global_yaw))):
                self.get_logger().info(f"Possible Obstacle detected, sending angular vel: {z}")
                twist.linear.x = self.fixed_x_vel
                twist.angular.z = z           

                # publish velocity
                self.cmd_vel_pub.publish(twist)

        # show the processed image
        img_msg = self.bridge.cv2_to_imgmsg(thresh, encoding="mono8")
        self.img_pub.publish(img_msg)
        #cv2.imshow("Processed Image", thresh[height//2:, :])
        #cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraBasedNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
