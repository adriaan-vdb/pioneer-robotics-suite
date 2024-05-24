import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as PILImage
from p3at_interface.msg import ObjectInfo
from p3at_interface.msg import Distance  # Import the custom message type
from datetime import datetime
import os

THRESHOLD = 100

PROXIMITY_THRESHOLD = 3  # After what SLAM map pixel distance are two objects considered different vs. the same

past_markers = []

thresh_distance = [0, 0, 0]  # the distance away of the object from the robot when it is recognised in the camera
lidar_angle = [0, 0, 0]
position = 1  # (0 = left, 1 = centre, 2 = right)
image_directory = False

class imageRecognitionNode(Node):

    def __init__(self):
        super().__init__('image_recognition_node')

        self.subscriber = self.create_subscription(Image, 'oak/rgb/image_raw', self.listener_callback, 1)
        # self.subscriber = self.create_subscription(Image, 'p3at/camera/image_raw', self.listener_callback, 1) # Simulation 
        
        self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, 'pose', self.pose_callback, 1)

        self.publisher = self.create_publisher(Image, 'photos_taken', 10)

        self.object_publisher = self.create_publisher(ObjectInfo, 'markers', 10)

        self.image_publisher = self.create_publisher(Image, 'red_yellow_image', 10)

        self.lidar_subscriber = self.create_subscription(Distance, 'obstacle_pos', self.lidar_callback, 1)

        self.br = CvBridge()

        self.x = 0
        self.y = 0
        self.theta = 0
    
    def lidar_callback(self, msg):
        global thresh_distance, lidar_angle
        scale = 7
        thresh_distance = [msg.left_distance * scale, msg.middle_distance * scale, msg.right_distance * scale]
        lidar_angle = [msg.left_angle, msg.middle_angle, msg.right_angle]
        # thresh_distance = msg.distance * 10 # ADJUSTTTTTTTTTTTTTTTTTTTTTTT
        # lidar_angle = msg.angle
    
    def not_existing(self, x, y):
        if x == None or y == None:
            return False
        # Checks if a marker already exists within the given PROXIMITY_THRESHOLD
        for past in past_markers:
            if abs(past[0] - x) < PROXIMITY_THRESHOLD and abs(past[1] - y) < PROXIMITY_THRESHOLD:
                return False
        return True

    def take_photo(self, msg, x, y):
        global image_directory
        if not image_directory:
            # Get the current date and time
            now = datetime.now()
            # Format it as a string
            dir_name = "Image_Folder_" + now.strftime("%Y-%m-%d_%H-%M-%S")
            # Create the directory
            os.makedirs(os.path.join("/workspaces/unity_ros2_ws/SavedImages",dir_name))
            image_directory = "/workspaces/unity_ros2_ws/SavedImages/" + dir_name
        self.publisher.publish(msg)
        image = self.br.imgmsg_to_cv2(msg)
        now = datetime.now()
        image_name = now.strftime("%Y-%m-%d_%H-%M-%S")
        cv2.imwrite(os.path.join(image_directory, image_name + 'pos_' + str(int(x)) + '_' + str(int(y)) + '.jpg'), image)
        self.get_logger().info("Photo Taken")

    
    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        # Converting quaternion to RPY
        roll, pitch, yaw = euler_from_quaternion([x, y, z, w])
        self.theta = yaw
    
    def get_position(self):
        # Determine position of object based on pose (position + orientation) and lidar NOT YET
        if thresh_distance[position] == -1:
            return None, None
        x = self.x + thresh_distance[position] * np.cos(self.theta + lidar_angle[position])
        y = self.y + thresh_distance[position] * np.sin(self.theta + lidar_angle[position])
        return x,y
    
    def listener_callback(self, msg):
        global position
        # Automatically skip if it's looking at a previously existing marker
        # if not self.not_existing(self.get_position()[0], self.get_position()[1]):
        #     self.image_publisher.publish(msg)
        #     return

        frame = self.br.imgmsg_to_cv2(msg)
        height, width, channels = frame.shape

        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        photo_taken = False # track whether a photo has been taken

        # HIGHLIGHT YELLOW OBJECTS
        lower_yellow = np.array([20, 120, 80])
        upper_yellow = np.array([25, 255, 255])
        mask = cv2.inRange(hsvImage, lower_yellow, upper_yellow)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w < THRESHOLD and h < THRESHOLD:
                continue
            
            center_x = x + w/2
            position = int(center_x // int(width/3))  # sets position appropriately
            
            # create and publish object info
            if self.get_position()[0] == None:
                continue
            object_info = ObjectInfo()
            object_info.x = float(self.get_position()[0])
            object_info.y = float(self.get_position()[1])
            object_info.description = 'yellow'
            object_info.number = -1
            if self.not_existing(self.get_position()[0], self.get_position()[1]):
                if (abs(object_info.x) < 10.0 and abs(object_info.y) < 10.0):
                    self.object_publisher.publish(object_info)
                    past_markers.append([self.get_position()[0], self.get_position()[1]])
                
                    if not photo_taken:
                        self.take_photo(msg, self.get_position()[0], self.get_position()[1])
                        photo_taken = True

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 3)
        
        # Define lower and upper bounds for red hue range
        lower_red1 = np.array([0, 160, 50])    # Lower bound for red hue (0 to 10)
        upper_red1 = np.array([5, 255, 255])   # Upper bound for red hue
        lower_red2 = np.array([175, 160, 50])  # Lower bound for red hue (350 to 360)
        upper_red2 = np.array([180, 255, 255])  # Upper bound for red hue
        red_mask1 = cv2.inRange(hsvImage, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsvImage, lower_red2, upper_red2)
        mask = cv2.bitwise_or(red_mask1, red_mask2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w < THRESHOLD and h < THRESHOLD:
                continue

            center_x = x + w/2
            position = int(center_x // int(width/3))  # sets position appropriately
            
            # create and publish object info
            if self.get_position()[0] == None:
                continue
            object_info = ObjectInfo()
            object_info.x = float(self.get_position()[0])
            object_info.y = float(self.get_position()[1])
            object_info.description = 'red'
            object_info.number = -1
            if self.not_existing(self.get_position()[0], self.get_position()[1]):
                if (abs(object_info.x) < 10.0 and abs(object_info.y) < 10.0):
                    self.object_publisher.publish(object_info)
                    past_markers.append([self.get_position()[0], self.get_position()[1]])
                
                    if not photo_taken:
                        self.take_photo(msg, self.get_position()[0], self.get_position()[1])
                        photo_taken = True

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)

        # cv2.imshow("camera", frame)
        self.image_publisher.publish(self.br.cv2_to_imgmsg(frame))
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    image_sub = imageRecognitionNode()

    rclpy.spin(image_sub)

    image_sub.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()




        

    

