import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as PILImage
from p3at_vision.msg import ObjectInfo

THRESHOLD = 150

class imageRecognitionNode(Node):

    def __init__(self):
        super().__init__('image_recognition_node')

        self.subscriber = self.create_subscription(Image, 'oak/rgb/image_raw', self.listener_callback, 1)

        self.publisher = self.create_publisher(Image, 'photos_taken', 10)

        self.object_publisher = self.create_publisher(ObjectInfo, 'identified_objects', 10)

        self.br = CvBridge()

    
    def take_photo(self, msg):
        self.publisher.publish(msg)
        self.get_logger().info("Photo Taken")
    
    def listener_callback(self, msg):
        
        self.get_logger().info("Image recieved!")

        frame = self.br.imgmsg_to_cv2(msg)

        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        photo_taken = False # track whether a photo has been taken

        # HIGHLIGHT YELLOW OBJECTS
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        mask = cv2.inRange(hsvImage, lower_yellow, upper_yellow)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w < THRESHOLD and h < THRESHOLD:
                continue
            
            if not photo_taken:
                self.take_photo(msg)
                photo_taken = True
            
            # create and publish object info
            object_info = ObjectInfo()
            object_info.x = x
            object_info.y = y
            object_info.color = 'yellow'
            self.object_publisher.publish(object_info)

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
            
            if not photo_taken:
                self.take_photo(msg)
                photo_taken = True
            
            # create and publish object info
            object_info = ObjectInfo()
            object_info.x = x
            object_info.y = y
            object_info.color = 'red' 
            self.object_publisher.publish(object_info)

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)

        cv2.imshow("camera", frame)

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    image_sub = imageRecognitionNode()

    rclpy.spin(image_sub)

    image_sub.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()




        

    

