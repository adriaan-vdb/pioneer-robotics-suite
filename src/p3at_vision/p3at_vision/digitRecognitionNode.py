import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as PILImage
from p3at_interface.msg import ObjectInfo
import easyocr
from std_msgs.msg import Int32
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion
from p3at_interface.msg import ObjectInfo
from p3at_interface.msg import Distance  # Import the custom message type

THRESHOLD = 150

PROXIMITY_THRESHOLD = 3  # After what SLAM map pixel distance are two objects considered different vs. the same

past_markers = []

thresh_distance = [0, 0, 0]  # the distance away of the object from the robot when it is recognised in the camera
lidar_angle = [0, 0, 0]
position = 1  # (0 = left, 1 = centre, 2 = right)

# Define parameters
x_percent = 0.7  # Portion of the width to consider
y_percent = 0.7  # Portion of the height to consider
downsample_factor = 0.4  # Factor to downsample the image // Very inaccurate lower than 0.6 (best is 0.8)
contrast_factor = 1  # Factor to increase contrast
rotation_angle = -15  # Rotation angle in degrees 

class digitRecognitionNode(Node):

    def __init__(self):
        super().__init__('digit_recognition_node')

        self.subscriber = self.create_subscription(Image, 'oak/rgb/image_raw', self.listener_callback, 1) # Real Robot
        # self.subscriber = self.create_subscription(Image, 'p3at/camera/image_raw', self.listener_callback, 1) # Simulation 

        self.publisher = self.create_publisher(Int32, 'digits', 10)

        self.br = CvBridge()

        self.reader = easyocr.Reader(['en'])

        self.image_publisher = self.create_publisher(Image, 'digits_image', 10)
        
        self.object_publisher = self.create_publisher(ObjectInfo, 'markers', 10)

        self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, 'pose', self.pose_callback, 1)

        self.lidar_subscriber = self.create_subscription(Distance, 'obstacle_pos', self.lidar_callback, 1)

        self.x = 0
        self.y = 0
        self.theta = 0
    
    def lidar_callback(self, msg):
        global thresh_distance, lidar_angle
        scale = 7
        thresh_distance = [msg.left_distance * scale, msg.middle_distance * scale, msg.right_distance * scale]
        lidar_angle = [msg.left_angle, msg.middle_angle, msg.right_angle]

    def not_existing(self, x, y):
        if x == None or y == None:
            return False
        # Checks if a marker already exists within the given PROXIMITY_THRESHOLD
        for past in past_markers:
            if abs(past[0] - x) < PROXIMITY_THRESHOLD and abs(past[1] - y) < PROXIMITY_THRESHOLD:
                return False
        return True
    

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


    def rotate_image(self, image, angle):
        # Get the center of the image
        center = (image.shape[1] // 2, image.shape[0] // 2)
        # Get the rotation matrix
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        # Perform the rotation
        rotated_image = cv2.warpAffine(image, rotation_matrix, (image.shape[1], image.shape[0]))
        return rotated_image

    
    def listener_callback(self, msg):
        global position
        # Automatically skip if it's looking at a previously existing marker
        # if not self.not_existing(self.get_position()[0], self.get_position()[1]):
            # self.image_publisher.publish(msg)
            # return
        
        frame = self.br.imgmsg_to_cv2(msg)

        height, width = frame.shape[:2]

        image = frame

        if image is None:
            return
        
        # Center crop the image
        start_x = int((width * (1 - x_percent)) / 2)
        start_y = int((height * (1 - y_percent)) / 2)
        end_x = start_x + int(width * x_percent)
        end_y = start_y + int(height * y_percent)
        cropped_image = image[start_y:end_y, start_x:end_x]

            # Downsample the image
        downsampled_image = cv2.resize(cropped_image, (0, 0), fx=downsample_factor, fy=downsample_factor)

        # Convert the image to grayscale
        gray_image = cv2.cvtColor(downsampled_image, cv2.COLOR_BGR2GRAY)

        # Increase contrast
        contrast_image = cv2.convertScaleAbs(gray_image, alpha=contrast_factor, beta=0)

        # Rotate the images
        rotated_contrast_image = self.rotate_image(contrast_image, rotation_angle)
        rotated_downsampled_image = self.rotate_image(downsampled_image, rotation_angle) # for colour

        images_to_process = [contrast_image] # for rotating
        # images_to_process = [contrast_image] # for greyscale
        # images_to_process = [downsampled_image, rotated_downsampled_image] # for colour

        for final_image in images_to_process:
            # if not self.not_existing(self.get_position()[0], self.get_position()[1]):
            #     # self.image_publisher.publish(msg)
            #     continue
            
            # Use EasyOCR to detect text
            results = self.reader.readtext(final_image, allowlist="0123456789gqlI", text_threshold=0.86, batch_size=4)


            if results:
                detect_one_digit = True

                if detect_one_digit:
                    threshold = 0.01
                    size_threshold_result = [x for x in results if (abs((x[0][0][1] - x[0][2][1])) > threshold * end_y and len(x[1]) == 1)]
                    # print(size_threshold_result)

                    if len(size_threshold_result) != 0:
                        best_result = max(size_threshold_result, key=lambda x: x[2])  # highest confidence
                        bbox, text, confidence = best_result
                        center_x = ((bbox[0][0] + bbox[2][0])/2)/downsample_factor + start_x
                        position = int(center_x // int(width/3))  # sets position appropriately
                        
                        self.get_logger().info(f">>>>>>>>>{position, center_x, int(width/3)}")
                        text = '9' if (text == 'g' or text == 'q') else text
                        text = '1' if (text == 'l' or text == 'I') else text

                        top_left = tuple([int(val) for val in bbox[0]])
                        bottom_right = tuple([int(val) for val in bbox[2]])
                        final_image = cv2.cvtColor(final_image, cv2.COLOR_GRAY2BGR)
                        cv2.rectangle(final_image, top_left, bottom_right, (0, 255, 0), 2)
                        cv2.putText(final_image, text, top_left, cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2)

                        # create and publish object info
                        object_info = ObjectInfo()
                        object_info.x = float(self.get_position()[0])
                        object_info.y = float(self.get_position()[1])
                        object_info.number = int(text)
                        object_info.description = 'number'

                        #if self.not_existing(self.get_position()[0], self.get_position()[1]):
                        #    if (abs(object_info.x) < 10.0 and abs(object_info.y) < 10.0):
                        self.object_publisher.publish(object_info)
                        past_markers.append([self.get_position()[0], self.get_position()[1]])

                        self.get_logger().info("LENGTH: %d" % len(past_markers))

                else:
                    for bbox, text, confidence in results:
                        threshold = 0.1
                        top_left = tuple([int(val) for val in bbox[0]])
                        bottom_right = tuple([int(val) for val in bbox[2]])

                        if (abs((top_left[0] - bottom_right[0]) * (top_left[1] - bottom_right[1])) < threshold * width * threshold * height):
                            continue
                        cv2.rectangle(final_image, top_left, bottom_right, (0, 255, 0), 2)
                        cv2.putText(final_image, text, top_left, cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2)
            self.image_publisher.publish(self.br.cv2_to_imgmsg(final_image))
    

        # # Convert the image to grayscale
        # gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # # Convert the grayscale image to binary black and white
        # _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

        # # Convert the binary image back to normal grayscale format
        # image = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
        # # Use EasyOCR to detect text
        # results = self.reader.readtext(image, allowlist="0123456789gqlI")
        # detect_one_digit = True

        # if not(not results or max([confidence for (_, _, confidence) in results]) < 0.7):

        #     # Draw bounding boxes around detected digits and print the results


        #     if detect_one_digit:
        #         threshold = 0.05
        #         size_threshold_result = [x for x in results if (abs((x[0][0][1] - x[0][2][1])) >  threshold * height)]
        #         # size_threshold_result = [x for x in results if (abs((x[0][0][0] - x[0][1][0]) * (x[0][0][1] - x[0][2][1])) > threshold * width * threshold * height)]
        #         print(size_threshold_result)
        #         if (len(size_threshold_result) != 0):
        #             best_result = max(size_threshold_result, key=lambda x: x[2]) # highest confidence
        #             bbox, text, confidence = best_result
        #             text = '9' if (text == 'g' or text == 'q') else text
        #             text = '1' if (text == 'l' or text == 'I') else text

        #             top_left = tuple([int(val) for val in bbox[0]])
        #             bottom_right = tuple([int(val) for val in bbox[2]])
        #             cv2.rectangle(image, top_left, bottom_right, (0, 255, 0), 2)
        #             cv2.putText(image, text, top_left, cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2)
        #             self.get_logger().info("Digit detected: %s" % text)
        #             msg = Int32()
        #             msg.data = int(text)
        #             self.publisher.publish(msg)

        #     else:
        #         for bbox, text, confidence in results:
        #             threshold = 0.1
        #             top_left = tuple([int(val) for val in bbox[0]])
        #             bottom_right = tuple([int(val) for val in bbox[2]])

        #             if (abs((top_left[0] - bottom_right[0]) * (top_left[1] - bottom_right[1])) < threshold * width * threshold * height):
        #                 continue
        #             cv2.rectangle(image, top_left, bottom_right, (0, 255, 0), 2)
        #             cv2.putText(image, text, top_left, cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2)
        #             print(f"Digit: {text}, Confidence: {confidence}")


        # Display the image with detected digits
        # cv2.imshow("Detected Digits", cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        # cv2.waitKey(1)
        
        # cv2.imshow("Document Scanner", img)
        # cv2.waitKey(1)

        

def main(args=None):
    rclpy.init(args=args)

    image_sub = digitRecognitionNode()

    rclpy.spin(image_sub)

    image_sub.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()




