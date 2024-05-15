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

THRESHOLD = 150

class digitRecognitionNode(Node):

    def __init__(self):
        super().__init__('digit_recognition_node')

        self.subscriber = self.create_subscription(Image, 'oak/rgb/image_raw', self.listener_callback, 1)

        self.publisher = self.create_publisher(Int32, 'digits', 10)

        self.br = CvBridge()

        self.reader = easyocr.Reader(['en'])

    
    def listener_callback(self, msg):
        self.get_logger().info("Image recieved!")

        frame = self.br.imgmsg_to_cv2(msg)

        height, width = frame.shape[:2]

        image = frame

        if image is None:
            return

        # Convert the image to grayscale
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Convert the grayscale image to binary black and white
        _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

        # Convert the binary image back to normal grayscale format
        image = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
        # Use EasyOCR to detect text
        results = self.reader.readtext(image, allowlist="0123456789")
        detect_one_digit = True

        if not(not results or max([confidence for (_, _, confidence) in results]) < 0.7):

            # Draw bounding boxes around detected digits and print the results


            if detect_one_digit:
                threshold = 0.15
                size_threshold_result = [x for x in results if (abs((x[0][0][0] - x[0][1][0]) * (x[0][0][1] - x[0][2][1])) > threshold * width * threshold * height)]
                print(size_threshold_result)
                if (len(size_threshold_result) != 0):
                    best_result = max(size_threshold_result, key=lambda x: x[2]) # highest confidence
                    bbox, text, confidence = best_result
                    top_left = tuple([int(val) for val in bbox[0]])
                    bottom_right = tuple([int(val) for val in bbox[2]])
                    cv2.rectangle(image, top_left, bottom_right, (0, 255, 0), 2)
                    cv2.putText(image, text, top_left, cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2)
                    print(f"Digit: {text}, Confidence: {confidence}")
                    self.get_logger().info("Digit detected: %s" % text)
                    msg = Int32()
                    msg.data = int(text)  # Replace with your digit
                    self.publisher.publish(msg)

            else:
                for bbox, text, confidence in results:
                    threshold = 0.1
                    top_left = tuple([int(val) for val in bbox[0]])
                    bottom_right = tuple([int(val) for val in bbox[2]])

                    if (abs((top_left[0] - bottom_right[0]) * (top_left[1] - bottom_right[1])) < threshold * width * threshold * height):
                        continue
                    cv2.rectangle(image, top_left, bottom_right, (0, 255, 0), 2)
                    cv2.putText(image, text, top_left, cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2)
                    print(f"Digit: {text}, Confidence: {confidence}")


        # Display the image with detected digits
        cv2.imshow("Detected Digits", cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        cv2.waitKey(1)

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




