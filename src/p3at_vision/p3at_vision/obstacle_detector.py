import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from p3at_interface.msg import Distance  # Import the custom message type
import numpy as np
from math import radians


# Need to define a cusom message in: my_msgs/msg/Distance.msg:
# float32 distance
# float32 angle

# Can subscribe to the obstance_distance topic to get the distance and angle to the nearest obstacle in the camera's field of view


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(Distance, 'obstacle_pos', 10)
        self.subscription  # prevent unused variable warning
        self.camera_fov = radians(60)  # 60 degrees in radians - camera fov

    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = np.array(msg.ranges)

        # Calculate the indices for the camera's field of view
        fov_start_angle = - self.camera_fov / 2
        fov_end_angle = self.camera_fov / 2
        fov_start_index = int((fov_start_angle - angle_min) / angle_increment)
        fov_end_index = int((fov_end_angle - angle_min) / angle_increment)

        # Publish the distance and angle information
        distance_msg = Distance()

        for i in range(3):
            # Extract the portion of the scan data within the camera's field of view
            start = int(fov_start_index + i * (fov_end_index - fov_start_index)/3)
            end = min(int(start + (fov_end_index - fov_start_index)/3), len(msg.ranges))
            fov_ranges = ranges[start:end]

            # Filter out invalid ranges
            valid_ranges = fov_ranges[np.isfinite(fov_ranges)]

            if len(valid_ranges) == 0:
                # self.get_logger().info('No valid ranges in field of view')
                return

            # Find the minimum distance in the field of view
            valid_ranges = np.where(valid_ranges > 0.1, valid_ranges, 10000000)
            if len(valid_ranges) == 0:
                # self.get_logger().info('No valid ranges after filtering')
                return
            min_distance = np.min(valid_ranges)

            if min_distance < 5.0:
                min_distance_index = np.argmin(valid_ranges) + i * int((fov_end_index - fov_start_index)/3)
                angle = (fov_start_index + min_distance_index) * angle_increment + angle_min

                # self.get_logger().info(f'Obstacle detected at distance: {min_distance}m, angle: {np.degrees(angle):.2f} degrees')

                match i:
                    case 0:
                        distance_msg.right_distance = float(min_distance)
                        distance_msg.right_angle = float(angle)
                    case 1:
                        distance_msg.middle_distance = float(min_distance)
                        distance_msg.middle_angle = float(angle)
                    case 2:
                        distance_msg.left_distance = float(min_distance)
                        distance_msg.left_angle = float(angle)
            else:
                match i:
                    case 0:
                        distance_msg.right_distance = -1.0
                        distance_msg.right_angle = -1.0
                    case 1:
                        distance_msg.middle_distance = -1.0
                        distance_msg.middle_angle = -1.0
                    case 2:
                        distance_msg.left_distance = -1.0
                        distance_msg.left_angle = -1.0
            
        self.publisher.publish(distance_msg)
            
def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetectorNode()
    rclpy.spin(obstacle_detector)
    obstacle_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
