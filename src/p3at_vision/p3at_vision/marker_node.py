import rclpy
from rclpy.node import Node
from p3at_interface.msg import ObjectInfo as Marker  # Import the correct custom message
from visualization_msgs.msg import Marker as RVizMarker
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Duration  # Import the correct Duration message type

class MarkerNode(Node):
    def __init__(self):
        super().__init__('marker_node')
        self.subscription = self.create_subscription(
            Marker,  # Use the custom message type
            'markers',  # Ensure the topic matches the publisher's topic
            self.listener_callback,
            10)
        self.markers = []
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

    def listener_callback(self, msg):
        marker = RVizMarker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.id = len(self.markers)

        marker.pose.position.x = msg.x
        marker.pose.position.y = msg.y
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        if msg.description in ["yellow", "red"]:
            marker.ns = "basic_shapes"
            marker.type = RVizMarker.CUBE
            self.get_logger().info('\n------ Obstacle Added ------')
            marker.action = RVizMarker.ADD

            marker.pose.position.z = 0.5
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 1.0

            if (msg.description == "yellow"):
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
        
        if msg.description in [str(i) for i in range(10)]:
            marker.ns = "basic_shapes"
            marker.type = RVizMarker.CUBE
            self.get_logger().info('\n------ Digit Added ------')
            marker.action = RVizMarker.ADD
            
            marker.pose.position.z = 0.5
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 1.0

            marker.lifetime = Duration()  # Correct the Duration type
            self.markers.append(marker)  # Add the marker to the list
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0


            text_marker = RVizMarker()
            text_marker.header.frame_id = "odom"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "digits"
            text_marker.id = len(self.markers)
            text_marker.type = RVizMarker.TEXT_VIEW_FACING
            text_marker.action = RVizMarker.ADD

            text_marker.pose.position.x = msg.x
            text_marker.pose.position.y = msg.y
            text_marker.pose.position.z = 1.2  # Position text above the cube
            text_marker.pose.orientation.x = 0.0
            text_marker.pose.orientation.y = 0.0
            text_marker.pose.orientation.z = 0.0
            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = 0.4  # Height of the text
            text_marker.color.r = 0.0
            text_marker.color.g = 1.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0

            text_marker.text = msg.description

            self.markers.append(text_marker)  # Add the text marker to the list


        # Set the lifetime correctly
        marker.lifetime = Duration()  # Correct the Duration type

        self.markers.append(marker)  # Add the marker to the list

        marker_array = MarkerArray()
        marker_array.markers = self.markers

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
