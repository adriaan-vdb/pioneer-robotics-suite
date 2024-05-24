import rclpy
from rclpy.node import Node
from p3at_interface.msg import ObjectInfo as Marker  # Import the correct custom message
from visualization_msgs.msg import Marker as RVizMarker
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Duration  # Import the correct Duration message type
import tkinter as tk
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion

class gui(Node):
    def __init__(self):
        super().__init__('gui')
        self.subscription = self.create_subscription(
            Marker,  # Use the custom message type
            'markers',  # Ensure the topic matches the publisher's topic
            self.listener_callback,
            10)
        
        self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, 'pose', self.pose_callback, 1)
        self.x = 0
        self.y = 0
        self.theta = 0
        
        # Create the main window
        self.root = tk.Tk()
        self.root.title("Robot Status Display")

        # Set dark mode background color
        self.root.configure(bg='#2e2e2e')

        # Fonts
        font_large = ('Helvetica', 18)

        # Colors
        bg_color = '#2e2e2e'
        text_color = 'white'

        # Create labels list to store label widgets
        self.labels = []

        # Define initial status data
        initial_status_data = [
            {'label': 'Pose', 'value': 'x=0.00, y=0.00, θ=0.00', 'color': text_color},
            {'label': 'Mode', 'value': 'Manual', 'color': text_color},
            {'label': 'Task Mode', 'value': 'Navigation', 'color': text_color},
            {'label': 'Emergency Stop', 'value': 'Deactivated', 'color': text_color},
            {'label': 'Obstacle Close', 'value': 'No', 'color': text_color},
            {'label': 'Message', 'value': '', 'color': text_color}  # New label for messages
        ]

        # Create labels dynamically based on initial_status_data
        for status in initial_status_data:
            label = tk.Label(self.root, font=font_large, bg=bg_color, fg=status['color'])
            label.pack(pady=10)
            self.labels.append(label)

        # Add some padding around the window
        self.root.geometry('400x300')
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

        # Call the function to update the status for the first time
        self.update_status()

        # Start the main event loop
        self.root.mainloop()

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
        self.get_logger().info("POSE CHANGED")
        self.get_logger().info("%f, %f, %f" % (self.x, self.y, self.theta))
        self.update_status()

    def listener_callback(self, msg):
        # Update the last label with the received message
        self.labels[-1].config(text=f"Message: {msg.description}")

    # Function to update the GUI with dummy data
    def update_status(self):
        self.get_logger().info("UPDATING GUI")
        # Dummy robot status
        pose = {'x': self.x, 'y': self.y, 'orientation': self.theta}
        mode = "Manual"
        task_mode = "Navigation"
        emergency_stop = False
        obstacle_close = False

        # Update the labels with the dummy data
        status_data = [
            {'label': 'Pose', 'value': f"x={pose['x']:.2f}, y={pose['y']:.2f}, θ={pose['orientation']:.2f}", 'color': 'white'},
            {'label': 'Mode', 'value': mode, 'color': '#5993f0'},
            {'label': 'Task Mode', 'value': task_mode, 'color': '#89d977'},
            {'label': 'Emergency Stop', 'value': 'Activated' if emergency_stop else 'Deactivated', 'color': '#de5b5b'},
            {'label': 'Obstacle Close', 'value': 'Yes' if obstacle_close else 'No', 'color': '#b96eeb'},
            {'label': 'Message', 'value': '', 'color': '#b96eeb'}  # Update the message label
        ]

        for i, status in enumerate(status_data):
            self.labels[i].config(text=f"{status['label']}: {status['value']}", fg=status['color'])

def main(args=None):
    rclpy.init(args=args)
    node = gui()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()