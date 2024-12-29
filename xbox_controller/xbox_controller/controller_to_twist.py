import rclpy
from rclpy.node import Node
from xbox_controller_interfaces.msg import ControllerEvent
from geometry_msgs.msg import Twist
from playsound import playsound

# Controller events
class ControllerEvents:
    # Button Events
    BUTTON_EVENTS = {
        "A", "B", "X", "Y", 
        "LB", "RB", "SELECT", 
        "START", "XBOX", "LS", "RS"
    }

    # Axis Events
    AXIS_EVENTS = {
        "LX", "LY",  # Left Stick
        "RX", "RY",  # Right Stick
        "RT", "LT"   # Triggers
    }

    # Hat Directions
    HAT_DIRECTIONS = {
        "LEFT", "RIGHT", 
        "UP", "DOWN"
    }

# Convert controller events to Twist messages
class ControllerToTwist(Node):
    def __init__(self):
        super().__init__('controller_to_twist')
        self.get_logger().info('Node initialized: controller_to_twist')
        self.subscription = self.create_subscription(
            ControllerEvent,
            'joystick/events',
            self.controller_event_callback,
            10
        )
        self.get_logger().info('Subscribed to topic: joystick/events')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Publishing to topic: cmd_vel')
        self.twist = Twist()

    def controller_event_callback(self, msg):
        self.get_logger().info(f'Received event: type={msg.type}, value={msg.value}')
        
        # Check if the event is a button event
        if msg.type in ControllerEvents.BUTTON_EVENTS:
            self.get_logger().info(f'Button event detected: {msg.type}')
            self._handle_button_event(msg)
        # Check if the event is an axis event
        elif msg.type in ControllerEvents.AXIS_EVENTS:
            self.get_logger().info(f'Axis event detected: {msg.type}')
            self._handle_axis_event(msg)
        # Check if the event is a hat event
        elif msg.type in ControllerEvents.HAT_DIRECTIONS:
            self.get_logger().info(f'Hat event detected: {msg.type}')
            self._handle_hat_event(msg)
        else:
            self.get_logger().info(f'Unknown event type: {msg.type}')

    def _handle_button_event(self, msg):
        # Log button handling (extend with specific logic if needed)
        self.get_logger().info(f'Handling button event: {msg.type}')
        playsound('/home/pi/ros_ws/src/xbox_controller/xbox_controller/7azza9.mp3', False)
        return

    def _handle_axis_event(self, msg):
        # Handle axis events
        if msg.type == "LX":
            # Round to 4 decimal places
            value = round(msg.value, 4)
            # Remove deadzone
            if abs(value) < 0.4:
                value = 0
            # Normalize value between 0.0 and 1.0 (the value now is between 0.4 and 1.0 in absolute value)
            if value != 0:
                value = (abs(value) - 0.4) / 0.6 * value / abs(value)
            
            self.twist.angular.z = float(-value)
            self.get_logger().info(f'Updated angular.z: {self.twist.angular.z}')
        elif msg.type == "RT":
            # Normalize value between 0.0 and 1.0
            value = (msg.value + 1) / 2
            # Remove deadzone
            if value < 0.05:
                value = 0
            self.twist.linear.x = float(value)
            self.get_logger().info(f'Updated linear.x (forward): {self.twist.linear.x}')
        elif msg.type == "LT":
            # Normalize value between 0.0 and 1.0
            value = (msg.value + 1) / 2
            # Remove deadzone
            if value < 0.05:
                value = 0
            self.twist.linear.x = float(-value)
            self.get_logger().info(f'Updated linear.x (backward): {self.twist.linear.x}')
        self.publisher.publish(self.twist)
        self.get_logger().info('Twist message published')

    def _handle_hat_event(self, msg):
        # Log hat handling (extend with specific logic if needed)
        self.get_logger().info(f'Handling hat event: {msg.type}')
        return

def main(args=None):
    rclpy.init(args=args)
    controller_to_twist = ControllerToTwist()
    rclpy.spin(controller_to_twist)
    controller_to_twist.get_logger().info('Shutting down node...')
    controller_to_twist.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
