import pygame
import rclpy
from rclpy.node import Node
from xbox_controller_interfaces.msg import ControllerEvent

# Constants for joystick events
class ControllerConstants:
    BUTTONS = {
        0: "A",
        1: "B",
        3: "X",
        4: "Y",
        6: "LB",
        7: "RB",
        10: "SELECT",
        11: "START",
        12: "XBOX",
        13: "LS",
        14: "RS",
    }

    AXES = {
        0: "LX",
        1: "LY",
        2: "RX",
        3: "RY",
        4: "RT",
        5: "LT",
    }

    HAT_DIRECTIONS = {
        (0, 0): "CENTER",
        (-1, 0): "LEFT",
        (1, 0): "RIGHT",
        (0, -1): "DOWN",
        (0, 1): "UP",
    }

class ControllerInputMapper(Node):
    def __init__(self):
        super().__init__('controller_input_mapper')
        pygame.init()
        self.joysticks = []
        self.clock = pygame.time.Clock()
        self.keep_playing = True
        self._initialize_joysticks()

        # Create a single ROS2 publisher
        self.joystick_publisher = self.create_publisher(ControllerEvent, 'joystick/events', 10)

    def _initialize_joysticks(self):
        """Detect and initialize all connected joysticks."""
        for i in range(0, pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            self.joysticks.append(joystick)
            self.get_logger().info(f"Detected joystick: {joystick.get_name()}")

    def publish_event(self, event_type, event_value):
        """Publish a single event to the joystick events topic."""
        msg = ControllerEvent()
        msg.type = event_type
        msg.value = float(event_value)
        self.joystick_publisher.publish(msg)
        self.get_logger().info(f"Published: type={msg.type}, value={msg.value}")

    def process_events(self):
        """Process events and map them to controller inputs."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.keep_playing = False

            # Handle button presses
            if event.type == pygame.JOYBUTTONDOWN:
                button_name = ControllerConstants.BUTTONS.get(event.button, f"unknown button {event.button}")
                self.publish_event(button_name, 1.0)  # Publish 1.0 for button press

            # Handle axis motion
            elif event.type == pygame.JOYAXISMOTION:
                axis_name = ControllerConstants.AXES.get(event.axis, "unknown axis")
                # Invert the Y-axis values
                if event.axis in [1, 3]:
                    event_value = -event.value
                else:
                    event_value = event.value
                self.publish_event(axis_name, event_value)

            # Handle hat (D-pad) motion
            elif event.type == pygame.JOYHATMOTION:
                direction = ControllerConstants.HAT_DIRECTIONS.get(event.value, "unknown direction")
                if direction != "CENTER":
                    self.publish_event(direction, 1.0)

def main():
    rclpy.init()
    controller = ControllerInputMapper()

    try:
        while controller.keep_playing:
            controller.clock.tick(60)
            controller.process_events()
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == "__main__":
    main()
