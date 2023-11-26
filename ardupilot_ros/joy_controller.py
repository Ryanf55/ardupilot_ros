import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from pynput import keyboard


class KeyboardJoyPublisher(Node):
    def __init__(self):
        super().__init__("keyboard_joy_publisher")
        self.publisher_ = self.create_publisher(Joy, "/ap/joy", 10)

        self.axes = [0.0, 0.0, 0.0, 0.0]

        # Create a timer to publish the message every 1 second
        self.timer = self.create_timer(1, self.publish_joy)

        # Register the keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        )
        self.listener.start()

    def on_press(self, key):
        if key.char == "w":
            self.axes[3] = 1.0
        elif key.char == "a":
            self.axes[1] = -1.0
        elif key.char == "s":
            self.axes[2] = 1.0
        elif key.char == "d":
            self.axes[0] = 1.0

    def on_release(self, key):
        if key.char == "w":
            self.axes[3] = 0.0
        elif key.char == "a":
            self.axes[1] = 0.0
        elif key.char == "s":
            self.axes[2] = 0.0
        elif key.char == "d":
            self.axes[0] = 0.0

    def publish_joy(self):
        joy_msg = Joy()
        joy_msg.axes = self.axes
        self.publisher_.publish(joy_msg)


def main(args=None):
    rclpy.init(args=args)
    keyboard_joy_publisher = KeyboardJoyPublisher()
    rclpy.spin(keyboard_joy_publisher)
    keyboard_joy_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
