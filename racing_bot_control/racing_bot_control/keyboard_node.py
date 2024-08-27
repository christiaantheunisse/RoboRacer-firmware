import rclpy
import rclpy.callback_groups
import rclpy.executors
from rclpy.node import Node
import curses
import numpy as np
from threading import Thread, Lock

from geometry_msgs.msg import TwistStamped
from racing_bot_interfaces.msg import BicycleControl

mutex = Lock()


class KeyboardInputNode(Node):
    """
    Velocity PID controllers
    """

    def __init__(self):
        super().__init__("keyboard_control_node")

        self.declare_parameter("velocity_control_topic", "goal_velocity")
        self.declare_parameter("bicycle_control_topic", "bicycle_control")
        self.declare_parameter("update_rate", 10.0)
        self.declare_parameter("max_linear_velocity", 0.5)
        self.declare_parameter("max_angular_velocity", 1.5)
        self.declare_parameter("max_steering_angle", 0.5)

        self.velocity_control_topic = self.get_parameter("velocity_control_topic").get_parameter_value().string_value
        self.bicycle_control_topic = self.get_parameter("bicycle_control_topic").get_parameter_value().string_value
        self.update_rate = self.get_parameter("update_rate").get_parameter_value().double_value
        self.max_linear_velocity = self.get_parameter("max_linear_velocity").get_parameter_value().double_value
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter("max_steering_angle").get_parameter_value().double_value

        parallel_cb_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.velocity_control_publisher = self.create_publisher(TwistStamped, self.velocity_control_topic, 5)
        self.bicycle_control_publisher = self.create_publisher(BicycleControl, self.bicycle_control_topic, 5)
        self.create_timer(1 / self.update_rate, self.publish_velocity, callback_group=parallel_cb_group)
        self.loop_func_timer = self.create_timer(1, self.input_loop, callback_group=parallel_cb_group)

        self._linear: float = 0.0
        self._angular: float = 0.0

    def calc_steering_angle(self, angular_velocity: float) -> float:
        return angular_velocity / self.max_angular_velocity * self.max_steering_angle

    def publish_velocity(self) -> None:
        """Regularly publish the goal velocities"""
        with mutex:
            linear = float(self._linear)
            angular = float(self._angular)

        twist = TwistStamped()
        twist.twist.linear.x = linear
        twist.twist.angular.z = angular
        self.velocity_control_publisher.publish(twist)

        bicycle_control = BicycleControl()
        bicycle_control.velocity.data = linear
        bicycle_control.steering_angle.data = self.calc_steering_angle(angular)
        self.bicycle_control_publisher.publish(bicycle_control)

    def input_loop(self) -> None:
        """Listen for curses inputs"""
        self.loop_func_timer.cancel()

        def main(stdscr):
            stdscr.clear()  # Clear screen
            curses.curs_set(0)  # Turn off cursor blinking
            stdscr.keypad(True)  # Enable special keys (like arrow keys)
            stdscr.addstr(
                0,
                0,
                "Press the arrow keys to drive and use the spacebar brake. "
                "The linear and angular velocity values are send as a velocity message.",
            )  # Print instructions
            stdscr.addstr(1, 0, "-" * 50)
            stdscr.refresh()

            linear, angular = 0, 0
            while True:
                key = stdscr.getch()  # Capture a keypress
                stdscr.clear()  # Clear previous input

                stdscr.addstr(
                    0,
                    0,
                    "Press the arrow keys to drive and use the spacebar to brake. "
                    "The linear and angular velocity values are send as a velocity message. "
                    "Press 'q' to exit.",
                )  # Print instructions
                stdscr.addstr(1, 0, "-" * 50)

                # If the key is 'q', exit the loop
                if key == ord("q"):
                    break

                # Handle arrow keys
                if key == curses.KEY_UP:
                    linear = linear + 0.25 * self.max_linear_velocity
                elif key == curses.KEY_DOWN:
                    linear = linear - 0.25 * self.max_linear_velocity
                elif key == curses.KEY_LEFT:
                    angular = angular + 0.25 * self.max_angular_velocity
                elif key == curses.KEY_RIGHT:
                    angular = angular - 0.25 * self.max_angular_velocity
                elif key == ord(" "):
                    stdscr.addstr(3, 0, "Full brake!")
                    linear = 0
                    angular = 0

                linear = np.clip(linear, 0, self.max_linear_velocity)
                angular = np.clip(angular, -self.max_angular_velocity, self.max_angular_velocity)
                with mutex:
                    self._linear = linear
                    self._angular = angular

                stdscr.addstr(
                    2,
                    0,
                    f"linear = {self._linear:.2f}\tangular = {self._angular:.2f}"
                    f"\tsteering angle = {self.calc_steering_angle(self._angular)}",
                )
                # Refresh the screen to show changes
                stdscr.refresh()

            # Clean up before exiting
            stdscr.clear()
            stdscr.refresh()

        # Initialize curses
        curses.wrapper(main)


def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    keyboard_input_node = KeyboardInputNode()
    executor.add_node(keyboard_input_node)
    executor.spin()
    # rclpy.spin(keyboard_input_node)
    keyboard_input_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
