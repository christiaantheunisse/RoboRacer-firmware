import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
from typing import Optional, Literal, TypeAlias
import time

from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from racing_bot_interfaces.msg import BicycleControl

# ros2 topic pub /goal_velocity geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"


class PIDcontroller:
    """Implements a simple PID controller"""

    def __init__(self, Kp: float, Ki: float = 0.0, Kd: float = 0.0) -> None:
        """
        Arguments:
            Kp -- proportional control gain of the PID controller
            Ki -- integral control gain of the PID controller
            Kd -- derivative control gain of the PID controller

        Return:
            None
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self._last_error: Optional[float] = None
        self._accumulated_error: float = 0
        self._last_time: Optional[float] = None

    def __call__(self, *args, **kwargs):
        """Just call the update method"""
        return self.update(*args, **kwargs)

    def update(self, current: float, target: float, stamp: Optional[float] = None) -> float:
        """Apply the PID control

        Arguments:
            current -- the current state
            target -- the target state
            stamp -- the time belonging to the value. If not provided, the system clock is used.

        Return:
            PID output
        """
        stamp = time.time() if stamp is None else stamp
        dt = stamp - self._last_time if self._last_time is not None else 0

        # Proportional part
        error = target - current
        pout = self.Kp * error

        # Integral part
        self._accumulated_error += error * dt
        iout = self.Ki * self._accumulated_error

        # Derivative part
        derror = (error - self._last_error) / dt if self._last_error is not None and dt > 0 else 0
        dout = self.Kd * derror

        return pout + dout + iout


class GeneralControlNode(Node):
    """
    Baseclass for the control nodes
    """

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter("odometry_topic", "odometry/filtered")
        self.declare_parameter("motor_topic", "cmd_motor")

        self.declare_parameter("update_rate", 30.0)
        self.declare_parameter("linear_vel_PID_params", [5.0, 0.0, 0.0])
        self.declare_parameter("angular_vel_PID_params", [0.2, 0.0, 0.1])

        self.odom_topic = self.get_parameter("odometry_topic").get_parameter_value().string_value
        self.motor_topic = self.get_parameter("motor_topic").get_parameter_value().string_value

        self.update_rate = self.get_parameter("update_rate").get_parameter_value().double_value
        self.linear_pid_params = self.get_parameter("linear_vel_PID_params").get_parameter_value().double_array_value
        self.angular_pid_params = self.get_parameter("angular_vel_PID_params").get_parameter_value().double_array_value
        assert (
            len(self.linear_pid_params) == 1 or len(self.linear_pid_params) == 3
        ), "Provide 1 or 3 linear velocity PID parameters: [Kp] or [Kp, Ki, Kd]"
        assert (
            len(self.angular_pid_params) == 1 or len(self.angular_pid_params) == 3
        ), "Provide 1 or 3 angular velocity PID parameters: [Kp] or [Kp, Ki, Kd]"

        self.linear_pid_controller = PIDcontroller(*self.linear_pid_params)
        self.angular_pid_controller = PIDcontroller(*self.angular_pid_params)

        self.velocity: TwistStamped = TwistStamped()

        self._linear_pwm: float = 0.0
        self._angular_pwm: float = 0.0

        self.create_subscription(Odometry, self.odom_topic, self.odometry_callback, 3)
        self.motor_publisher = self.create_publisher(Int16MultiArray, self.motor_topic, 5)

        self.create_timer(1 / self.update_rate, self.apply_control)

    def odometry_callback(self, msg: Odometry) -> None:
        # self.get_logger().info("Odometry message received", throttle_duration_sec=2)
        self.velocity = TwistStamped(header=msg.header, twist=msg.twist.twist)

    def apply_control(self) -> None:
        """The control loop function"""
        raise NotImplementedError

    def apply_pid_control(self, current: TwistStamped, target: TwistStamped) -> tuple[float, float]:
        """Applies linear velocity control

        Arguments:
            current -- the actual velocity as a TwistStamped message
            target -- the target velocity as a TwistStamped message

        Returns:
            tuple with (linear PWM value, angular pwm value) as a float in the range [0, 1]
        """
        stamp_float = Time.from_msg(current.header.stamp).nanoseconds * 1e-9

        current_lin = current.twist.linear.x
        target_lin = target.twist.linear.x
        linear_pwm = self.linear_pid_controller(current_lin, target_lin, stamp_float)

        current_ang = current.twist.angular.z
        target_ang = target.twist.angular.z
        angular_pwm = self.angular_pid_controller(current_ang, target_ang, stamp_float)

        return linear_pwm, angular_pwm

    def calculate_wheel_pwms(self, linear_pwm: float, angular_pwm: float) -> tuple[float, float]:
        """Calculates the angular pwm value for each wheel. The linear pwm is reduced up to make sure that the angular
        pwm can be fully applied. However, the angular pwm is clipped at a certain threshold.

        Arguments:
            linear_pwm -- required pwm signal for the linear velocity
            angular_pwm -- required pwm signal for the angular velocity

        Returns:
            tuple with (left wheel pwm, right wheel pwm) in the range of [0, 1]
        """
        linear_pwm = np.clip(linear_pwm, 0, 1 - np.abs(angular_pwm))

        return linear_pwm - angular_pwm, linear_pwm + angular_pwm

    def publish_pwms(self, left_pwm: float, right_pwm: float) -> None:
        """Publishes the pwm values after converting them to the range [0, 255] (uint8)

        Arguments:
            left_pwm -- pwm value of the left motor in the range [0, 1]
            right_pwm -- pwm value of the right motor in the range [0, 1]

        Returns:
            Nothing
        """
        left_pwm255 = round(left_pwm * 255)
        right_pwm255 = round(right_pwm * 255)
        motor_msg = Int16MultiArray(data=[left_pwm255, right_pwm255, 0, 0])
        self.motor_publisher.publish(motor_msg)


class VelocityControlNode(GeneralControlNode):
    """
    This node takes a TwistStamped as input from which it takes the linear and angular velocity (twist.linear.x and
    twist.angular.z) and uses this as the target values for the PID controllers
    """

    def __init__(self):
        super().__init__("velocity_control_node")

        self.declare_parameter("velocity_control_topic", "goal_velocity")
        self.declare_parameter("angular_velocity_limit", 0.2)

        self.velocity_control_topic = self.get_parameter("velocity_control_topic").get_parameter_value().string_value
        self.angular_pwm_limit = self.get_parameter("angular_velocity_limit").get_parameter_value().double_value
        assert 0.0 < self.angular_pwm_limit <= 1.0, f"angular_pwm_limit should be in the range (0., 1.]"

        self.target_velocity: TwistStamped = TwistStamped()

        self.create_subscription(TwistStamped, self.velocity_control_topic, self.velocity_control_callback, 3)

    def velocity_control_callback(self, msg: TwistStamped) -> None:
        # self.get_logger().info("TwistStamped message received", throttle_duration_sec=2)
        self.target_velocity = msg

    def apply_control(self) -> None:
        """The control loop function"""
        current, target = self.velocity, self.target_velocity

        # calculate the acceleration with a PID controller and update the velocities
        linear_pwm_acc, angular_pwm_acc = self.apply_pid_control(current, target)
        self._linear_pwm += linear_pwm_acc / self.update_rate
        self._angular_pwm += angular_pwm_acc / self.update_rate
        # the pwm can never be larger than 1 (i.e. 100%) and the angular pwm has an addition limit
        self._linear_pwm = np.clip(self._linear_pwm, 0, 1)
        self._angular_pwm = np.clip(self._angular_pwm, -self.angular_pwm_limit, self.angular_pwm_limit)

        left_pwm, right_pwm = self.calculate_wheel_pwms(self._linear_pwm, self._angular_pwm)

        self.publish_pwms(left_pwm, right_pwm)

        # self.get_logger().info(
        #     f"Control applied:"
        #     f"\n\t{[current.twist.linear.x, target.twist.linear.x]=}"
        #     f"\n\t{[current.twist.angular.z, target.twist.angular.z]=}"
        #     f"\n\t{[linear_pwm_acc, angular_pwm_acc]=}"
        #     f"\n\t{[self._linear_pwm, self._angular_pwm]=}"
        #     f"\n\t{[left_pwm255, right_pwm255]=}",
        # )


class BicycleControlNode(GeneralControlNode):
    """This node takes a BicycleControl message as input from which it takes the linear velocity and steering angle.
    These values are use to control a differential drive robot."""

    def __init__(self):
        super().__init__("bicycle_control_node")
        self.declare_parameter("bicycle_control_topic", "bicycle_control")
        self.declare_parameter("wheel_base_length", 0.1)  # a fictitious value

        self.bicycle_control_topic = self.get_parameter("bicycle_control_topic").get_parameter_value().string_value
        self.wheel_base_length = self.get_parameter("wheel_base_length").get_parameter_value().double_value

        self.bicycle_control: BicycleControl = BicycleControl()

        self.create_subscription(BicycleControl, self.bicycle_control_topic, self.bicycle_control_callback, 3)

    def bicycle_control_callback(self, msg: BicycleControl) -> None:
        """The bicycle control messages callback"""
        self.bicycle_control = msg

    def apply_control(self) -> None:
        """The control loop function"""
        current_twist, control = self.velocity, self.bicycle_control

        target_twist = TwistStamped()
        target_twist.twist.linear.x = control.velocity.data
        target_twist.twist.angular.z = self.calc_angular_velocity(control.steering_angle.data, current_twist.twist.linear.x)

        # calculate the acceleration with a PID controller and update the velocities
        linear_pwm_acc, angular_pwm_acc = self.apply_pid_control(current_twist, target_twist)
        self._linear_pwm += linear_pwm_acc / self.update_rate
        self._angular_pwm += angular_pwm_acc / self.update_rate
        # the pwm can never be larger than 1 (i.e. 100%)
        self._linear_pwm = np.clip(self._linear_pwm, 0, 1)
        self._angular_pwm = np.clip(self._angular_pwm, -0.5, 0.5)
        self.get_logger().info(f"{self._linear_pwm=} {self._angular_pwm=}")

        left_pwm, right_pwm = self.calculate_wheel_pwms(self._linear_pwm, self._angular_pwm)
        self.get_logger().info(f"{left_pwm=} {right_pwm=}")

        self.publish_pwms(left_pwm, right_pwm)

    def calc_angular_velocity(self, steering_angle: float, velocity: float) -> float:
        """This function calculates the angular velocity for a certain steering angle and linear velocity. In case of
        a differential driven robot, this is only a fictitious steering angle. Nevertheless, trajectory following
        approaches often rely on the steering angle since this better relates to the position.

        Arguments:
            steering_angle -- the steering angle of the car / mobile robot
            velocity -- the velocity of the robot

        Returns:
            the angular velocity
        """
        # the steering angle and (fictitious) wheel base length give the corner radius, so you actual control the
        #  corner radius.
        if steering_angle == 0.:
            return 0.
        corner_radius = self.wheel_base_length / np.sin(steering_angle)
        angular_velocity = velocity / corner_radius
        return angular_velocity
