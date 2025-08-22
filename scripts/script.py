#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class PID:
    def __init__(self, Kp, Ki, Kd, integral_limit=0.4, derivative_limit=20):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = rospy.Time.now()

        self.integral_limit = integral_limit
        self.derivative_limit = derivative_limit

    def reset_state(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = rospy.Time.now()

    def correction(self, setpoint, actual):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        if dt <= 0:
            dt = 1e-4
        self.last_time = now

        error = setpoint - actual

        # PID terms
        p = self.Kp * error

        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        i = self.Ki * self.integral

        derivative = (error - self.prev_error) / dt
        derivative = max(min(derivative, self.derivative_limit), -self.derivative_limit)
        d = self.Kd * derivative

        self.prev_error = error

        return p + i + d


class SelfBalance:
    def __init__(self):
        rospy.init_node("selfbalancerobot")

        self.pitch = 0.0
        self.fallen_angle_threshold = math.radians(60)

        # Base PID gains
        Kp = 15.0
        Ki = 6.0
        Kd = 0.001
        self.pid_controller = PID(Kp, Ki, Kd)

        # Forward speed bias (m/s)
        self.forward_cmd = rospy.get_param("~forward_speed", 0.2)  

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

    def imu_callback(self, msg):
        q = msg.orientation
        orientation_list = [q.x, q.y, q.z, q.w]
        (_, pitch, _) = euler_from_quaternion(orientation_list)
        self.pitch = pitch

    def compute_balance_velocity(self):
        setpoint = 0.0
        actual_pitch = self.pitch

        if abs(actual_pitch) > self.fallen_angle_threshold:
            self.pid_controller.reset_state()
            rospy.logwarn("Robot fallen. PID reset.")
            return 0.0

        balance_output = self.pid_controller.correction(setpoint, actual_pitch)

        # Add forward bias to move robot while balancing
        vel = balance_output + self.forward_cmd  

        # Limit velocity
        max_vel = 0.5
        vel = max(min(vel, max_vel), -max_vel)

        rospy.loginfo(f"Pitch: {math.degrees(actual_pitch):.2f}°, Balance: {balance_output:.3f}, Cmd: {vel:.3f}")
        return vel

    def run(self):
        rate = rospy.Rate(330)  # Match IMU rate
        while not rospy.is_shutdown():
            vel_cmd = self.compute_balance_velocity()

            twist = Twist()
            twist.linear.x = vel_cmd
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

            rate.sleep()


if __name__ == '__main__':
    try:
        node = SelfBalance()
        node.run()
    except rospy.ROSInterruptException:
        pass

