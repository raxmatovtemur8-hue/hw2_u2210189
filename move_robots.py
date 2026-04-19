#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class MultiRobotController:
    def __init__(self):
        self.num_robots = 35
        self.positions = {}
        self.yaws = {}
        self.cmd_pubs = {}
        self.reached = {}

        self.targets = {
            "tb3_1": (-10.0, 3.0),
            "tb3_2": (-10.0, 2.0),
            "tb3_3": (-10.0, 1.0),
            "tb3_4": (-10.0, 0.0),
            "tb3_5": (-10.0, -1.0),
            "tb3_6": (-10.0, -2.0),
            "tb3_7": (-10.0, -3.0),
            "tb3_8": (-9.0, -3.0),
            "tb3_9": (-11.0, -3.0),

            "tb3_10": (-3.0, 3.0),
            "tb3_11": (-4.0, 2.0),
            "tb3_12": (-2.0, 2.0),
            "tb3_13": (-4.0, 1.0),
            "tb3_14": (-2.0, 1.0),
            "tb3_15": (-3.0, 0.0),
            "tb3_16": (-4.0, -1.0),
            "tb3_17": (-2.0, -1.0),
            "tb3_18": (-4.0, -2.0),
            "tb3_19": (-2.0, -2.0),
            "tb3_20": (-3.0, -3.0),
            "tb3_21": (-4.0, 0.0),
            "tb3_22": (-2.0, 0.0),

            "tb3_23": (5.0, 3.0),
            "tb3_24": (4.0, 2.0),
            "tb3_25": (6.0, 2.0),
            "tb3_26": (4.0, 1.0),
            "tb3_27": (6.0, 1.0),
            "tb3_28": (5.0, 0.0),
            "tb3_29": (6.0, -1.0),
            "tb3_30": (5.0, -2.0),
            "tb3_31": (4.0, -3.0),
            "tb3_32": (6.0, -3.0),
            "tb3_33": (5.0, 2.0),
            "tb3_34": (4.0, 0.0),
            "tb3_35": (6.0, 0.0),
        }

        for i in range(1, self.num_robots + 1):
            name = f"tb3_{i}"
            self.positions[name] = None
            self.yaws[name] = 0.0
            self.reached[name] = False

            rospy.Subscriber(f"/{name}/odom", Odometry, self.odom_callback, callback_args=name)
            self.cmd_pubs[name] = rospy.Publisher(f"/{name}/cmd_vel", Twist, queue_size=10)

    def odom_callback(self, msg, robot_name):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)
        self.positions[robot_name] = (x, y)
        self.yaws[robot_name] = yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def move_robot(self, robot_name):
        if self.positions[robot_name] is None:
            return

        if self.reached[robot_name]:
            self.cmd_pubs[robot_name].publish(Twist())
            return

        current_x, current_y = self.positions[robot_name]
        target_x, target_y = self.targets[robot_name]

        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx * dx + dy * dy)

        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.yaws[robot_name])

        cmd = Twist()

        if distance < 0.10:
            self.reached[robot_name] = True
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            if abs(angle_error) > 0.25:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.6 if angle_error > 0 else -0.6
            else:
                cmd.linear.x = 0.12
                cmd.angular.z = 0.8 * angle_error

        self.cmd_pubs[robot_name].publish(cmd)

    def stop_all(self):
        stop_cmd = Twist()
        for name in self.cmd_pubs:
            self.cmd_pubs[name].publish(stop_cmd)

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            ready = sum(1 for p in self.positions.values() if p is not None)
            if ready == self.num_robots:
                break
            rate.sleep()

        group1 = [f"tb3_{i}" for i in range(1, 12)]
        group2 = [f"tb3_{i}" for i in range(12, 24)]
        group3 = [f"tb3_{i}" for i in range(24, 36)]

        while not rospy.is_shutdown():
            if not all(self.reached[name] for name in group1):
                for name in group1:
                    self.move_robot(name)
            elif not all(self.reached[name] for name in group2):
                for name in group2:
                    self.move_robot(name)
            elif not all(self.reached[name] for name in group3):
                for name in group3:
                    self.move_robot(name)
            else:
                self.stop_all()
                rospy.loginfo("All robots reached targets.")
                break

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("move_robots")
    controller = MultiRobotController()
    rospy.sleep(2.0)
    controller.run()