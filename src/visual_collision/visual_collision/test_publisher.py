# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import String


# class MinimalPublisher(Node):

#     def __init__(self):
#         super().__init__('minimal_publisher')
#         self.publisher_ = self.create_publisher(String, 'topic', 10)
#         timer_period = 0.5  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0

#     def timer_callback(self):
#         msg = String()
#         msg.data = 'Hello World: %d' % self.i
#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.data)
#         self.i += 1


# def main(args=None):
#     rclpy.init(args=args)

#     minimal_publisher = MinimalPublisher()

#     rclpy.spin(minimal_publisher)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_publisher.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from multi_crane_msg.msg import MultiCraneMsg, TowerCraneMsg, LuffingJibCraneMsg  # Replace with your package name

import math

class MultiCranePublisher(Node):

    def __init__(self):
        super().__init__('multi_crane_publisher')
        self.publisher_ = self.create_publisher(MultiCraneMsg, 'multi_crane', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # Create an instance of MultiCraneMsg
        multi_crane_msgs = MultiCraneMsg()

        # Create two TowerCrane messages
        tower_crane_msg1 = TowerCraneMsg()
        tower_crane_msg1.crane_id = 1
        tower_crane_msg1.crane_type = "towerCrane"
        tower_crane_msg1.crane_x = 20.0
        tower_crane_msg1.crane_y = 30.0
        tower_crane_msg1.crane_z = 50.0
        tower_crane_msg1.boom_length = 25.0
        # tower_crane_msg1.boom_angle = math.radians(30 + 10 * math.sin(self.i / 10.0))  # Dynamically changing
        tower_crane_msg1.boom_angle = math.radians((self.i * 5.0) % 360) # Dynamically changing
        tower_crane_msg1.trolley_radius = 5 + 2 * math.sin(self.i / 10.0)  # Dynamically changing
        tower_crane_msg1.hook_height = 5 + 2 * math.sin(self.i / 10.0)  # Dynamically changing

        tower_crane_msg2 = TowerCraneMsg()
        tower_crane_msg2.crane_id = 2
        tower_crane_msg2.crane_type = "towerCrane"
        tower_crane_msg2.crane_x = 40.0
        tower_crane_msg2.crane_y = 50.0
        tower_crane_msg2.crane_z = 40.0
        tower_crane_msg2.boom_length = 20.0
        tower_crane_msg2.boom_angle = math.radians((self.i * 10.0) % 360) # Dynamically changing
        tower_crane_msg2.trolley_radius = 5 + 2 * math.sin(self.i / 10.0)  # Dynamically changing
        tower_crane_msg2.hook_height = 5 + 2 * math.sin(self.i / 10.0)  # Dynamically changing

        # Create a LuffingJibCrane message
        luffing_jib_crane_msg1 = LuffingJibCraneMsg()
        luffing_jib_crane_msg1.crane_id = 3
        luffing_jib_crane_msg1.crane_type = "luffingJibCrane"
        luffing_jib_crane_msg1.crane_x = 60.0
        luffing_jib_crane_msg1.crane_y = 20.0
        luffing_jib_crane_msg1.crane_z = 20.0
        luffing_jib_crane_msg1.boom_length = 20.0
        luffing_jib_crane_msg1.boom_hor_angle = math.radians((self.i * 5.0) % 360) # Dynamically changing
        luffing_jib_crane_msg1.boom_ver_angle = math.radians((self.i * 5.0) % 90) # Dynamically changing
        luffing_jib_crane_msg1.hook_height = 15 + 2 * math.sin(self.i / 10.0)  # Dynamically changing

        # Add messages to MultiCraneMsg
        multi_crane_msgs.tower_crane_msgs.append(tower_crane_msg1)
        multi_crane_msgs.tower_crane_msgs.append(tower_crane_msg2)
        multi_crane_msgs.luffing_jib_crane_msgs.append(luffing_jib_crane_msg1)

        # Publish the message
        self.publisher_.publish(multi_crane_msgs)
        self.get_logger().info(f'Publishing MultiCraneMsg with {len(multi_crane_msgs.tower_crane_msgs)} tower cranes and {len(multi_crane_msgs.luffing_jib_crane_msgs)} luffing jib cranes.')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    multi_crane_publisher = MultiCranePublisher()

    rclpy.spin(multi_crane_publisher)

    multi_crane_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()