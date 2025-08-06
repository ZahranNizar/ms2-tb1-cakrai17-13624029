#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from queue import PriorityQueue
import time

class Nodems2(Node):
    def __init__(self):
        super().__init__('Nodems2')

        # Deklarasi dan Retrieve Parameter

        self.declare_parameter('input_autonomous_vel', 'autonomous_vel')
        self.declare_parameter('input_joy_vel', 'joy_vel')
        self.declare_parameter('input_keyboard_vel', 'keyboard_vel')        
        self.declare_parameter('output_cmd_vel', 'cmd_vel')
        self.declare_parameter('output_cmd_type', 'cmd_type')

        self.declare_parameter('autonomous_status', True)
        self.declare_parameter('joy_status', True)
        self.declare_parameter('keyboard_status', True)
        self.declare_parameter('input_topic_frequency', 5.0)
        

        topic_autonomous_vel = self.get_parameter('input_autonomous_vel').get_parameter_value().string_value
        topic_joy_vel = self.get_parameter('input_joy_vel').get_parameter_value().string_value
        topic_keyboard_vel = self.get_parameter('input_keyboard_vel').get_parameter_value().string_value
        topic_cmd_vel = self.get_parameter('output_cmd_vel').get_parameter_value().string_value
        topic_cmd_type = self.get_parameter('output_cmd_type').get_parameter_value().string_value

        autonomous_status = self.get_parameter('autonomous_status').get_parameter_value().bool_value    
        joy_status = self.get_parameter('joy_status').get_parameter_value().bool_value
        keyboard_status = self.get_parameter('keyboard_status').get_parameter_value().bool_value
        interval = self.get_parameter('input_topic_frequency').get_parameter_value().double_value

        self.last_received_message_autonomous_twist = Twist()
        self.last_received_message_joy_twist = Twist()
        self.last_received_message_keyboard_twist = Twist()

        self.last_received_message_autonomous_type = 'unknown'
        self.last_received_message_joy_type = 'unknown'
        self.last_received_message_keyboard_type = 'unknown'

        # Publishers
        self.type_publisher = self.create_publisher(
            String,
            topic_cmd_type,
            10
        )

        self.twist_publisher = self.create_publisher(
            Twist,
            topic_cmd_vel,
            10
        )

        # Subscriber
        if autonomous_status:
            self.subscriber_autonomous = self.create_subscription(
                Twist,
                topic_autonomous_vel,
                self.subscriber_autonomous_callback,
                10
            )

        if joy_status:
            self.subscriber_joy = self.create_subscription(
                Twist,
                topic_joy_vel,
                self.subscriber_joy_callback,
                10
            )

        if keyboard_status:
            self.subscriber_keyboard = self.create_subscription(
                Twist,
                topic_keyboard_vel,
                self.subscriber_keyboard_callback,
                10
            )

        self.get_logger().info('Node Milestone 2 Started')
        self.get_logger().info(f'Subscribing to : autonomous_vel')
        self.get_logger().info(f'Publishing Twist to : cmd_vel')
        self.get_logger().info(f'Publishing Type to : cmd_type')
    
        # Timer
        self.timer = self.create_timer(
            1.0 / interval,
            self.timer_callback
        )
        
    # Subscriber Callback
    def subscriber_autonomous_callback(self, msg):
        self.last_received_message_autonomous_twist = msg
        self.last_received_message_autonomous_type = 'autonomous'
        
    def subscriber_joy_callback(self, msg):
        self.last_received_message_joy_twist = msg
        self.last_received_message_joy_type = 'joy'

    def subscriber_keyboard_callback(self, msg):
        self.last_received_message_keyboard_twist = msg
        self.last_received_message_keyboard_type = 'keyboard'

    # Timer Callback
    def timer_callback(self):

        output_twist = Twist()
        output_type = String()

        # Hasil Output
        if self.last_received_message_keyboard_type == 'keyboard':
            output_twist = self.last_received_message_keyboard_twist
            output_type = self.last_received_message_keyboard_type
            
        elif self.last_received_message_joy_type == 'joy':
            output_twist = self.last_received_message_joy_twist
            output_type = self.last_received_message_joy_type

        elif self.last_received_message_autonomous_type == 'autonomous':
            output_twist = self.last_received_message_autonomous_twist
            output_type = self.last_received_message_autonomous_type

        else:
            output_twist.linear.x = float(0)
            output_twist.angular.z = float(0)
            output_type = "unkown"

        linear = output_twist.linear.x
        angular = output_twist.angular.z

        self.twist_publisher.publish(output_twist)
        self.type_publisher.publish(String(data=output_type))

        self.get_logger().info(f'Mempublikasikan Type : "{output_type}" | Waktu : {time.strftime("%H:%M:%S")} | Linear : {linear:.2f} | Angular : {angular:.2f}')
    
        self.last_received_message_autonomous_type = 'unknown'
        self.last_received_message_joy_type = 'unknown'
        self.last_received_message_keyboard_type = 'unknown'


def main(args=None):
    rclpy.init(args=args)
    node = Nodems2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
        