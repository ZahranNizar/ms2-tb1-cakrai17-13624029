#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
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
        self.declare_parameter('input_frequency', 5.0)
        

        topic_autonomous_vel = self.get_parameter('input_autonomous_vel').get_parameter_value().string_value
        topic_joy_vel = self.get_parameter('input_joy_vel').get_parameter_value().string_value
        topic_keyboard_vel = self.get_parameter('input_keyboard_vel').get_parameter_value().string_value
        topic_cmd_vel = self.get_parameter('output_cmd_vel').get_parameter_value().string_value
        topic_cmd_type = self.get_parameter('output_cmd_type').get_parameter_value().string_value

        autonomous_status = self.get_parameter('autonomous_status').get_parameter_value().bool_value    
        joy_status = self.get_parameter('joy_status').get_parameter_value().bool_value
        keyboard_status = self.get_parameter('keyboard_status').get_parameter_value().bool_value
        self.interval = self.get_parameter('input_frequency').get_parameter_value().double_value

        self.last_received_message_autonomous_twist = Twist()
        self.last_received_message_joy_twist = Twist()
        self.last_received_message_keyboard_twist = Twist()

        self.waktu_autonomous = self.get_clock().now().nanoseconds / 1e9
        self.waktu_joy = self.get_clock().now().nanoseconds / 1e9
        self.waktu_keyboard = self.get_clock().now().nanoseconds / 1e9

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
            1.0 / self.interval,
            self.timer_callback
        )
        
    # Subscriber Callback
    def subscriber_autonomous_callback(self, msg):
        self.last_received_message_autonomous_twist = msg
        self.waktu_autonomous = self.get_clock().now().nanoseconds / 1e9
        
    def subscriber_joy_callback(self, msg):
        self.last_received_message_joy_twist = msg
        self.waktu_joy = self.get_clock().now().nanoseconds / 1e9

    def subscriber_keyboard_callback(self, msg):
        self.last_received_message_keyboard_twist = msg
        self.waktu_keyboard = self.get_clock().now().nanoseconds / 1e9

    # Timer Callback
    def timer_callback(self):
        waktu_sekarang = self.get_clock().now().nanoseconds / 1e9

        output_twist = Twist()
        output_type = 'Null'

        # Hasil Output
        if (waktu_sekarang - self.waktu_keyboard) < (1.0 / self.interval):
            output_twist = self.last_received_message_keyboard_twist
            output_type = 'keyboard'
            
        elif (waktu_sekarang - self.waktu_joy) < (1.0 / self.interval):
            output_twist = self.last_received_message_joy_twist
            output_type = 'joy'

        elif (waktu_sekarang - self.waktu_autonomous) < (1.0 / self.interval):
            output_twist = self.last_received_message_autonomous_twist
            output_type = 'autonomous'

        else:
            output_twist.linear.x = 0.0
            output_twist.angular.z = 0.0
            output_type = 'unknown'

        linear = output_twist.linear.x
        angular = output_twist.angular.z

        type_msg = String()
        type_msg.data = output_type

        self.type_publisher.publish(type_msg)
        self.twist_publisher.publish(output_twist)

        self.get_logger().info(f'Mempublikasikan Type : "{output_type}" | Waktu : {time.strftime("%H:%M:%S")} | Linear : {linear:.2f} | Angular : {angular:.2f}')


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
        
