#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf_transformations import quaternion_from_euler
import tf2_ros
import math

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        
        # 发布 /odom 话题
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 订阅 /cmd_vel 话题
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 初始化机器人位置和方向
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # 初始化线速度和角速度
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        # 设置循环频率
        self.timer_period = 0.1  # 10 Hz
        self.last_time = self.get_clock().now()
        
        self.timer = self.create_timer(self.timer_period, self.update_odometry)
    
    def cmd_vel_callback(self, msg):
        # 从 /cmd_vel 话题获取速度信息
        self.vx = msg.linear.x
        self.vy = msg.linear.y  # 对于差速驱动机器人通常为0
        self.vth = msg.angular.z
    
    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # 转换为秒
        
        # 计算位移
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt
        
        # 更新位置信息
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # 创建四元数表示的方向
        odom_quat = quaternion_from_euler(0, 0, self.th)
        
        # 发布TF变换
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = odom_quat[0]
        transform.transform.rotation.y = odom_quat[1]
        transform.transform.rotation.z = odom_quat[2]
        transform.transform.rotation.w = odom_quat[3]
        
        self.tf_broadcaster.sendTransform(transform)
        
        # 发布里程计消息
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        self.odom_pub.publish(odom)
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

