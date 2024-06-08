#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class VelocityAdjuster:
    def __init__(self):
        self.node_name = "velocity_adjuster"
        self.scale_factor = rospy.get_param('~exploration_percentage_scale', 1.0)  # 探索百分比的缩放因子

        # 订阅cmd_vel话题以获取当前速度命令
        self.subscriber_cmd_vel = rospy.Subscriber("/tb3_1/cmd_vel", Twist, self.cmd_vel_callback)
        
        # 订阅exploration_percentage话题
        self.subscriber_exploration = rospy.Subscriber("exploration_percentage", Float32, self.exploration_percentage_callback)
        
        # 发布到cmd_vel话题
        self.publisher_cmd_vel = rospy.Publisher("/tb3_1/cmd_vel", Twist, queue_size=10)

        self.current_twist = Twist()

    def cmd_vel_callback(self, data):
        # 保存当前的cmd_vel消息
        self.current_twist = data

    def exploration_percentage_callback(self, data):
        # 将探索百分比除以100并取e为底的指数
        exploration_percentage = data.data / 100.0
        exponential_factor = math.exp(exploration_percentage * self.scale_factor)

        # 根据指数因子调整线速度和角速度
        if self.current_twist is not None:
            adjusted_linear_speed = self.current_twist.linear.x * exponential_factor
            adjusted_angular_speed = self.current_twist.angular.z * exponential_factor

            # 更新线速度和角速度
            self.current_twist.linear.x = adjusted_linear_speed
            self.current_twist.angular.z = adjusted_angular_speed
            self.publisher_cmd_vel.publish(self.current_twist)
            # rospy.loginfo("Published adjusted velocity: linear = %f, angular = %f" % (adjusted_linear_speed, adjusted_angular_speed))

    def run(self):
        rospy.init_node(self.node_name, anonymous=True)
        rospy.spin()

if __name__ == '__main__':
    try:
        va = VelocityAdjuster()
        va.run()
    except rospy.ROSInterruptException:
        pass