#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import OccupancyGrid,Odometry
import os
import subprocess
import time
import rospy
from geometry_msgs.msg import PointStamped,PoseStamped
import matplotlib.pyplot as plt
import datetime
import math

from std_msgs.msg import Float32  # 导入Float32消息类型

mapData = OccupancyGrid()
odomData = Odometry()
start_time = None
run_time = 0
exploration_percentages = []
times = []
record_frequency = 0.1 # 每隔5秒记录一次数据
total_distance = 0  # 新增变量，用于存储总路程
last_record_time = None

def odom_callback(data):
    global  prev_pose, total_distance  # 使用global声明prev_pose和total_distance为全局变量
    odomData = data
    if prev_pose is not None:
        position = [odomData.pose.pose.position.x, odomData.pose.pose.position.y]
        distance = math.sqrt((position[0] - prev_pose[0])**2 + (position[1] - prev_pose[1])**2)
        total_distance += distance
    prev_pose = [odomData.pose.pose.position.x, odomData.pose.pose.position.y, odomData.pose.pose.position.z]

def clicked_point_callback(data):
    rospy.loginfo("Received click point: %s", data)

def record_clicked_points():
    rospy.Subscriber("/clicked_point", PointStamped, clicked_point_callback)

def map_callback(data):
    global mapData
    mapData = data
    

def check_and_save_map():
    global mapData, start_time, run_time, exploration_percentages, times,percentage_threshold,last_record_time
    percentage_threshold= rospy.get_param('percentage_threshold',20)
    # w = mapData.info.width
    # h = mapData.info.height
    # rospy.loginfo("chang%s" % w)
    # rospy.loginfo("kuan%s" % h)
    if mapData is None or start_time is None:
        return
    total_cells = float(mapData.info.width * mapData.info.height)
    if total_cells == 0:
    
        rospy.logwarn("Total number of cells is zero, can't calculate exploration percentage.")
        return
    explored_cells = sum(1 for cell in mapData.data if cell == 0)

    exploration_percentage = (explored_cells / total_cells * 100)*100/24
    pub = rospy.Publisher('exploration_percentage', Float32, queue_size=10)
    pub.publish(exploration_percentage)


    # rospy.loginfo("Exploration percentage: %s%%" % exploration_percentage)
    current_time = datetime.datetime.now()
    if last_record_time is None or (current_time - last_record_time).total_seconds() >= record_frequency:
        # rospy.loginfo("Exploration percentage: %s%%" % exploration_percentage)
        exploration_percentages.append(exploration_percentage)
        times.append((current_time - start_time).total_seconds())
        last_record_time = current_time



    if exploration_percentage >= percentage_threshold*100/24:  # Change the threshold as needed
        save_map_and_shutdown("Exploration reached %s. Stopping."%percentage_threshold)

def save_map_and_shutdown(reason):
    global start_time,total_distance
    if start_time is None:
        run_time = 0
    else:
        run_time = (datetime.datetime.now() - start_time).total_seconds()
    rospy.loginfo("RViz has been running for %s seconds." % run_time)
    rospy.loginfo("Total distance traveled: %f meters" % total_distance)
    # rospy.loginfo("fafafafaaafa %s", mapData.info.resolution)
    

    
    output_folder = os.path.expanduser('~/catkin_explore/mymap')
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    map_filename = os.path.join(output_folder, 'explored_map_%s.yaml' % time.strftime("%Y%m%d%H%M%S"))
    # Plot and save the graph
    plt.figure()
    plt.plot(times, exploration_percentages)
    plt.title('Exploration Percentage Over Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Exploration Percentage (%)')
    plt.grid(True)
    plt.ylim(0, 100)
    plt.xlim(0, run_time)
    plt.savefig(os.path.join(output_folder, 'exploration_percentage_over_time.png'))
    plt.close()

    try:
        # rospy.loginfo('212121212')
        # Use shell=True parameter to execute rostopic command
        subprocess.Popen("rostopic echo /tb3_1/map | rostopic pub /map nav_msgs/OccupancyGrid -r 1", shell=True)
        # rospy.loginfo('87897798')
        # Use shell=True parameter to execute rosrun command
        subprocess.Popen("rosrun map_server map_saver -f %s" % map_filename, shell=True)
        # rospy.loginfo('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
        rospy.loginfo("Map saved successfully to %s" % map_filename)
        # rospy.loginfo('bbbbbbbbbbbbbbbbbbb')

    except subprocess.CalledProcessError as e:
        rospy.logerr("Failed to save the map. Error: %s" % str(e))
    finally:
        rospy.signal_shutdown(reason)

def node():
    global mapData, start_time,last_record_time,prev_pose
    rospy.init_node('map_saver_node', anonymous=False)
    start_time = datetime.datetime.now()  # 确保这里设置 start_time
    last_record_time = start_time  # 初始化 last_record_time
    prev_pose = None  # 新增变量，用于存储上一个位置
    map_topic = rospy.get_param('~map_topic', '/map')
    rospy.Subscriber(map_topic, OccupancyGrid, map_callback)
    rate = rospy.Rate(10)  # Check frequency at 10Hz
    rospy.Subscriber("/tb3_1/odom", Odometry, odom_callback)

    while not rospy.is_shutdown():
        check_and_save_map()
        rate.sleep()
        # pass

if __name__ == '__main__':
    try:
        record_clicked_points()
        node()
    except rospy.ROSInterruptException:
        pass