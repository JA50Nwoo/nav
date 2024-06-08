#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import numpy as np

mapData = OccupancyGrid()
explored_area = 0


def mapCallBack(data):
    global mapData
    mapData = data

def calculateExploredArea():
    global explored_area
    explored_area = np.sum(np.array(mapData.data) == 0) * mapData.info.resolution ** 2

def calculateCentroid():
    global mapData

    # Initialize variables to store centroid coordinates and weight
    centroid_x = 0.0
    centroid_y = 0.0
    total_weight = 0

    # Calculate the physical coordinates of the map origin
    map_origin_x = mapData.info.origin.position.x
    map_origin_y = mapData.info.origin.position.y

    # Iterate through the map data
    for i in range(len(mapData.data)):
        # Convert map index to grid coordinates
        row = i // mapData.info.width
        col = i % mapData.info.width

        # Calculate weight based on exploration status (explored = 1, unexplored = 0)
        weight = 1 if mapData.data[i] == 0 else 0

        # Calculate physical coordinates of the current cell
        cell_x = map_origin_x + col * mapData.info.resolution
        cell_y = map_origin_y + row * mapData.info.resolution

        # Update centroid coordinates and total weight
        centroid_x += cell_x * weight
        centroid_y += cell_y * weight
        total_weight += weight

    # Normalize centroid coordinates
    if total_weight > 0:
        centroid_x /= total_weight
        centroid_y /= total_weight

    return np.array([centroid_x, centroid_y])


# 在 node() 函数外创建发布者
centroid_pub = rospy.Publisher('/explored_centroid', PointStamped, queue_size=10)


def node():
    global mapData
    rospy.init_node('explored_centroid', anonymous=False)

    map_topic = rospy.get_param('~map_topic', '/map')
    rateHz = rospy.get_param('~rate', 100) # Update rate
    rate = rospy.Rate(rateHz)

    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)



    while not rospy.is_shutdown():
        calculateExploredArea()  # Calculate the explored area
        centroid = calculateCentroid()  # Calculate the centroid
        centroid_msg = PointStamped()
        centroid_msg.header.stamp = rospy.Time.now()
        centroid_msg.header.frame_id = mapData.header.frame_id
        centroid_msg.point.x = centroid[0]
        centroid_msg.point.y = centroid[1]
        centroid_msg.point.z = 0.0

        
        # Publish the centroid
        centroid_pub.publish(centroid_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
