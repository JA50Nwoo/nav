#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 筛选前沿点，并将其发布出去。它订阅了地图数据和全局成本地图数据，通过机器人的全局成本地图数据，判断前沿点是否可达，并将可达的前沿点发布出去。
# 这段代码中的 callBack 函数负责接收并合并前沿点数据，mapCallBack 函数接收地图数据，globalMap 函数接收全局成本地图数据，
# 而 node 函数则是主要的运行函数，用于筛选前沿点并发布。


# --------Include modules---------------
from copy import copy
# 用于复制对象
import rospy
from visualization_msgs.msg import Marker
# 从ROS的 visualization_msgs 包中导入 Marker 消息类型。
from geometry_msgs.msg import Point
# 从ROS的 geometry_msgs 包中导入 Point 消息类型。
from nav_msgs.msg import OccupancyGrid
# 从ROS的 nav_msgs 包中导入 OccupancyGrid 消息类型。
from geometry_msgs.msg import PointStamped
# 从ROS的 geometry_msgs 包中导入 PointStamped 消息类型。
import tf
# 导入ROS的 tf 库，用于处理坐标变换
from numpy import array, vstack, delete
# 从 numpy 库中导入 array, vstack, 和 delete 函数，用于数组操作。
from functions import gridValue, index_of_point, informationGain,unvalid
# 从自定义的 functions 模块中导入 gridValue 和 informationGain 函数。
from sklearn.cluster import MeanShift
# 用于聚类算法。
from rrt_exploration.msg import PointArray

# Subscribers' callbacks------------------------------
mapData = OccupancyGrid()
# mapData 是一个全局变量，用于存储接收到的局部地图数据。
# 当收到地图数据的回调函数 mapCallBack 被调用时，地图数据会存储在 mapData 中。
# 主要用于处理机器人周围的局部环境，例如识别前沿点和计算信息增益等。
frontiers = []
globalmaps = []
# gloabamaps是一个列表，用于全局地图储存
# 当收到全局地图数据的回调函数 globalMap 被调用时，全局地图数据会存储在 globalmaps 中的相应位置。
# 主要用于处理全局路径规划和多机器人协同导航的情况，例如检测障碍物、规划全局路径等


def callBack(data, args):#处理前沿点数据
    global frontiers, min_distance#声明全局变量
    transformedPoint = args[0].transformPoint(args[1], data)
    # 通过传入的参数 args，调用 transformPoint 方法将数据 data 从一个坐标系转换到另一个坐标系，得到转换后的点坐标 transformedPoint。
    x = [array([transformedPoint.point.x, transformedPoint.point.y])]
    # 将转换后的点坐标组成一个数组 x，这里使用了 array 函数将点坐标转换为数组形式。
    if len(frontiers) > 0:
        frontiers = vstack((frontiers, x))
        #将x加入frontier列表
    else:
        frontiers = x


def mapCallBack(data):#局部地图信息
    global mapData
    mapData = data


def globalMap(data):
    global global1, globalmaps, litraIndx, namespace_init_count, n_robots
    global1 = data
     # 将收到的全局地图数据存储在 global1 变量中
    if n_robots > 1:
        indx = int(data._connection_header['topic']
                   [litraIndx])-namespace_init_count
    elif n_robots == 1:

        # 如果机器人数量大于1，则需要根据数据的topic来确定在globalmaps列表中的索引
    # 从数据的连接头中获取topic的索引，并根据 namespace_init_count 进行调整
    # 这个索引用于确定在 globalmaps 列表中存储全局地图数据的位置
        indx = 0
    globalmaps[indx] = data
     # 将收到的全局地图数据存储在 globalmaps 列表中的相应位置
    # 如果只有一个机器人，存储在索引为0的位置，否则根据计算出的indx进行存储

# Node----------------------------------------------


def node():
    global frontiers, mapData, global1, global2, global3, globalmaps, litraIndx, n_robots, namespace_init_count
    rospy.init_node('filter', anonymous=False)

    # fetching all parameters
    map_topic = rospy.get_param('~map_topic', '/map')
    threshold = rospy.get_param('~costmap_clearing_threshold', 70)
    # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    info_radius = rospy.get_param('~info_radius', 1.0)
    goals_topic = rospy.get_param('~goals_topic', '/detected_points')
    n_robots = rospy.get_param('~n_robots', 1)
    namespace = rospy.get_param('~namespace', '')
    namespace_init_count = rospy.get_param('namespace_init_count', 1)
    rateHz = rospy.get_param('~rate', 100)
    global_costmap_topic = rospy.get_param(
        '~global_costmap_topic', '/move_base_node/global_costmap/costmap')
    robot_frame = rospy.get_param('~robot_frame', 'base_link')

    litraIndx = len(namespace)
    rate = rospy.Rate(rateHz)
# -------------------------------------------
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
     # 订阅地图话题，当有新的地图数据到来时，调用mapCallBack函数进行处理


# ---------------------------------------------------------------------------------------------------------------

    for i in range(0, n_robots):
        globalmaps.append(OccupancyGrid())
# 初始化存储全局地图数据的列表，长度为机器人数量，每个元素都是一个空的 OccupancyGrid 对象

    if len(namespace) > 0:
        for i in range(0, n_robots):
            rospy.Subscriber(namespace+str(i+namespace_init_count) +
                             global_costmap_topic, OccupancyGrid, globalMap)
    # 如果存在命名空间（说明有多个机器人），根据命名空间和topic订阅每个机器人的全局地图数据
    # 每个机器人的topic命名为namespace + i + namespace_init_count + global_costmap_topic
    # 当有新的全局地图数据到来时，调用globalMap函数进行处理
    elif len(namespace) == 0:
        # 如果没有命名空间，直接订阅全局地图话题
        rospy.Subscriber(global_costmap_topic, OccupancyGrid, globalMap)
# wait if map is not received yet
    while (len(mapData.data) < 1):
        rospy.loginfo('Waiting for the map')
        rospy.sleep(0.1)
        pass
        # 等待接收到地图数据，如果地图数据长度小于1，则一直等待
# wait if any of robots' global costmap map is not received yet
    for i in range(0, n_robots):
        while (len(globalmaps[i].data) < 1):
            rospy.loginfo('Waiting for the global costmap')
            rospy.sleep(0.1)
            pass

    global_frame = "/"+mapData.header.frame_id
    # 获取地图数据的全局坐标系，用于后续坐标转换

    tfLisn = tf.TransformListener()
    if len(namespace) > 0:
        for i in range(0, n_robots):
            tfLisn.waitForTransform(global_frame[1:], namespace+str(
                i+namespace_init_count)+'/'+robot_frame, rospy.Time(0), rospy.Duration(10.0))
#在tf监听器tfLisn上调用waitForTransform方法，等待获取从全局坐标系到机器人坐标系的坐标变换关系。具体参数解释如下：
# global_frame[1:]：表示全局坐标系，使用了索引操作[1:]来去除字符串开头的斜杠。
# namespace + str(i + namespace_init_count) + '/' + robot_frame：构建机器人坐标系的名称。命名空间 + 机器人索引 + 机器人基准坐标系名称。
# rospy.Time(0)：表示获取最新的坐标变换。
# rospy.Duration(10.0)：表示等待10秒钟的时间，如果在这段时间内没有获取到坐标变换关系，则放弃等待。
                
    elif len(namespace) == 0:
# 如果命名空间的长度为0，表示不存在命名空间，直接等待全局坐标系到机器人基准坐标系的坐标变换关系。
        tfLisn.waitForTransform(
            global_frame[1:], '/'+robot_frame, rospy.Time(0), rospy.Duration(10.0))

    rospy.Subscriber(goals_topic, PointStamped, callback=callBack,
                     callback_args=[tfLisn, global_frame[1:]])
 # 订阅前沿点话题，当有新的前沿点数据到来时，调用callBack函数进行处理

    pub = rospy.Publisher('frontiers', Marker, queue_size=10)
    pub2 = rospy.Publisher('centroids', Marker, queue_size=10)
    filterpub = rospy.Publisher('filtered_points', PointArray, queue_size=10)
 # 创建三个话题发布器，用于发布前沿点、中心点和过滤后的点
    rospy.loginfo("the map and global costmaps are received")

    # wait if no frontier is received yet
    while len(frontiers) < 1:
        pass

    points = Marker()
    points_clust = Marker()
    # 创建两个 Marker 类型的消息对象，用于可视化显示前沿点和聚类后的中心点，ros2中没有

# Set the frame ID and timestamp.  See the TF tutorials for information on these.
    points.header.frame_id = mapData.header.frame_id
    points.header.stamp = rospy.Time.now()
    # 设置前沿点消息的坐标系和时间戳

    points.ns = "markers2"
    points.id = 0
# 设置前沿点消息的命名空间和ID，以及消息类型为POINTS，表示将要发布的是一组点
    points.type = Marker.POINTS

# Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points.action = Marker.ADD

    points.pose.orientation.w = 1.0

    points.scale.x = 0.2
    points.scale.y = 0.2

    points.color.r = 255.0/255.0
    points.color.g = 255.0/255.0
    points.color.b = 0.0/255.0

    points.color.a = 1
    # 这一段都在设置maker的显示，颜色之类的
    points.lifetime = rospy.Duration()

    p = Point()
    p.z = 0
 # 创建一个 Point 类型的消息对象，并将其z坐标设为0，因为前沿点是二维的
    pp = []
    pl = []

    points_clust.header.frame_id = mapData.header.frame_id
    points_clust.header.stamp = rospy.Time.now()

    points_clust.ns = "markers3"
    points_clust.id = 4

    points_clust.type = Marker.POINTS

# Set the marker action for centroids.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points_clust.action = Marker.ADD

    points_clust.pose.orientation.w = 1.0

    points_clust.scale.x = 0.2
    points_clust.scale.y = 0.2
    points_clust.color.r = 0.0/255.0
    points_clust.color.g = 255.0/255.0
    points_clust.color.b = 0.0/255.0

    points_clust.color.a = 1
    points_clust.lifetime = rospy.Duration()

    temppoint = PointStamped()
    temppoint.header.frame_id = mapData.header.frame_id
    temppoint.header.stamp = rospy.Time(0)
    temppoint.point.z = 0.0

    arraypoints = PointArray()
    tempPoint = Point()
    tempPoint.z = 0.0

    # 创建一个 PointArray 类型的消息对象，用于存储聚类后的中心点坐标
    # 创建一个临时的 Point 类型的消息对象，并将其z坐标设为0
# -------------------------------------------------------------------------
# ---------------------     Main   Loop     -------------------------------
# -------------------------------------------------------------------------
    while not rospy.is_shutdown():
        # -------------------------------------------------------------------------
        # Clustering frontier points
        centroids = []
        # 空列表储存聚类中心点
        front = copy(frontiers)
        # 复制以便于不改变原始数据 
        if len(front) > 1:
            ms = MeanShift(bandwidth=0.3)
            ms.fit(front)
# 在这里，fit 是 MeanShift 类的一个方法，用于对数据进行聚类操作。
# MeanShift 是一种基于密度的非参数聚类算法，它的主要思想是在数据集中寻找局
# 部密度最大的区域，然后通过迭代的方式将数据点移动到密度最大的区域中心，直到收敛为止。

# 具体来说，fit 方法会接收数据集作为输入，并在数据集上执行聚类操作。
# 对于输入的数据集，MeanShift 算法会根据数据点的密度分布自动确定聚类的中心，并将数据点分配到相应的聚类中。
# 最终，fit 方法会返回聚类的中心点。

            centroids = ms.cluster_centers_  # centroids array is the centers of each cluster

        # if there is only one frontier no need for clustering, i.e. centroids=frontiers
        if len(front) == 1:
            # 数量为一则直接赋值
            centroids = front
        frontiers = copy(centroids)
# -------------------------------------------------------------------------
# clearing old frontiers

        z = 0
        while z < len(centroids):
            cond = False
            # 用于记录是否需要清理当前中心点
            temppoint.point.x = centroids[z][0]
            temppoint.point.y = centroids[z][1]
            # 将当前中心点的坐标赋值给 temppoint。
# temppoint是个pointstamp，坐标为浮点数，occupany是个网格，其上面的data是分配给每个单元的，因此这里给了一个index_of_point函数用来得到某个点最近的index，用来代表这个点的状态（是否已知）
# 而需要使用unvalid的话，需要先使用index_of_point来找到其最近的index，再在occupanyGrid查看周围的点有没有墙壁





            for i in range(0, n_robots):
                # 遍历每个机器人

                transformedPoint = tfLisn.transformPoint(
                    globalmaps[i].header.frame_id, temppoint)
                    # 使用 tfLisn.transformPoint 函数将中心点坐标从全局坐标系转换到机器人的坐标系。
                x = array([transformedPoint.point.x, transformedPoint.point.y])
                # 根据转换后的坐标，计算在机器人所处地图中对应的坐标。
                # cond = (gridValue(globalmaps[i], x) > threshold) or cond
                cond2=unvalid(mapData, centroids[z])
                # 如果当前中心点位于障碍物或者信息增益低于阈值，则将 cond 置为True
            if (cond2 or (informationGain(mapData, [centroids[z][0], centroids[z][1]], centroids,info_radius*0.5)) < 0.2):
                # 如果 cond 为True，或者当前中心点的信息增益低于阈值，删除
                # rospy.loginfo('fuck')
                centroids = delete(centroids, (z), axis=0)
                # rospy.loginfo('fuckfuck')
                z = z-1
                # rospy.loginfo('fuckfuckfuck')
            z += 1
# -------------------------------------------------------------------------
# publishing
        arraypoints.points = []#中心点数据存储列表，列表是可变容器可以储存任意数据，这里用于储存[x,y]这样的坐标
        for i in centroids:
            tempPoint.x = i[0]
            tempPoint.y = i[1]
            arraypoints.points.append(copy(tempPoint))
            # 将每个中心点（聚类后的）储存到arraypoints中，准备发布
            # transformedPoint = PointStamped()
            # transformedPoint.header.frame_id = global_frame
            # transformedPoint.point.x = tempPoint.x
            # transformedPoint.point.y = tempPoint.y
            # grid_value = gridValue(mapData, [tempPoint.x, tempPoint.y])
            # rospy.loginfo("Point (%f, %f): gridValue = %f" % (tempPoint.x, tempPoint.y, grid_value))
            # rospy.loginfo("%s",unvalid(mapData, [tempPoint.x, tempPoint.y]))
        filterpub.publish(arraypoints)
        #  将存储了中心点数据的 arraypoints 发布到 filtered_points 话题上，供其他节点使用。
        pp = []#储存前沿点的坐标数据
        for q in range(0, len(frontiers)):
            #遍历前沿点
            p.x = frontiers[q][0]
            p.y = frontiers[q][1]
            # 将当前前沿点的x坐标和y坐标分别赋值给临时变量 p 的x坐标和y坐标。
            pp.append(copy(p))
        points.points = pp
        pp = []
        for q in range(0, len(centroids)):
            #遍历中心点
            p.x = centroids[q][0]
            p.y = centroids[q][1]
            pp.append(copy(p))
        points_clust.points = pp
        pub.publish(points)
        pub2.publish(points_clust)
        rate.sleep()
# -------------------------------------------------------------------------


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
