# -*- coding: utf-8 -*-
# 这段代码定义了一个名为 robot 的类，以及一些辅助函数。

# 工具函数库，其中包含了一些常用的功能函数，例如机器人的移动、路径规划、信息增益计算等。这些函数在前面的代码段中被调用和使用。

import rospy
import tf
from numpy import array
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from numpy import floor
from numpy.linalg import norm
from numpy import inf
# ________________________________________________________________________________


class robot:
    goal = MoveBaseGoal()
    start = PoseStamped()
    end = PoseStamped()

    def __init__(self, name):
        self.assigned_point = []
        self.name = name

        # 机器人的全局坐标系
        self.global_frame = rospy.get_param('~global_frame', '/map')
        
        # 机器人的局部坐标系
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        
        # 路径规划服务的名称
        self.plan_service = rospy.get_param(
            '~plan_service', '/move_base_node/NavfnROS/make_plan')
           
        # 创建了一个 TransformListener 对象，用于监听坐标变换。
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(
            self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0), rospy.Duration(10.0))
        cond = 0
        while cond == 0:
            try:
                rospy.loginfo('Waiting for the robot transform')
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0))
        # 这行代码尝试从 TransformListener 对象 listener 中获取机器人坐标系到全局坐标系的变换，并将结果存储在 trans 和 rot 中。
        # 变换的平移部分 trans 和旋转部分 rot，这里的trans估计是机器人相对于全局坐标的【x，y】
                
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond = 0

        # 如果成功获取了坐标变换，将变换后的机器人位置（即全局坐标系中的坐标）存储在 self.position 中。这里假设 trans 是一个包含三个元素的数组，分别表示机器人在全局坐标系中的 x、y 和 z 坐标，而我们只关心 x 和 y 坐标，所以取前两个元素作为机器人的位置。
        self.position = array([trans[0], trans[1]])

        # 将当前位置视为机器人被分配的目标点。当机器人启动时，可能还没有分配具体的目标点，因此将当前位置作为初始目标点是合理的。
        self.assigned_point = self.position


        self.client = actionlib.SimpleActionClient(
            self.name+'/move_base', MoveBaseAction) 
        # 创建一个名为 client 的动作客户端对象，用于与移动基地（move_base）服务器通信。
            
        self.client.wait_for_server()

        # 设置移动目标点的框架 ID 和时间戳。这里使用的框架 ID 是全局坐标系，时间戳是当前时间。
        robot.goal.target_pose.header.frame_id = self.global_frame
        robot.goal.target_pose.header.stamp = rospy.Time.now()

        # 等待服务 self.name+self.plan_service 准备就绪。self.plan_service 是用于获取路径规划的服务名称，通常为 move_base 节点提供。
        rospy.wait_for_service(self.name+self.plan_service)
        self.make_plan = rospy.ServiceProxy(
            self.name+self.plan_service, GetPlan)

        # 设置路径规划的起始点和目标点的框架 ID，都为全局坐标系
        robot.start.header.frame_id = self.global_frame
        robot.end.header.frame_id = self.global_frame


        # 获取机器人在全局坐标系中的当前位置
        # 通过 TransformListener 对象的 lookupTransform 方法获取机器人的坐标变换，并将其转换为机器人在全局坐标系中的位置
    def getPosition(self):
        cond = 0
        while cond == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond == 0
        self.position = array([trans[0], trans[1]])
        return self.position

# sendGoal 方法用于向移动基地服务器发送目标位置。
# 它接受一个点 point，将其设置为移动目标点，并通过动作客户端对象 self.client 发送给移动基地服务器。
# 同时，将 self.assigned_point 更新为已分配的目标点。
    def sendGoal(self, point):
        robot.goal.target_pose.pose.position.x = point[0]
        robot.goal.target_pose.pose.position.y = point[1]
        robot.goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(robot.goal)
        self.assigned_point = array(point)

# self.assigned_point 更新为当前机器人的位置，取消目标位置。
    def cancelGoal(self):
        self.client.cancel_goal()
        self.assigned_point = self.getPosition()

    def getState(self):
        return self.client.get_state()

# 用于生成从起始点到目标点的路径规划。
# 它接受起始点 start 和目标点 end，将其设置为路径规划的起始和目标位置。
# 然后，通过监听器对象将起始点和目标点转换到地图坐标系中，调用路径规划服务生成路径规划，并返回规划结果。
    def makePlan(self, start, end):
        robot.start.pose.position.x = start[0]
        robot.start.pose.position.y = start[1]
        robot.end.pose.position.x = end[0]
        robot.end.pose.position.y = end[1]
        start = self.listener.transformPose(self.name+'/map', robot.start)
        end = self.listener.transformPose(self.name+'/map', robot.end)
        plan = self.make_plan(start=start, goal=end, tolerance=0.0)
# make_plan是move_base包的方法：https://docs.ros.org/en/api/nav_msgs/html/srv/GetPlan.html
        
        return plan.plan.poses
# ________________________________________________________________________________

# 这个函数 index_of_point 用于计算地图中给定点 Xp 的索引。
# 它通过使用地图的分辨率、原点位置和宽度，将给定点的坐标转换为地图数据中的索引。
# 最后返回计算得到的索引值。
def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data
    index = int(	(floor((Xp[1]-Xstarty)/resolution) *
                  width)+(floor((Xp[0]-Xstartx)/resolution)))
    return index


def point_of_index(mapData, i):
    y = mapData.info.origin.position.y + \
        (i/mapData.info.width)*mapData.info.resolution
    x = mapData.info.origin.position.x + \
        (i-(i/mapData.info.width)*(mapData.info.width))*mapData.info.resolution
    return array([x, y])
# ________________________________________________________________________________

# 计算了给定点附近半径为 r 的区域内的信息增益。
# 首先，它通过调用 index_of_point 函数获取给定点在地图数据中的索引。
# 然后，根据给定的半径 r 计算了一个区域，遍历这个区域内的所有索引。
# 对于每个索引，如果该索引对应的地图数据值为 -1（表示未知区域）且与给定点的距离不超过半径 r，则将信息增益增加1。
# 最后，将信息增益乘以地图的分辨率的平方，并返回计算结果。
# def informationGain(mapData, point, r):
#     infoGain = 0
#     index = index_of_point(mapData, point)
#     r_region = int(r/mapData.info.resolution)
#     init_index = index-r_region*(mapData.info.width+1)
#     for n in range(0, 2*r_region+1):
#         start = n*mapData.info.width+init_index
#         end = start+2*r_region
#         limit = ((start/mapData.info.width)+2)*mapData.info.width
#         for i in range(start, end+1):
#             if (i >= 0 and i < limit and i < len(mapData.data)):
#                 if(mapData.data[i] == -1 and norm(array(point)-point_of_index(mapData, i)) <= r):
#                     infoGain += 1



#     return infoGain*(mapData.info.resolution**2)
# ________________________________________________________________________________
# def ImproveInformationGain(mapData, point,centroids, R1,R2,R3,a2,a3):
def informationGain(mapData, point, centroids,r):
    infoGain = 0
    index = index_of_point(mapData, point)
    r_region = int(r/mapData.info.resolution)
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if(mapData.data[i] == -1 and norm(array(point)-point_of_index(mapData, i)) <= r):
                    infoGain += 1
                    if(mapData.data[i] == 100 and norm(array(point)-point_of_index(mapData, i)) <= r):
                         infoGain += 0.2
                         if((point_of_index(mapData,centroids(j))-point_of_index(mapData, i)) <= 2*r):
                            infoGain += 0.5

    #分别遍历

        # infoGain_factor=1#由于后面的discournt函数的折扣是1，因此第一个增益的因子设置为1
    # R1_region = int(R1/mapData.info.resolution)
    # R2_region = int(R2/mapData.info.resolution)#范围内障碍物
    # R3_region = int(R3/mapData.info.resolution)#范围内前沿点
    # init_index_R1 = index-R1_region*(mapData.info.width+1)
    # init_index_R2 = index-R2_region*(mapData.info.width+1)
    # init_index_R3 = index-R3_region*(mapData.info.width+1)

    # for n in range(0, 2*R1_region+1):
    #     start = n*mapData.info.width+init_index_R1
    #     end = start+2*R1_region
    #     limit = ((start/mapData.info.width)+2)*mapData.info.width
    #     for i in range(start, end+1):
    #         if (i >= 0 and i < limit and i < len(mapData.data)):
    #             if(mapData.data[i] == -1 and norm(array(point)-point_of_index(mapData, i)) <= R1):
    #                 infoGain += infoGain_factor

    # for n in range(0, 2*R2_region+1):#范围内障碍物
    #     start = n*mapData.info.width+init_index_R2
    #     end = start+2*R2_region
    #     limit = ((start/mapData.info.width)+2)*mapData.info.width
    #     for i in range(start, end+1):
    #         if (i >= 0 and i < limit and i < len(mapData.data)):
    #             if(mapData.data[i] == 100 and norm(array(point)-point_of_index(mapData, i)) <= R2):
    #                 infoGain += a2*infoGain_factor

    # for n in range(0, 2*R3_region+1):#范围内前沿点
    #     start = n*mapData.info.width+ init_index_R3
    #     end = start+2*R3_region
    #     limit = ((start/mapData.info.width)+2)*mapData.info.width
    #     for i in range(start, end+1):
    #         if (i >= 0 and i < limit and i < len(mapData.data)):
    #             for j in range(0,len(centroids)):
    #                 if((point_of_index(mapData,centroids(j))-point_of_index(mapData, i)) <= R3):
    #                     infoGain += a3*infoGain_factor

    return infoGain*(mapData.info.resolution**2)
# ________________________________________________________________________________



def discount(mapData, assigned_pt, centroids, infoGain, r):
    index = index_of_point(mapData, assigned_pt)
    r_region = int(r/mapData.info.resolution)
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                for j in range(0, len(centroids)):
                    current_pt = centroids[j]
                    if(mapData.data[i] == -1 and norm(point_of_index(mapData, i)-current_pt) <= r and norm(point_of_index(mapData, i)-assigned_pt) <= r):
                        # this should be modified, subtract the area of a cell, not 1
                        infoGain[j] -= 1
                        if(mapData.data[i] == 100 and norm(point_of_index(mapData, i)-current_pt) <= r and norm(point_of_index(mapData, i)-assigned_pt) <= r):
                            infoGain[j] -= 0.2
                            if(mapData.data[i] == centroids[j] and norm(point_of_index(mapData, i)-current_pt) <= r and norm(point_of_index(mapData, i)-assigned_pt) <= 2*r):
                                infoGain[j] -= 0.5


    return infoGain
# ________________________________________________________________________________

# 计算给定路径的代价
def pathCost(path):
    if (len(path) > 0):
        i = len(path)/2
        p1 = array([path[i-1].pose.position.x, path[i-1].pose.position.y])
        p2 = array([path[i].pose.position.x, path[i].pose.position.y])
        return norm(p1-p2)*(len(path)-1)
    else:
        return inf
# ________________________________________________________________________________



# 用于检查给定点 pt 是否处于无效区域（障碍物区域）。
# 首先，它通过调用 index_of_point 函数获取给定点在地图数据中的索引。
# 然后，类似于前面的函数，计算了一个区域，遍历这个区域内的所有索引。
# 对于每个索引，如果该索引对应的地图数据值为 1（表示障碍物），则返回 True，表示给定点处于无效区域。
# 如果遍历完区域后都没有发现障碍物，则返回 False，表示给定点处于有效区域。
def unvalid(mapData, pt):
    index = index_of_point(mapData, pt)
    r_region = 3
    init_index = index-r_region*(mapData.info.width+1)
    for n in range(0, 2*r_region+1):
        start = n*mapData.info.width+init_index
        end = start+2*r_region
        limit = ((start/mapData.info.width)+2)*mapData.info.width
        for i in range(start, end+1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                 if(mapData.data[i] >0):
                    return True
    return False
# ________________________________________________________________________________

# 这个函数 Nearest 用于找到点集 V 中与给定点 x 最近的点的索引。
# 它遍历点集 V 中的所有点，计算每个点与给定点的距离，并找到距离最小的点的索引。
# 最后返回距离最小的点的索引。
def Nearest(V, x):
    n = inf
    i = 0
    for i in range(0, V.shape[0]):
        n1 = norm(V[i, :]-x)
        if (n1 < n):
            n = n1
            result = i
    return result

# ________________________________________________________________________________


def Nearest2(V, x):
    n = inf
    result = 0
    for i in range(0, len(V)):
        n1 = norm(V[i]-x)

        if (n1 < n):
            n = n1
    return i
# ________________________________________________________________________________


# 这个函数 gridValue 用于获取地图中指定位置 Xp 的网格值.
# 它首先计算出指定位置在地图数据中的索引 index，然后根据索引返回对应的地图数据值。
# 如果索引超出了地图数据的范围，则返回 100，表示占据状态。
def gridValue(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    width = mapData.info.width
    Data = mapData.data
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free
    index = (floor((Xp[1]-Xstarty)/resolution)*width) + \
        (floor((Xp[0]-Xstartx)/resolution))

    if int(index) < len(Data):
        return Data[int(index)]
    else:
        return 100
