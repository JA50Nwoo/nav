#!/usr/bin/env python
#Shebang 行是指在脚本文件的第一行中的特殊注释，用于指定要用于执行脚本的解释器。通常的形式是 #!/path/to/interpreter，其中 #! 是一个特殊的字符序列，后面跟着解释器的路径。


# 使用OpenCV检测地图中的前沿点。它订阅了地图数据，通过调用 getfrontier 函数获取前沿点，并将其发布出去。
# 这段代码中的 mapCallBack 函数用于接收地图数据，而 node 函数是主要的运行函数，负责检测前沿点并发布。


#--------Include modules---------------
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
# 地图数据的 ROS 消息类型。
from geometry_msgs.msg import PointStamped
from getfrontier import getfrontier

#-----------------------------------------------------
# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
# safety_distance= rospy.get_param('safety_distance',100)

def mapCallBack(data):
    global mapData
    mapData=data
    

    

# Node----------------------------------------------
def node():
		global mapData
		exploration_goal=PointStamped()
		rospy.init_node('detector', anonymous=False)
		#初始化 ROS 节点，命名为 "detector"，并指定为非匿名节点
        
		map_topic= rospy.get_param('~map_topic','/robot_1/map')
        
		rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
		targetspub = rospy.Publisher('/detected_points', PointStamped, queue_size=10)
		# 用于发布检测到的目标点
		pub = rospy.Publisher('shapes', Marker, queue_size=10)
		# 用于发布可视化标记
# wait until map is received, when a map is received, mapData.header.seq will not be < 1
		while mapData.header.seq<1 or len(mapData.data)<1:
			pass
    	   	
		rate = rospy.Rate(50)	


# 创建一个 Marker 类型的对象 points，用于发布前沿点的可视化标记。

		points=Marker()

		#Set the frame ID and timestamp.  See the TF tutorials for information on these.
		points.header.frame_id=mapData.header.frame_id
		points.header.stamp=rospy.Time.now()

		points.ns= "markers"
		points.id = 0

		points.type = Marker.POINTS
		#Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		points.action = Marker.ADD

		points.pose.orientation.w = 1.0
		points.scale.x=points.scale.y=0.3
		points.color.r = 255.0/255.0
		points.color.g = 0.0/255.0
		points.color.b = 0.0/255.0
		points.color.a=1
		points.lifetime == rospy.Duration()
# 设置标记的基本属性，包括坐标系、标记类型、颜色、大小等。




#-------------------------------OpenCV frontier detection------------------------------------------
		while not rospy.is_shutdown():
                     
			# 循环中获取地图中的前沿点
			# frontiers=getfrontier(mapData,safety_distance)
			frontiers=getfrontier(mapData)
			for i in range(len(frontiers)):
				# 遍历每个前沿点，将其封装为 PointStamped 消息，设置坐标和时间戳。

				x=frontiers[i]
				# 获取当前遍历的前沿点的坐标，并将其存储在变量 x 中。

				exploration_goal.header.frame_id= mapData.header.frame_id
				# 设置探索目标点的参考坐标系，通常为地图的坐标系。

				exploration_goal.header.stamp=rospy.Time(0)
				# 设置探索目标点的时间戳，通常为当前时间。

				exploration_goal.point.x=x[0]
				exploration_goal.point.y=x[1]
				exploration_goal.point.z=0	
				# 设置探索目标点的 x 和 y 坐标为当前前沿点的坐标。

				targetspub.publish(exploration_goal)
				# 发布探索目标点，将探索目标点发送到 /detected_points 主题上，供其他节点使用。

				points.points=[exploration_goal.point]
				# 设置用于可视化的标记点，将当前探索目标点添加到 points 中。

				pub.publish(points) 
				# 发布可视化标记点，将标记点发送到 /shapes 主题上，用于在 RViz 或其他可视化工具中显示。
			rate.sleep()
          	
		

	  	#rate.sleep()



#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 