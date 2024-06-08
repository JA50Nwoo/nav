#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------Include modules---------------
from copy import copy
import rospy
from nav_msgs.msg import OccupancyGrid

import numpy as np
import cv2

#-----------------------------------------------------

def getfrontier(mapData):

# """
#     获取地图的前沿点坐标
#     Args:
#         mapData (OccupancyGrid): 输入的地图数据
#     Returns:
#         all_pts (list): 包含所有前沿点坐标的列表
#     """


#读取地图的各种信息
	data=mapData.data
	w=mapData.info.width
	h=mapData.info.height
	resolution=mapData.info.resolution
	Xstartx=mapData.info.origin.position.x
	Xstarty=mapData.info.origin.position.y


	 # 创建一个与地图大小相同的空白图像
	img = np.zeros((h, w, 1), np.uint8)
	 # 遍历地图数据，根据不同的值赋予不同的像素值
	for i in range(0,h):
		for j in range(0,w):
			# if data[i*w+j]!=100:
			# 	img[i,j]=0# 占据状态
			if data[i*w+j]==0:
				img[i,j]=255# 自由状态
			elif data[i*w+j]==-1:
				img[i,j]=205# 未知状态
			else: img[i,j]=0

	
	
	obstacles=cv2.inRange(img,0,1) 
	# 这个操作创建了一个二值图像 o，其中所有值为0的像素（代表未占据状态）被设置为0，所有其他像素（代表占据状态）被设置为1。这样，我们就得到了一个突出显示占据状态像素的图像。
	edges = cv2.Canny(img,0,255,apertureSize=2,L2gradient=True) 
	# 使用 Canny 算法对原始图像 img 进行边缘检测，生成边缘图像 edges。Canny 算法会突出显示图像中的边缘，即使在边缘周围存在噪声的情况下。

 	
	im2, contours, hierarchy = cv2.findContours(o,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	# 在这个步骤中，使用 cv2.findContours 函数在二值图像 o 中寻找所有轮廓。cv2.RETR_TREE 指定了轮廓检索的模式，它保留了轮廓的层次关系。cv2.CHAIN_APPROX_SIMPLE 用于对轮廓点进行压缩，只保留轮廓的拐点。
	cv2.drawContours(o, contours, -1, (255,255,255), 50)
	# 这个函数在图像 o 上绘制轮廓，-1 表示绘制所有轮廓，颜色为白色，线条粗细为5。
	anti_obstacles = cv2.bitwise_not(obstacles)
	res = cv2.bitwise_and(anti_obstacles, edges)
	frontier_1=copy(res)
	# 这个操作将图像 o 取反，将原本的占据状态（白色）转换为黑色，未占据状态（黑色）转换为白色。
	im2, contours, hierarchy = cv2.findContours(frontier_1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(frontier_1, contours, -1, (255,255,255), 2)
	im2, contours, hierarchy = cv2.findContours(frontier_1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	
	 # 存储所有前沿点的坐标
	all_pts=[]

	# 如果存在前沿轮廓
	if len(contours)>0:  
		# 遍历所有轮廓
		upto=len(contours)-1
		i=0
		maxx=0
		maxind=0
		

	#这里得到的返回的pts是每个前沿的质心，代表了边界的中心位置
		for i in range(0,len(contours)):
				cnt = contours[i]# 获取当前轮廓
				M = cv2.moments(cnt)# 计算轮廓的矩
				cx = int(M['m10']/M['m00'])# 计算质心x坐标
				cy = int(M['m01']/M['m00']) # 计算质心y坐标
				# 转换成实际地图坐标
				xr=cx*resolution+Xstartx
				yr=cy*resolution+Xstarty
				pt=[np.array([xr,yr])]# 将坐标存入数组中
				if len(all_pts)>0:
					all_pts=np.vstack([all_pts,pt]) # 将坐标数组堆叠起来
				else:
							
					all_pts=pt# 如果是第一个坐标，直接赋值
	
	return all_pts# 返回所有前沿点的坐标


