#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.h"
#include "mtrand.h"
// Mersenne Twister随机数生成器的头文件（mtrand.h）


#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>


// 在循环中，通过随机采样生成一个随机点 x_rand，这个点表示地图中的一个自由空间。
// 然后，从当前已知点集 V 中找到与随机点最近的点 x_nearest，这个点是RRT树上的节点。
// 接下来，通过规划算法（Steer函数）在 x_nearest 和 x_rand 之间找到一个新的点 x_new，用于扩展RRT树。
// 然后，检查新点 x_new 是否与障碍物碰撞或未知区域相交（ObstacleFree函数），如果安全则将其加入到已知点集 V 中，并将其与最近点 x_nearest 连接，形成一条路径。
// 如果新点 x_new 是未知区域，则将其作为探测目标发布，并将其存储到 points 容器中进行可视化。
// 如果新点 x_new 是自由空间，将其加入到已知点集 V 中，并将其与最近点 x_nearest 连接形成路径，并将这条路径存储到 line 容器中进行可视化。
// RRT树的第一个枝干是从随机点向用户点击的第五个点（初始探测点）生长的。

// global variables
nav_msgs::OccupancyGrid mapData;
geometry_msgs::PointStamped clickedpoint;
geometry_msgs::PointStamped exploration_goal;
geometry_msgs::PointStamped centroid_point; // 新增的质心坐标
visualization_msgs::Marker points,line;
float xdim,ydim,resolution,Xstartx,Xstarty,init_map_x,init_map_y;

// 这里声明了一些全局变量，用于存储地图数据、点击的点、探索目标、可视化的点和线以及一些尺寸参数。

rdm r; // for genrating random numbers，Mersenne Twister随机数生成器的实例



//Subscribers callback functions---------------------------------------
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
// ConstPtr 可被理解为const pointer（常量指针），根据网页中进而可被理解为shared pointer to a constant message（传递常量数据的共享指针，ps.不理解共享指针没关系，就把它当成一个指针就行）
// & 表示通过引用方式传递，不过我实测，不加& 好像也没有什么影响
// 为什么要用指针而不直接传数据，因为数据太大了，值传递需要进行对数据的复制，而指针引用不需要。

{
mapData=*msg;
}


 
void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{ 

geometry_msgs::Point p;  
p.x=msg->point.x;
p.y=msg->point.y;
p.z=msg->point.z;

points.points.push_back(p);

}
void centroidCallBack(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    centroid_point = *msg; // 存储接收到的质心坐标
}

std::vector<float> calculateRepulsiveForce(float xr, float yr, float xc, float yc, float force_gain_coeff)
{
    // 计算距离
    float distance = sqrt(pow(xr - xc, 2) + pow(yr - yc, 2));
    if (distance < 0.01) // 避免除以零错误
        distance = 0.01;

    // 计算斥力
    float force_x = force_gain_coeff * (xr - xc) / pow(distance, 3);
    float force_y = force_gain_coeff * (yr - yc) / pow(distance, 3);

    return {force_x, force_y};
}


float adjusted_eta = 1.0; // 假设初始eta值为1

// exploration_percentage的回调函数
void exploration_percentage_callback(const std_msgs::Float32::ConstPtr& msg) {
    // 假设探索百分比的20%对应eta的100%，探索百分比的100%对应eta的20%，
    // 使用指数函数来计算这个递减过程。
    // 首先，将探索百分比标准化到0到1之间，其中20%对应0，100%对应1。
    float normalized_exploration = (msg->data - 20.0) / 80.0;

    // 使用指数函数计算eta值，这里0.2可以替换为您希望的最小eta值。
    // 指数衰减的速率由alpha参数控制，您可以根据需要调整这个值。
    // 为了简化计算，我们假设最小eta值为0.2，并且使用以下公式：
    // eta = 1 - (1 - min_eta) * exp(-alpha * normalized_exploration)
    float alpha = 1.0; // 您可以调整这个值来控制衰减的速度
    adjusted_eta =  (1 - 0.2) * exp(-alpha * normalized_exploration);

    // 确保eta值在0.2到1之间
    adjusted_eta = std::max(0.2f, std::min(adjusted_eta, 1.0f));
}



int main(int argc, char **argv)
// argc表示命令行参数的数量，argv是一个指向参数字符串的指针数组。
{
	// 初始化随机数生成器
  unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
  MTRand_int32 irand(init, length); // 32-bit int generator
// this is an example of initializing by an array
// you may use MTRand(seed) with any 32bit integer
// as a seed for a simpler initialization
  MTRand drand; // double in [0, 1) generator, already init

// generate the same numbers as in the original C test program
  ros::init(argc, argv, "global_rrt_frontier_detector");
//   用于初始化ROS节点，第三个参数是节点的名称。
  ros::NodeHandle nh;
//   用于创建一个节点的句柄，用于与ROS系统通信。
 
 
  // fetching all parameters
  float eta,init_map_x,init_map_y,range,force_gain_coeff;
  std::string map_topic,base_frame_topic;
 
  
  std::string ns;
  ns=ros::this_node::getName();

  ros::param::param<float>(ns+"/eta", eta, 0.5);
    ros::param::param<float>(ns+"/force_gain_coeff", force_gain_coeff, 2.0);
  ros::param::param<std::string>(ns+"/map_topic", map_topic, "/robot_1/map"); 

 ros::Subscriber exploration_sub = nh.subscribe("exploration_percentage", 0, exploration_percentage_callback);
//---------------------------------------------------------------



 // 订阅地图和RViz点击点
ros::Subscriber sub= nh.subscribe(map_topic, 100 ,mapCallBack);	
// ros::Subscriber rviz_sub= nh.subscribe("/clicked_point", 100 ,rvizCallBack);	
 ros::Subscriber centroid_sub = nh.subscribe("/explored_centroid", 100, centroidCallBack); // 订阅质心坐标
ros::Publisher targetspub = nh.advertise<geometry_msgs::PointStamped>("/detected_points", 10);
ros::Publisher pub = nh.advertise<visualization_msgs::Marker>(ns+"_shapes", 10);



ros::Rate rate(100); 
 
 
// wait until map is received, when a map is received, mapData.header.seq will not be < 1  
while (mapData.header.seq<1 or mapData.data.size()<1)  {  ros::spinOnce();  ros::Duration(0.1).sleep();}



//visualizations  points and lines..
points.header.frame_id=mapData.header.frame_id;
line.header.frame_id=mapData.header.frame_id;
points.header.stamp=ros::Time(0);
line.header.stamp=ros::Time(0);
	
points.ns=line.ns = "markers";
points.id = 0;
line.id =1;


points.type = points.POINTS;
line.type=line.LINE_LIST;

//Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
points.action =points.ADD;
line.action = line.ADD;
points.pose.orientation.w =1.0;
line.pose.orientation.w = 1.0;
line.scale.x =  0.03;
line.scale.y= 0.03;
points.scale.x=0.3; 
points.scale.y=0.3; 

line.color.r =9.0/255.0;
line.color.g= 91.0/255.0;
line.color.b =236.0/255.0;
points.color.r = 255.0/255.0;
points.color.g = 0.0/255.0;
points.color.b = 0.0/255.0;
points.color.a=1.0;
line.color.a = 1.0;
points.lifetime = ros::Duration();
line.lifetime = ros::Duration();
// 这部分代码初始化了用于在RViz中可视化的points和line变量。它们将用于显示探测到的点和连线。

// 这段代码在RViz中显示初始的5个点，以便用户可以点击以选择探测区域。
//points.points[0]：这个点是 points 容器中的第一个点，代表RViz中点击的第一个点。它的坐标被用来计算地图的初始长度和宽度。
// points.points[1]：在这段代码中并没有直接使用这个点。
// points.points[2]：这个点是 points 容器中的第三个点，代表RViz中点击的第二个点。它的坐标被用来计算地图的初始长度和宽度。
// points.points[3]：在这段代码中并没有直接使用这个点。
// points.points[4]：这个点是 points 容器中的最后一个点，也是RViz中点击的最后一个点。它的坐标被存储在 V 中，作为初始探测点。
geometry_msgs::Point p;  

double preLoad_points[6] = {0};


preLoad_points[0]=7.4266166687;
preLoad_points[1]=4.85690784454;
// 左上角的点

preLoad_points[2]=-7.08084964752;
preLoad_points[3]=-6.00671339035;
//  右下角的点


preLoad_points[4]= 4.48516654968;
preLoad_points[5]=2.46182155609;
// 第一个点击的点

// while(points.points.size()<3)
// {
// ros::spinOnce();

// pub.publish(points) ;
// }



// 这段代码计算了地图的初始参数，包括地图的长度、宽度和起始坐标。
// std::vector<float> temp1;
// temp1.push_back(points.points[0].x);
// temp1.push_back(points.points[0].y);
	
// std::vector<float> temp2; 
// temp2.push_back(points.points[2].x);
// temp2.push_back(points.points[0].y);


// init_map_x=Norm(temp1,temp2);
// temp1.clear();		temp2.clear();

// temp1.push_back(points.points[0].x);
// temp1.push_back(points.points[0].y);

// temp2.push_back(points.points[0].x);
// temp2.push_back(points.points[2].y);

std::vector<float> temp1;
temp1.push_back(preLoad_points[0]);
temp1.push_back(preLoad_points[1]);
	
std::vector<float> temp2; 
temp2.push_back(preLoad_points[2]);
temp2.push_back(preLoad_points[0]);


init_map_x=Norm(temp1,temp2);
temp1.clear();		temp2.clear();

temp1.push_back(preLoad_points[0]);
temp1.push_back(preLoad_points[1]);

temp2.push_back(preLoad_points[0]);
temp2.push_back(preLoad_points[3]);

init_map_y=Norm(temp1,temp2);
//init_map_y和init_map_x分别代表初始框出来的方框的长宽尺寸，是由第一个点和第三个点的坐标计算出来的
temp1.clear();		temp2.clear();

// Xstartx=(points.points[0].x+points.points[2].x)*.5;
// Xstarty=(points.points[0].y+points.points[2].y)*.5;

//start点处于正中心，不同于用户点击的第五个点
Xstartx=(preLoad_points[0]+preLoad_points[2])*.5;
Xstarty=(preLoad_points[1]+preLoad_points[3])*.5;


// 这段代码将RViz中点击的最后一个点作为初始探测点，并将其存储在V中。同时清空之前收集的所有点击点。
geometry_msgs::Point trans;
// trans=points.points[4];
trans.x=preLoad_points[4];
trans.y=preLoad_points[5];
std::vector< std::vector<float>  > V; 
std::vector<float> xnew; 
xnew.push_back( trans.x);
xnew.push_back( trans.y);  
V.push_back(xnew);

points.points.clear();
pub.publish(points) ;


// std::vector<float> frontiers;
int i=0;
float xr,yr;
std::vector<float> x_rand,x_nearest,x_new;
//Xstart:用户点击的第五个点
//x_rand:随机生成的点，用来模拟在地图中的自由采样,由地图尺寸计算得到
// x_nearest：表示与 x_rand 最近的点，用来模拟在地图中找到最接近随机采样点的现有点。
// 				在代码中，通过调用 Nearest(V, x_rand) 函数来找到最近的点，其中 V 是存储点的向量，
// x_new:表示根据 x_nearest 和 x_rand 进行规划后得到的新点，用来模拟在地图中规划出的下一个探索点。




// Main loop
while (ros::ok()){
// 在这里使用质心坐标
        float X_centroid, Y_centroid;
        if (centroid_point.header.stamp.isZero())
        {
            // 如果没有收到质心的消息，则按照原来的方式计算
            X_centroid = 0.0; // 这里放置原来的计算方式
            Y_centroid = 0.0; // 这里放置原来的计算方式
        }
        else
        {
            X_centroid = centroid_point.point.x; // 从接收到的消息中获取质心x坐标
            Y_centroid = centroid_point.point.y; // 从接收到的消息中获取质心y坐标
        }







// Sample free 这段代码生成一个随机点 (xr, yr)，以进行自由空间采样。
x_rand.clear();//x_rand.clear() 清空 x_rand 向量，以便重新存储新的随机点。

xr=(drand()*init_map_x)-(init_map_x*0.5)+Xstartx;
yr=(drand()*init_map_y)-(init_map_y*0.5)+Xstarty;
// [-地图长度的一半+起始点的横/纵坐标，地图长度的一半+起始点的横/纵坐标]
// 随机点更有可能分布在地图中心附近



std::vector<float> repulsive_force = calculateRepulsiveForce(xr, yr, X_centroid, Y_centroid, force_gain_coeff);
        xr += repulsive_force[0];
        yr += repulsive_force[1];



x_rand.push_back( xr ); x_rand.push_back( yr );


// Nearest
//V里面目前的点是Xnew，来自用户点击的第五个点，但是实际上V是一组点
//Nearest函数会遍历V找到其中与x_rand最近的点，将其赋值给x_nearest
x_nearest=Nearest(V,x_rand);



// Steer



eta=adjusted_eta;
x_new=Steer(x_nearest,x_rand,eta);
//eta是规划参数,如果rand和nearest点之间距离小于eta,rand直接作为下一个点
//如果大于,进行插值,在 x_nearest 和 x_rand 之间按照一定比例插值，以使得 x_new 落在两点连线上，并且与 x_nearest 之间的距离为规划参数 eta
//如果rand和nearest重合,则直接将新点的横坐标设为 x_nearest[0]，纵坐标加上规划参数 eta。
// ROS_INFO("Adjusted eta: %f", adjusted_eta);


// ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
char   checking=ObstacleFree(x_nearest,x_new,mapData);
// 如果存在未知区域，则返回 -1，表示这个区域是未知的。
// 如果存在障碍物，则返回 0，表示这个区域是有障碍物的。
// 如果没有遇到障碍物或未知区域，则返回 1，表示这个区域是自由空间的
	  if (checking==-1){
          	exploration_goal.header.stamp=ros::Time(0);
          	exploration_goal.header.frame_id=mapData.header.frame_id;
			// 首先，设置探索目标点的时间戳和参考坐标系，这里时间戳设置为 0 表示立即执行。
          	exploration_goal.point.x=x_new[0];
          	exploration_goal.point.y=x_new[1];
          	exploration_goal.point.z=0.0;
			// 然后，将探索目标点的坐标设置为 x_new，即新点的坐标
          	p.x=x_new[0]; 
			p.y=x_new[1]; 
			p.z=0.0;
			// 接着，将这个探索目标点加入到 points 容器中，以便后续可视化。
          	points.points.push_back(p);
          	pub.publish(points) ;
          	targetspub.publish(exploration_goal);
		  	points.points.clear();
			// 将更新后的 points 发布出去，以便 RViz 可以显示出来。
			// 最后，将探索目标点发布到 targetspub 主题上，通知其他节点。
        	
        	}
	  	
	  
	  else if (checking==1){

// 		首先，将新点 x_new 加入到 V 容器中，表示这个点是安全的探索点。
// 		然后，将这个点以及与之最近的点 x_nearest 加入到 line 容器中，以便后续可视化。这里将它们作为一条线段的端点。
// 		将更新后的 line 发布出去，以便 RViz 可以显示出来。
	 	V.push_back(x_new);
	 	
	 	p.x=x_new[0]; 
		p.y=x_new[1]; 
		p.z=0.0;
	 	line.points.push_back(p);
	 	p.x=x_nearest[0]; 
		p.y=x_nearest[1]; 
		p.z=0.0;
	 	line.points.push_back(p);

	        }

//-1则变为目标点,1则加入树,即使全部为1(地图探索完成)树仍然继续生长,Xnew的值一直是上一个Xnew的值

pub.publish(line);  


   

ros::spinOnce();
rate.sleep();
  }return 0;}
