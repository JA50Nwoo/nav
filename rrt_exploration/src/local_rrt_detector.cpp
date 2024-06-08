#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.h"
#include "mtrand.h"


#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>



// global variables
nav_msgs::OccupancyGrid mapData;
geometry_msgs::PointStamped clickedpoint;
geometry_msgs::PointStamped exploration_goal;
geometry_msgs::PointStamped centroid_point; // 新增的质心坐标
visualization_msgs::Marker points,line;
float xdim,ydim,resolution,Xstartx,Xstarty,init_map_x,init_map_y;

rdm r; // for genrating random numbers



//Subscribers callback functions---------------------------------------
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
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
    adjusted_eta = (1 - 0.2) * exp(-alpha * normalized_exploration);

    // 确保eta值在0.2到1之间
    adjusted_eta = std::max(0.2f, std::min(adjusted_eta, 1.0f));
}



int main(int argc, char **argv)
{

  unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
  MTRand_int32 irand(init, length); // 32-bit int generator
// this is an example of initializing by an array
// you may use MTRand(seed) with any 32bit integer
// as a seed for a simpler initialization
  MTRand drand; // double in [0, 1) generator, already init

// generate the same numbers as in the original C test program
  ros::init(argc, argv, "local_rrt_frontier_detector");
  ros::NodeHandle nh;
  
  // fetching all parameters
  float eta,init_map_x,init_map_y,range,force_gain_coeff;
  std::string map_topic,base_frame_topic;
  
  std::string ns;
  ns=ros::this_node::getName();

  ros::param::param<float>(ns+"/eta", eta, 0.5);
      ros::param::param<float>(ns+"/force_gain_coeff", force_gain_coeff, 2.0);
  ros::param::param<std::string>(ns+"/map_topic", map_topic, "/robot_1/map"); 
  ros::param::param<std::string>(ns+"/robot_frame", base_frame_topic, "/robot_1/base_link"); 
//---------------------------------------------------------------
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

line.color.r =255.0/255.0;
line.color.g= 0.0/255.0;
line.color.b =0.0/255.0;
points.color.r = 255.0/255.0;
points.color.g = 0.0/255.0;
points.color.b = 0.0/255.0;
points.color.a=0.3;
line.color.a = 1.0;
points.lifetime = ros::Duration();
line.lifetime = ros::Duration();

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


// points.points[0].x=7.4266166687;
// points.points[0].y=4.85690784454;


// points.points[2].x=-7.08084964752;
// points.points[2].y=-6.00671339035;
 


// points.points[4].x= 4.48516654968;
// points.points[4].y=2.46182155609;


// while(points.points.size()<3)
// {
// ros::spinOnce();

// pub.publish(points) ;
// }



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

init_map_y=Norm(temp1,temp2);
temp1.clear();		temp2.clear();

// Xstartx=(points.points[0].x+points.points[2].x)*.5;
// Xstarty=(points.points[0].y+points.points[2].y)*.5;


Xstartx=(preLoad_points[0]+preLoad_points[2])*.5;
Xstarty=(preLoad_points[1]+preLoad_points[3])*.5;



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

tf::TransformListener listener;
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



// Sample free
x_rand.clear();
xr=(drand()*init_map_x)-(init_map_x*0.5)+Xstartx;
yr=(drand()*init_map_y)-(init_map_y*0.5)+Xstarty;

std::vector<float> repulsive_force = calculateRepulsiveForce(xr, yr, X_centroid, Y_centroid,force_gain_coeff);
        xr += repulsive_force[0];
        yr += repulsive_force[1];



x_rand.push_back( xr ); x_rand.push_back( yr );


// Nearest
x_nearest=Nearest(V,x_rand);

// Steer


eta=adjusted_eta;

x_new=Steer(x_nearest,x_rand,eta);


// ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
char   checking=ObstacleFree(x_nearest,x_new,mapData);

	  if (checking==-1){

			exploration_goal.header.stamp=ros::Time(0);
          	exploration_goal.header.frame_id=mapData.header.frame_id;
          	exploration_goal.point.x=x_new[0];
          	exploration_goal.point.y=x_new[1];
          	exploration_goal.point.z=0.0;
          	p.x=x_new[0]; 
			p.y=x_new[1]; 
			p.z=0.0;
					
          	points.points.push_back(p);
          	pub.publish(points) ;
          	targetspub.publish(exploration_goal);
		  	points.points.clear();



			
		  	V.clear();
		  	
		  	
			tf::StampedTransform transform;
			int  temp=0;
			while (temp==0){
			try{
			temp=1;
			listener.lookupTransform(map_topic, base_frame_topic , ros::Time(0), transform);
			}
			catch (tf::TransformException ex){
			temp=0;
			ros::Duration(0.1).sleep();
			}}
			
			x_new[0]=transform.getOrigin().x();
			x_new[1]=transform.getOrigin().y();
        	V.push_back(x_new);
        	line.points.clear();
        	}
	  	
	  
	  else if (checking==1){
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



pub.publish(line);  


   

ros::spinOnce();
rate.sleep();
  }return 0;}
