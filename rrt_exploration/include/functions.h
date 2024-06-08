#ifndef functions_H
#define functions_H
#include "ros/ros.h"
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float32.h" // 添加这个包含以识别std_msgs::Float32
// rdm class, for gentaring random flot numbers
class rdm{
int i;
public:
rdm();
float randomize();
};


//Norm function prototype
float Norm( std::vector<float> , std::vector<float> );

//sign function prototype
float sign(float );

//Nearest function prototype
std::vector<float> Nearest(  std::vector< std::vector<float>  > , std::vector<float> );

//Steer function prototype
std::vector<float> Steer(  std::vector<float>, std::vector<float>, float );

//gridValue function prototype
int gridValue(nav_msgs::OccupancyGrid &,std::vector<float>);

//ObstacleFree function prototype
char ObstacleFree(std::vector<float> , std::vector<float> & , nav_msgs::OccupancyGrid);

std::vector<float> calculateRepulsiveForce(float xr, float yr, float xc, float yc, float force_gain);
void exploration_percentage_callback(const std_msgs::Float32::ConstPtr& msg);

#endif