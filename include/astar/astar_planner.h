#ifndef ASTAR_PLANNER
#define ASTAR_PLANNER

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "constants.h" //和当前文件在相同的路径下
//#include "node.h"
#include <vector>

class AstarPlanner
{
public:
  AstarPlanner();  //constructor
  ~AstarPlanner(); //destructor

  void callbackMap(const nav_msgs::OccupancyGrid &map_msg);
  void callbackStart(const geometry_msgs::PoseStamped &start_msg);
  void callbackEnd(const geometry_msgs::PoseStamped &end_msg);
  void updateGrid(const pose_index &p_index);              //更新网格地图（有新添加进来的）
  void initialGrid();                                      //每次规划初始化网格地图
  bool isTraversable(const pose_index &p_index);           //当前搜索点是否可以通过
  search_node searchNextNode(const search_node &pre_node); //给出当前点，搜索返回下一个
  void vectorToPath();
  void publishPath();
  void astar();
  void plan(); //开始规划，并且发布地图/path

  /******get method******/
  nav_msgs::OccupancyGrid getMap() { return map; }
  geometry_msgs::PoseStamped getStartPpose() { return start_pose; };
  geometry_msgs::PoseStamped getEndPose() { return end_pose; };
  nav_msgs::Path getPath() { return path; };

  double getMapWidth() { return map_width; };
  double getMapHeight() { return map_height; };
  pose_index getStartPoint() { return start_index; };
  pose_index getEndPoint() { return end_index; };

private:
  ros::NodeHandle n;
  ros::Subscriber sub_map;
  ros::Subscriber sub_start_pose;
  ros::Subscriber sub_end_pose;
  ros::Publisher path_pub;

  nav_msgs::OccupancyGrid map;           //地图
  bool **grid;                           //网格
  geometry_msgs::PoseStamped start_pose; //起点
  geometry_msgs::PoseStamped end_pose;   //终点

  nav_msgs::Path path;                 //路径
  std::vector<pose_index> path_vector; //路径——容器

  int map_width, map_height;         //地图的长度和宽度
  double map_resolution;             //地图的分辨率
  pose_index start_index, end_index; //起点和终点网格索引

  bool map_flag, start_flag, end_flag; //是否已经接收到地图flag
  pose_index directions[4];            //搜索方向
};

#endif