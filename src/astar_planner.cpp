#include "astar/astar_planner.h"

/****************constructor**********************/
AstarPlanner::AstarPlanner()
{
    this->map_flag = false;
    this->start_flag = false;
    this->end_flag = false;
    this->grid = nullptr; //set pointer to null

    this->directions[0].x = 0;
    this->directions[0].y = 1;
    this->directions[1].x = 0;
    this->directions[1].y = -1;
    this->directions[2].x = 1;
    this->directions[2].y = 0;
    this->directions[3].x = -1;
    this->directions[3].y = 0;

    //topic to subscribe
    this->sub_map = this->n.subscribe("/map", 1, &AstarPlanner::callbackMap, this);
    this->sub_start_pose = this->n.subscribe("/move_base_simple/start", 1, &AstarPlanner::callbackStart, this);
    this->sub_end_pose = this->n.subscribe("/move_base_simple/goal", 1, &AstarPlanner::callbackEnd, this);

    //topic to publish
    this->path_pub = this->n.advertise<nav_msgs::Path>("/2D_path", 1);
}

/*****************destructor*******************/
AstarPlanner::~AstarPlanner()
{
    if (!(this->grid == nullptr)) //reclaim the memory in destructor
    {
        for (int i = 0; i < this->map_height; i++)
        {
            delete this->grid[i];
        }
        delete this->grid;
    }
}

void AstarPlanner::callbackMap(const nav_msgs::OccupancyGrid &map_msg)
{
    this->map = map_msg;
    this->map_resolution = map_msg.info.resolution;
    this->map_width = map_msg.info.width;
    this->map_height = map_msg.info.height;
    //printf("map_width = %d,map_height = %d\n", map_width, map_height);
    this->map_flag = true;
    this->plan();
}

void AstarPlanner::callbackStart(const geometry_msgs::PoseStamped &start_msg)
{
    if (!this->map_flag)
        return;
    if ((start_msg.pose.position.x - this->map.info.origin.position.x) < 0 ||
        (start_msg.pose.position.x - this->map.info.origin.position.x) >= (this->map_width * this->map_resolution) ||
        (start_msg.pose.position.y - this->map.info.origin.position.y) < 0 ||
        (start_msg.pose.position.y - this->map.info.origin.position.y) >= (this->map_height * this->map_resolution))
    {
        printf("start pose is not in map !\n");
        this->start_flag = false;
        return;
    }

    pose_index temp_index;
    temp_index.x = int((start_msg.pose.position.x - this->map.info.origin.position.x) / this->map.info.resolution);
    temp_index.y = int((start_msg.pose.position.y - this->map.info.origin.position.y) / this->map.info.resolution);

    if (this->map.data[temp_index.y * this->map_width + temp_index.x])
    {
        printf("start pose is in obs !\n");
        this->start_flag = false;
        return;
    }

    this->start_pose = start_msg;
    this->start_index = temp_index;
    this->start_flag = true;
    this->plan();
}

void AstarPlanner::callbackEnd(const geometry_msgs::PoseStamped &end_msg)
{
    if (!this->map_flag)
        return;
    if ((end_msg.pose.position.x - this->map.info.origin.position.x) < 0 ||
        (end_msg.pose.position.x - this->map.info.origin.position.x) >= (this->map_width * this->map_resolution) ||
        (end_msg.pose.position.y - this->map.info.origin.position.y) < 0 ||
        (end_msg.pose.position.y - this->map.info.origin.position.y) >= (this->map_height * this->map_resolution))
    {
        printf("end pose is not in map !\n");
        this->end_flag = false;
        return;
    }

    pose_index temp_index;
    temp_index.x = int((end_msg.pose.position.x - this->map.info.origin.position.x) / this->map.info.resolution);
    temp_index.y = int((end_msg.pose.position.y - this->map.info.origin.position.y) / this->map.info.resolution);
    if (this->map.data[temp_index.y * this->map_width + temp_index.x])
    {
        printf("end pose is in obs !\n");
        this->end_flag = false;
        return;
    }
    //printf("end_index.x = %d,end_index.y = %d\n", end_index.x, end_index.y);

    this->end_pose = end_msg;
    this->end_index = temp_index;
    this->end_flag = true;
    this->plan();
}

/********************* detect astar search point ************************/
bool AstarPlanner::isTraversable(const pose_index &p_index)
{
    if (p_index.x >= this->map_width ||
        p_index.x < 0 ||
        p_index.y >= this->map_height ||
        p_index.y < 0)
        return false;
    return this->grid[p_index.y][p_index.x];
}

/******************initial grid at the begining of every search************/
void AstarPlanner::initialGrid()
{
    if (!(this->grid == nullptr)) //if pointer is not null,reclaim memory before use
    {
        for (int i = 0; i < this->map_height; i++)
        {
            delete this->grid[i];
        }
        delete this->grid;
    }

    this->grid = new bool *[this->map_height];
    for (int x = 0; x < this->map_height; x++)
        this->grid[x] = new bool[this->map_width];

    for (int y = 0; y < this->map_height; ++y)
        for (int x = 0; x < this->map_width; ++x)
            this->grid[y][x] = this->map.data[y * this->map_width + x] ? false : true; //100-false is occupancy，0-true un-occupancy
}

/***************** updateGrid******************/
void AstarPlanner::updateGrid(const pose_index &p_index)
{
    this->grid[p_index.y][p_index.x] = false;
}

/************search next node and retrun the node***********/
search_node AstarPlanner::searchNextNode(const search_node &pre_node) //can't mod param in function
{
    if (isEqual(pre_node.index, this->end_index))
    {
        printf("astar search is finished !");
        return pre_node;
    }
    std::vector<search_node> temp_node_vector;
    search_node temp_node;

    // for (int i = 0; i < 4; i++)
    // {
    //     pose_index temp_index = pre_node.index;
    //     temp_index.x += this->directions[i].x;
    //     temp_index.y += this->directions[i].y;
    //     if (!this->isTraversable(temp_index)) //如果不能通过
    //         continue;

    //     temp_node.index = temp_index;
    //     temp_node.g = pre_node.g + 1.0;
    //     temp_node.h = abs(temp_index.x - end_index.x) + abs(temp_index.y - end_index.y);
    //     temp_node.g_h = temp_node.g + temp_node.h;
    //     temp_node_vector.push_back(temp_node);
    // }
    for (const auto &item : this->directions)
    {
        pose_index temp_index = pre_node.index;
        temp_index.x += item.x;
        temp_index.y += item.y;
        if (!this->isTraversable(temp_index)) //如果不能通过
            continue;

        temp_node.index = temp_index;
        temp_node.g = pre_node.g + 1.0;
        temp_node.h = abs(temp_index.x - end_index.x) + abs(temp_index.y - end_index.y);
        temp_node.g_h = temp_node.g + temp_node.h;
        temp_node_vector.push_back(temp_node);
    }
    if (temp_node_vector.size() == 0)
    {
        printf("astar search is error !\n");
        return pre_node;
    }

    temp_node = temp_node_vector[0];
    for (int i = 0; i < (int)temp_node_vector.size(); i++)
    {
        if (temp_node.g_h > temp_node_vector[i].g_h)
            temp_node = temp_node_vector[i];
    }

    return temp_node;
}

void AstarPlanner::vectorToPath()
{
    this->path.poses.clear(); //clear the vector before use

    this->path.header.stamp = ros::Time::now();
    this->path.header.frame_id = this->map.header.frame_id;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = this->map.header.frame_id; //frame_id
    pose_stamped.pose.position.z = 0;
    pose_stamped.pose.orientation.x = 0;
    pose_stamped.pose.orientation.y = 0;
    pose_stamped.pose.orientation.z = 0;
    pose_stamped.pose.orientation.w = 1;
    // for (int i = 0; i < (int)this->path_vector.size(); i++)
    // {
    //     pose_stamped.header.stamp = ros::Time::now(); //stamp
    //     pose_stamped.header.seq = i;                  //seq

    //     pose_stamped.pose.position.x = this->path_vector[i].x * this->map_resolution +
    //                                    this->map.info.origin.position.x +
    //                                    this->map_resolution / 2;
    //     pose_stamped.pose.position.y = this->path_vector[i].y * this->map_resolution +
    //                                    this->map.info.origin.position.y +
    //                                    this->map_resolution / 2;

    //     this->path.poses.push_back(pose_stamped);
    // }

    int i = 0;
    for (const auto &item : this->path_vector)
    {
        pose_stamped.header.stamp = ros::Time::now(); //stamp
        pose_stamped.header.seq = i;                  //seq
        i++;

        pose_stamped.pose.position.x = item.x * this->map_resolution +
                                       this->map.info.origin.position.x +
                                       this->map_resolution / 2;
        pose_stamped.pose.position.y = item.y * this->map_resolution +
                                       this->map.info.origin.position.y +
                                       this->map_resolution / 2;
        this->path.poses.push_back(pose_stamped);
    }
}

void AstarPlanner::publishPath()
{
    if (this->path.poses.size() <= 0)
    {
        printf("no path !\n");
        return;
    }
    this->path_pub.publish(this->path);
}

void AstarPlanner::astar()
{
    this->initialGrid();
    this->updateGrid(this->start_index); //把起点和终点在grid中进行标记
    //this->updateGrid(this->end_index);不能标记终点

    this->path_vector.clear(); //clear the vector before use
    this->path_vector.push_back(this->start_index);

    search_node node; //start_search_node
    node.index = this->start_index;
    node.g = 0;
    node.h = abs(start_index.x - end_index.x) + abs(start_index.y - end_index.y);
    node.g_h = node.g + node.h;

    while (!isEqual(node.index, this->end_index))
    {
        search_node node_temp = this->searchNextNode(node);

        if (isEqual(node, node_temp))
            break;

        node = node_temp;
        path_vector.push_back(node.index); //node add to path_vector
        this->updateGrid(node.index);      //update Grid
    }
}

void AstarPlanner::plan()
{
    if (!this->map_flag)
    {
        printf("no map !\n");
        return;
    }
    if (!this->start_flag)
    {
        printf("no start point !\n");
        return;
    }
    if (!this->end_flag)
    {
        printf("no end point !\n");
        return;
    }

    this->astar();
    this->vectorToPath();
    this->publishPath();
    return;
}