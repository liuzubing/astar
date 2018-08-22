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
    this->directions[4].x = -1;
    this->directions[4].y = -1;
    this->directions[5].x = 1;
    this->directions[5].y = -1;
    this->directions[6].x = -1;
    this->directions[6].y = 1;
    this->directions[7].x = 1;
    this->directions[7].y = 1;

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
    return this->grid[p_index.y][p_index.x].is_pass;
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

    this->grid = new search_node *[this->map_height];
    for (int x = 0; x < this->map_height; x++)
        this->grid[x] = new search_node[this->map_width];

    for (int y = 0; y < this->map_height; ++y)
        for (int x = 0; x < this->map_width; ++x)
        {
            this->grid[y][x].index.x = x;
            this->grid[y][x].index.y = y;
            this->grid[y][x].g = -1;
            this->grid[y][x].h = -1;
            this->grid[y][x].g_h = -1;
            this->grid[y][x].pre_index.x = -1;
            this->grid[y][x].pre_index.y = -1;
            this->grid[y][x].is_pass = this->map.data[y * this->map_width + x] ? false : true; //100-false is occupancy，0-true un-occupancy
        }
}

/***************** updateGrid******************/
void AstarPlanner::updateGrid(const search_node &node)
{
    this->grid[node.index.y][node.index.x] = node;
}

/************search next node and retrun the node***********/
std::vector<search_node> AstarPlanner::searchNode(const std::vector<search_node> &node_vector)
{
    std::vector<search_node> multi_temp_node_vector;
    for (const auto &item : node_vector)
    {
        for (int i = 0; i < 8; i++)
        {
            search_node temp_node = item;
            temp_node.index.x += this->directions[i].x;
            temp_node.index.y += this->directions[i].y;

            //首先检查temp_node是否能够通行
            if (!this->isTraversable(temp_node.index))
                continue;

            if (temp_node.index.x == item.index.x || temp_node.index.y == item.index.y) //如果搜索方向是在竖直方向
                temp_node.g = item.g + 1.0;
            else //如果搜索方向是在对角方向
                temp_node.g = item.g + 1.414;

            int min_d = abs(temp_node.index.x - this->end_index.x) < abs(temp_node.index.y - this->end_index.y) ? abs(temp_node.index.x - this->end_index.x) : abs(temp_node.index.y - this->end_index.y);
            int max_d = abs(temp_node.index.x - this->end_index.x) > abs(temp_node.index.y - this->end_index.y) ? abs(temp_node.index.x - this->end_index.x) : abs(temp_node.index.y - this->end_index.y);
            temp_node.h = 1.414 * (double)min_d + fabs(max_d - min_d);

            temp_node.g_h = temp_node.g + temp_node.h;
            temp_node.pre_index = item.index; //节点的前一个节点索引
            temp_node.is_pass = false;        //把这个节点设置为不可通过

            multi_temp_node_vector.push_back(temp_node);
        }
    }

    std::vector<search_node> temp_node_vector;
    if (multi_temp_node_vector.size() == 0)
        return temp_node_vector;

    temp_node_vector.push_back(multi_temp_node_vector[0]);
    for (int i = 0; i < (int)multi_temp_node_vector.size(); i++)
    {
        //首先寻找item在temp_node_vector中是否存在,如果不存在,就添加,否则就和原来的item_比较选择代价最低的
        if (isInVector(multi_temp_node_vector[i], temp_node_vector))
        {
            for (auto &item_ : temp_node_vector)
            {
                if (isEqual(multi_temp_node_vector[i].index, item_.index)) //如果是同一个网格,就选择代价最低的
                {
                    item_ = (multi_temp_node_vector[i].g_h < item_.g_h ? multi_temp_node_vector[i] : item_);
                }
            }
        }
        else
            temp_node_vector.push_back(multi_temp_node_vector[i]);
    }

    for (const auto &item : temp_node_vector)
        this->updateGrid(item); //update Grid

    return temp_node_vector;
}

void AstarPlanner::getPathFromGrid()
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

    int i = 0;
    pose_index index_ = this->end_index;
    while (index_.x != -1 && index_.y != -1)
    {
        pose_stamped.header.stamp = ros::Time::now(); //stamp
        pose_stamped.header.seq = i;                  //seq
        i++;

        pose_stamped.pose.position.x = this->grid[index_.y][index_.x].index.x * this->map_resolution +
                                       this->map.info.origin.position.x +
                                       this->map_resolution / 2;
        pose_stamped.pose.position.y = this->grid[index_.y][index_.x].index.y * this->map_resolution +
                                       this->map.info.origin.position.y +
                                       this->map_resolution / 2;

        this->path.poses.push_back(pose_stamped); //得到的path是从end点到start点的顺序,记得将容器逆序

        index_ = this->grid[index_.y][index_.x].pre_index;
    }

    //std::reverse(std::begin(this->path.poses),std::end(this->path.poses));//将路径逆序从start开始  //或者在搜索路径的时候从end点往回搜索,就不用逆序了
    std::reverse(this->path.poses.begin(), this->path.poses.end()); //将路径逆序从start开始  //或者在搜索路径的时候从end点往回搜索,就不用逆序了
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
    this->initialGrid(); //首先对grid初始化

    search_node node; //start_search_node
    node.index = this->start_index;
    node.g = 0;
    node.h = abs(start_index.x - end_index.x) + abs(start_index.y - end_index.y);
    node.g_h = node.g + node.h;
    node.pre_index.x = -1;
    node.pre_index.y = -1;
    node.is_pass = false;

    this->updateGrid(node); //把起点和终点在grid中进行标记

    std::vector<search_node> node_vector;
    node_vector.push_back(node);

    search_node end_node; //end_node用于检测搜索结果中是否含有终点,如果含有就停止搜索,否则继续(避免全部搜索,节省时间)
    end_node.index = this->end_index;

    while (node_vector.size() != 0)
    {
        std::vector<search_node> temp_vector(searchNode(node_vector)); //计算搜索结果
        if (isInVector(end_node, temp_vector))                         //如果搜索结果中已经含有end node,则停止搜索
            break;
        node_vector.assign(temp_vector.begin(), temp_vector.end()); //赋值给node_vector进行循环结构的判断
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

    ros::Time t0 = ros::Time::now();

    this->astar();

    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;

    this->getPathFromGrid();
    this->publishPath();

    return;
}