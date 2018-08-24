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
STATUS AstarPlanner::isTraversable(const pose_index &p_index)
{
    if (p_index.x >= this->map_width ||
        p_index.x < 0 ||
        p_index.y >= this->map_height ||
        p_index.y < 0)
        return STATUS::NONE; //在地图之外
    if (this->grid[p_index.y][p_index.x].status == STATUS::OBS)
        return STATUS::OBS; //所在位置是障碍物
    if (this->grid[p_index.y][p_index.x].status == STATUS::CLOSED)
        return STATUS::CLOSED; //所在位置已经被搜索过了
    return STATUS::OPEN;
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

            if (this->map.data[y * this->map_width + x])
                this->grid[y][x].status = STATUS::OBS; //100-false is occupancy，0-true un-occupancy
            else
                this->grid[y][x].status = STATUS::OPEN;
        }
}

/***************** updateGrid******************/
void AstarPlanner::updateGrid(const search_node &node)
{
    this->grid[node.index.y][node.index.x] = node;
}

std::vector<search_node> AstarPlanner::searchNodeVector(const std::vector<search_node> &node_11)
{
    std::vector<search_node> node_vector; //存储初次生成得到的node
    for (const auto &node : node_11)
    {
        for (int i = 0; i < 8; i++)
        {
            search_node temp_node;
            temp_node.index.x = node.index.x + this->directions[i].x;
            temp_node.index.y = node.index.y + this->directions[i].y;

            //首先检查temp_node是否能够通行
            if (this->isTraversable(temp_node.index) != STATUS::OPEN)
                continue;

            if (temp_node.index.x == node.index.x || temp_node.index.y == node.index.y) //如果搜索方向是在竖直方向
                temp_node.g = node.g + 1.0;
            else //如果搜索方向是在对角方向
                temp_node.g = node.g + 1.414;
            //temp_node.g = 0;

            int min_d = abs(temp_node.index.x - this->end_index.x) < abs(temp_node.index.y - this->end_index.y) ? abs(temp_node.index.x - this->end_index.x) : abs(temp_node.index.y - this->end_index.y);
            int max_d = abs(temp_node.index.x - this->end_index.x) > abs(temp_node.index.y - this->end_index.y) ? abs(temp_node.index.x - this->end_index.x) : abs(temp_node.index.y - this->end_index.y);
            temp_node.h = 1.414 * (double)min_d + fabs(max_d - min_d);

            // double d = abs(temp_node.index.x - this->end_index.x) + abs(temp_node.index.y - this->end_index.y);
            // temp_node.h = d;

            // temp_node.h = 0;
            temp_node.g_h = temp_node.g + temp_node.h;
            temp_node.pre_index = node.index;  //节点的前一个节点索引
            temp_node.status = STATUS::CLOSED; //把这个节点设置为不可通过

            node_vector.push_back(temp_node);
        }
    }
    return node_vector;
}
std::vector<search_node> AstarPlanner::getMinGHNode(std::vector<search_node> node_vector)
{
    search_node node = node_vector[0];
    for (const auto &item : node_vector)
        node = node.g_h < item.g_h ? node : item;
    std::vector<search_node> node_vector_;
    node_vector_.push_back(node);
    for (const auto &item : node_vector)
    {
        if (isEqual(node.index, item.index))
            continue;
        if (node.g_h == item.g_h)
            node_vector_.push_back(item);
    }
    return node_vector_;
}
search_node AstarPlanner::reCalGH(const search_node &node)
{
    std::vector<search_node> node_vector;
    for (const auto &item : this->directions)
    {
        pose_index a;
        a.x = node.index.x + item.x;
        a.y = node.index.y + item.y;

        if (this->isTraversable(a) != STATUS::CLOSED)
            continue;

        node_vector.push_back(this->grid[a.y][a.x]); //至少可以搜索到1个计算他的一个父节点
    }
    //重新计算node处的代价
    std::vector<search_node> node_vector_;
    for (const auto &item : node_vector)
    {
        search_node node_;
        node_.index = node.index; //index不会变化

        if (item.index.x == node.index.x || item.index.y == node.index.y) //如果搜索方向是在竖直方向
            node_.g = item.g + 1.0;
        else //如果搜索方向是在对角方向
            node_.g = item.g + 1.414;

        node_.h = node.h; //h值不会变化
        node_.g_h = node_.g + node_.h;

        node_.pre_index = item.index;
        node_.status = STATUS::CLOSED;

        node_vector_.push_back(node_);
    }
    //选择代价最小的node
    search_node node_1 = node_vector_[0];
    for (const auto &item : node_vector_)
        node_1 = node_1.g_h < item.g_h ? node_1 : item;
    return node_1;
}

std::vector<search_node> AstarPlanner::searchNode(const std::vector<search_node> &node_vectors)
//search_node AstarPlanner::searchNode(search_node &node)
{

    std::vector<search_node> node_vector = this->searchNodeVector(node_vectors);
    if (node_vector.size() == 0)
    {
        std::vector<search_node> temp_node_vector;
        for (const auto &item : node_vectors)
            temp_node_vector.push_back(this->grid[item.pre_index.y][item.pre_index.x]);

        return temp_node_vector; //如果无路可走就返回父节点
        // return this->grid[node.pre_index.y][node.pre_index.x]; //如果无路可走就返回父节点
    }

    std::vector<search_node> node_vector_ = this->getMinGHNode(node_vector);

    std::vector<search_node> node_vector_1;
    for (const auto &item : node_vector_)
    {
        search_node node_ = this->reCalGH(item);
        node_vector_1.push_back(node_);
    }
    std::vector<search_node> node_vector_11 = this->getMinGHNode(node_vector_1);
    for (const auto &item : node_vector_11)
        this->updateGrid(item);
    return node_vector_11;

    //选择代价最小的node
    // search_node node_1 = node_vector_1[0];
    // for (const auto &item : node_vector_1)
    //     node_1 = node_1.g_h < item.g_h ? node_1 : item;

    // this->updateGrid(node_1); //更新grid

    // return node_1;
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
    node.status = STATUS::CLOSED;

    this->updateGrid(node); //把起点和终点在grid中进行标记

    std::vector<search_node> node_vector;
    node_vector.push_back(node);

    while (!isEqual(node_vector[0].index, this->end_index)) //搜索点与终点不重合才进行搜索
    {
        std::vector<search_node> node_vector_ = this->searchNode(node_vector);
        node_vector.assign(node_vector_.begin(), node_vector_.end());
    }
    //node = this->searchNode(node);
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