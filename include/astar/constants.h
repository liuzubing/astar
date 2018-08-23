#ifndef CONSTANTS
#define CONSTANTS
#include <iostream>
#include <vector>

enum class STATUS : uint8_t//enum class
{
  NONE,
  OPEN,//空白
  CLOSED,//路径
  OBS//障碍
};

struct pose_index
{
    int x;
    int y;
};

struct search_node
{
    pose_index index;
    double g;
    double h;
    double g_h;
    pose_index pre_index;
    //search_node pre_node;
    STATUS status = STATUS::NONE;
};

inline static bool isEqual(const pose_index &a, const pose_index &b)
{   
    return (a.x == b.x && a.y == b.y) ? true : false;
}

inline bool isInVector(search_node node, std::vector<search_node> node_vector)
{
    for (const auto &item : node_vector)
    {
        if (isEqual(node.index, item.index))
            return true;
    }
    return false;
}
#endif