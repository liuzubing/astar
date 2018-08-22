#ifndef CONSTANTS
#define CONSTANTS
#include <iostream>
#include <vector>

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
    bool is_pass;
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