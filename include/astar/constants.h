#ifndef CONSTANTS
#define CONSTANTS
#include <iostream>

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
};

inline static bool isEqual(const pose_index &a, const pose_index &b)
{   
    return (a.x == b.x && a.y == b.y) ? true : false;
}
inline static bool isEqual(const search_node &a, const search_node &b)
{    
    if (a.index.x != b.index.x || a.index.y != b.index.y)
        return false;
    if (a.g != b.g)
        return false;
    if (a.h != b.h)
        return false;
    if (a.g_h != b.g_h)
        return false;
    return true;
}
#endif