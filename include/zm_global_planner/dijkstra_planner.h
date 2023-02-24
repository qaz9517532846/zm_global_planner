#ifndef DIJKSTRA_CPP
#define DIJKSTRA_CPP

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf/transform_broadcaster.h>
#include <algorithm>

typedef struct{
    uint32_t mapParentIdx;
    bool visit;
    uint8_t cost;
} cellInfo;

namespace zm_global_planner
{
    class DijkstraPlanner
    {
        public:
            DijkstraPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            ~DijkstraPlanner();
            std::vector<int> GlobalPlanner(int start, int goal);

        private:
            unsigned int width_;
            unsigned int height_;
            unsigned int mapSize_;
    };
}

#endif