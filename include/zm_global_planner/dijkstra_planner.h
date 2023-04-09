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
#include <zm_global_planner/all_type.h>
#include <algorithm>

namespace zm_global_planner
{
    class DijkstraPlanner
    {
        public:
            DijkstraPlanner(unsigned int width_, unsigned int height_, bool* obsMap_);
            ~DijkstraPlanner();
            std::vector<int> GlobalPlanner(int start, int goal);

        private:
            bool CheckInMap(int x, int y);
            void reset();
            int width, height, size;
            CELL_INFO *map; // map; 
    };
}

#endif