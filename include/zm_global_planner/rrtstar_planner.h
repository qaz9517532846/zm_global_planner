#ifndef RRT_STAR_CPP
#define RRT_STAR_CPP

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
    class RRTstarPlanner
    {
        public:
            RRTstarPlanner(unsigned int width_, unsigned int height_, bool* obsMap_, 
                           double minRand, double maxRand, 
                           double expandDist, double goalSampleRate,
                           unsigned int maxIter);
            ~RRTstarPlanner();
            std::vector<int> GlobalPlanner(int start, int goal, bool animation);

        private:
            CELL_POS getRandomPoint(int goal);
            CELL_POS steer(std::vector<CELL_POS> list, const CELL_POS& rnd, int nind);
            int getNearestListIndex(const std::vector<CELL_POS>& list, const CELL_POS& rnd);
            bool CollisionCheck(CELL_POS node, std::vector<int> list);
            std::vector<int> FindNearNodes(const CELL_POS& newNode, std::vector<CELL_POS> nodeList);
            void ChooseParent(const std::vector<int>& nearinds, std::vector<CELL_POS> nodeList, CELL_POS& newNode);

            bool CheckInMap(int x, int y);
            void reset();
            int width, height, size;
            CELL_INFO *map; // map; 
            std::vector<int> obsList;

            double minRand, maxRand;
            double expandDist;
            double sampleRate;
            unsigned int iter;
            
    };
}

#endif