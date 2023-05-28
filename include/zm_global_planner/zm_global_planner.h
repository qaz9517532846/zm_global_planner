#ifndef ZM_GLOBAL_PLANNER_CPP
#define ZM_GLOBAL_PLANNER_CPP

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf/transform_broadcaster.h>

#include <zm_global_planner/dijkstra_planner.h>
#include <zm_global_planner/astar_planner.h>

#include <dynamic_reconfigure/server.h>
#include <zm_global_planner/ZMGlobalPlannerConfig.h>

using namespace std;
using std::string;

namespace zm_global_planner
{
    class ZMGlobalPlanner : public nav_core::BaseGlobalPlanner
    {
        public:
            ZMGlobalPlanner();
            ~ZMGlobalPlanner(); 
            ZMGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

        private:
            void reconfigureCB(ZMGlobalPlannerConfig &config, uint32_t level);
            bool CheckInMap(geometry_msgs::PoseStamped pos);
            int MapPosToCostMapIdx(geometry_msgs::PoseStamped pose);
            geometry_msgs::PoseStamped CostMapIdxToMapPos(int posIdx);
            nav_msgs::Path path_publisher(std::vector<geometry_msgs::PoseStamped> plan);

            bool astar_, initial_;
            dynamic_reconfigure::Server<ZMGlobalPlannerConfig> *dsrv_;
            ros::Publisher plan_pub_;

            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D* costmap_;
            unsigned int width_;
            unsigned int height_;
            unsigned int mapSize_;

            float resolution_;

            bool* obsMap_; // cost map; 
            std::string mapFrame_;

            boost::shared_ptr<DijkstraPlanner> dp_;
            boost::shared_ptr<AstarPlanner> ap_;
    };
};

#endif 