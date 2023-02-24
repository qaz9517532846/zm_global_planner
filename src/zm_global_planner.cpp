#include <zm_global_planner/zm_global_planner.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(zm_global_planner::ZMGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

 //Default Constructor
namespace zm_global_planner
{
    ZMGlobalPlanner::ZMGlobalPlanner()
    {

    }
    
    ZMGlobalPlanner::~ZMGlobalPlanner()
    {
        delete dsrv_;
    }

    ZMGlobalPlanner::ZMGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        initialize(name, costmap_ros);
    }
    
    void ZMGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ros::NodeHandle private_nh("~/" + name);

        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        width_ = costmap_->getSizeInCellsX();
        height_ = costmap_->getSizeInCellsY();
        resolution_ = costmap_->getResolution();
        mapSize_ = width_ * height_;
        ROS_INFO("width = %d, height = %d, mapsize = %d x %d, resolution = %f", width_, height_, width_, height_, resolution_);

        obsMap_ = new bool [mapSize_];

        for (unsigned int iy = 0; iy < height_; iy++)
        {
            for (unsigned int ix = 0; ix < width_; ix++)
            {
                unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
                if(cost < obsCost_)
                    obsMap_[iy * width_ + ix] = true;
                else
                    obsMap_[iy * width_ + ix] = false;
            }
        }

        dp_ = boost::shared_ptr<DijkstraPlanner>(new DijkstraPlanner(name, costmap_ros));

        dsrv_ = new dynamic_reconfigure::Server<ZMGlobalPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<ZMGlobalPlannerConfig>::CallbackType cb = boost::bind(&ZMGlobalPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        ROS_INFO("zm global planner initialized successfully");
        initial_ = true;
    }

    void ZMGlobalPlanner::reconfigureCB(ZMGlobalPlannerConfig &config, uint32_t level)
    {
        astar_ = config.use_astart;
        obsCost_ = config.obs_cost;
    }
    
    bool ZMGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan)
    {
        bool result = false;
        if(initial_ && CheckInMap(start) && CheckInMap(goal)) 
        {
            result = true;
        }
        else ROS_INFO("zm global planner plan failed.");

        return result;
    }

    bool ZMGlobalPlanner::CheckInMap(geometry_msgs::PoseStamped pos)
    {
        bool result = true;
        int costX, costY;
        costmap_->worldToMapEnforceBounds(pos.pose.position.x, pos.pose.position.y, costX, costY);
        if(costX > width_ || costX < 0 || costY > height_ || costY < 0) return false;
        return !obsMap_[costY * width_ + costX] ;
    }
};