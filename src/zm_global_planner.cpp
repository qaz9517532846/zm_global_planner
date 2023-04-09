#include <zm_global_planner/zm_global_planner.h>
#include <pluginlib/class_list_macros.h>

#define OBSTACLE_COST_VALUE         (10)

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

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);

        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        width_ = costmap_->getSizeInCellsX();
        height_ = costmap_->getSizeInCellsY();
        resolution_ = costmap_->getResolution();
        mapSize_ = width_ * height_;

        obsMap_ = new bool [mapSize_];

        for (unsigned int iy = 0; iy < height_; iy++)
        {
            for (unsigned int ix = 0; ix < width_; ix++)
            {
                int cost = static_cast<int>(costmap_->getCost(ix, iy));
                if(cost > OBSTACLE_COST_VALUE)
                    obsMap_[iy * width_ + ix] = true;
                else
                    obsMap_[iy * width_ + ix] = false;
            }
        }

        ap_ = boost::shared_ptr<AstarPlanner>(new AstarPlanner(width_, height_, obsMap_));
        dp_ = boost::shared_ptr<DijkstraPlanner>(new DijkstraPlanner(width_, height_, obsMap_));

        dsrv_ = new dynamic_reconfigure::Server<ZMGlobalPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<ZMGlobalPlannerConfig>::CallbackType cb = boost::bind(&ZMGlobalPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        ROS_INFO("zm global planner initialized successfully");
        initial_ = true;
    }

    void ZMGlobalPlanner::reconfigureCB(ZMGlobalPlannerConfig &config, uint32_t level)
    {
        astar_ = config.use_astart;
        mapFrame_ = config.map_frame;
    }
    
    bool ZMGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan)
    {
        std::vector<int> pathIdx;
        int startIdx, goalIdx;
        geometry_msgs::PoseStamped findPose;
        nav_msgs::Path path;
        bool result = false;
        plan.clear();
        if(initial_ && CheckInMap(start) && CheckInMap(goal)) 
        {
            startIdx = MapPosToCostMapIdx(start);
            goalIdx = MapPosToCostMapIdx(goal);
            if(astar_) pathIdx = ap_->GlobalPlanner(startIdx, goalIdx);
            else       pathIdx = dp_->GlobalPlanner(startIdx, goalIdx);
            for(int i = 0; i < pathIdx.size(); i++)
            {
                static geometry_msgs::PoseStamped lastPose;
                double angle = 0;
                if(i == 0) findPose = start;
                else if(i == pathIdx.size() - 1) findPose = goal;
                else
                {
                    findPose = CostMapIdxToMapPos(pathIdx[i]);
                    angle = atan2((findPose.pose.position.y - lastPose.pose.position.y),
                                  (findPose.pose.position.x - lastPose.pose.position.x));
                    findPose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
                }

                plan.push_back(findPose);
                lastPose = findPose;
            }

            path = path_publisher(plan);
            plan_pub_.publish(path);
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

    int ZMGlobalPlanner::MapPosToCostMapIdx(geometry_msgs::PoseStamped pose)
    {
        int posIdx_x, posIdx_y;
        costmap_->worldToMapEnforceBounds(pose.pose.position.x, pose.pose.position.y, posIdx_x, posIdx_y);
        return posIdx_y * width_ + posIdx_x;
    }

    geometry_msgs::PoseStamped ZMGlobalPlanner::CostMapIdxToMapPos(int posIdx)
    {
        geometry_msgs::PoseStamped pos;
        pos.header.stamp = ros::Time::now();
        pos.header.frame_id = mapFrame_;
        int posIdx_x = posIdx % width_;
        int posIdx_y = posIdx / width_;
        costmap_->mapToWorld(posIdx_x, posIdx_y, pos.pose.position.x, pos.pose.position.y);
        return pos;
    }

    nav_msgs::Path ZMGlobalPlanner::path_publisher(std::vector<geometry_msgs::PoseStamped> plan)
    {
        nav_msgs::Path pub_path_;
		pub_path_.header.stamp = ros::Time::now();
     	pub_path_.header.frame_id = mapFrame_;
     	pub_path_.poses = plan;
		return pub_path_;
    }
};