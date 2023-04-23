#include <zm_global_planner/rrtstar_planner.h>

namespace zm_global_planner
{
    RRTstarPlanner::RRTstarPlanner(unsigned int width_, unsigned int height_, bool* obsMap_, 
                           double minRand, double maxRand, 
                           double expandDist, double goalSampleRate,
                           unsigned int maxIter)
    {
        size = width_ * height_;
        width = width_;
        height = height_;
        map = new CELL_INFO [size];
        for(int i = 0; i < size; i++)
        {
            map[i].obs = *(obsMap_ + i);
            if(map[i].obs) obsList.push_back(i);
        }
    }

    RRTstarPlanner::~RRTstarPlanner()
    {

    }

    std::vector<int> RRTstarPlanner::GlobalPlanner(int start, int goal, bool animation)
    {
        std::vector<int> path;
        std::vector<CELL_POS> nodeList;

        CELL_POS startPos;
        CELL_POS goalPose;
        startPos.x = start % width;
        startPos.y = start / width;
        goalPose.x = goal % width;
        goalPose.y = goal / width;
        reset();
        srand(time(0));

        nodeList.push_back(startPos);
        for(int i=0; i < iter; i++)
        {
            /// Random Sampling
            CELL_POS rnd = getRandomPoint(goal);

            /// Find nearest node
            auto nind = getNearestListIndex(nodeList, rnd);

            auto new_node = steer(nodeList, rnd, nind);

            if(CollisionCheck(new_node, obsList))
            {
                auto nearinds = FindNearNodes(new_node, nodeList);
                /*chooseParent(nearinds, new_node);
                node_list_.push_back(new_node);
                rewire(new_node, nearinds); // FIXME: for what?*/
            }

            /*if (animation) {
                drawGraph(rnd);
            }*/
        }
        return path;
    }

    CELL_POS RRTstarPlanner::getRandomPoint(int goal)
    {
        CELL_POS rnd;
        double sample_rate = fmod(double(rand()),(100 - 0 + 1)) + 0;
        if(sample_rate > sampleRate)
        {
            rnd.x = fmod(double(rand()),(maxRand - minRand + 1)) + minRand;
            rnd.y = fmod(double(rand()),(maxRand - minRand + 1)) + minRand;
        }
        else // goal point sampling
        {
            rnd.x = goal % width;
            rnd.y = goal / width;
        }
        return rnd;
    }

    int RRTstarPlanner::getNearestListIndex(const std::vector<CELL_POS>& list, const CELL_POS& rnd)
    {
        double minind = std::numeric_limits<double>::infinity();
        int nind = -1;
        for (int i=0; i< list.size(); i++)
        {
            double dx = list[i].x - rnd.x;
            double dy = list[i].y - rnd.y;
            double d = sqrt(pow(dx, 2) + pow(dy, 2));
            if(d < minind)
            {
                minind = d;
                nind = i;
            }
        }

        return nind;
    }

    CELL_POS RRTstarPlanner::steer(std::vector<CELL_POS> list, const CELL_POS& rnd, int nind)
    {
        // expand tree
        int newNodeIdx = 0;
        CELL_POS nearestNode = list[nind];
        double theta = atan2(rnd.y - nearestNode.y, rnd.x - nearestNode.x);

        CELL_POS newNode = nearestNode;
        newNode.x += (int)(expandDist * cos(theta));
        newNode.y += (int)(expandDist * sin(theta));
        newNodeIdx = newNode.y * width + newNode.x;
        map[newNodeIdx].cost += expandDist;
        map[newNodeIdx].parent = nind;

        return newNode;
    }

    bool RRTstarPlanner::CollisionCheck(CELL_POS node, std::vector<int> list)
    {
        bool result;
        for(int i = 0; i < list.size(); i++)
        {
            double dx = list[i] % width - node.x;
            double dy = list[i] / width - node.y;
            double dist = sqrt(pow(dx, 2) + pow(dy, 2));
            if(dist <= 1) result = false;
        }

        return result;
    }

    std::vector<int> RRTstarPlanner::FindNearNodes(const CELL_POS& newNode, std::vector<CELL_POS> nodeList)
    {
        std::vector<int> nearinds;
        int nodeNum = nodeList.size();
        double r = 50.0 * sqrt((log(nodeNum) / nodeNum));
        for (int i = 0; i< nodeList.size(); i++)
        {
            double dlist = pow(nodeList[i].x - newNode.x, 2) + pow(nodeList[i].y - newNode.y, 2);
            if (dlist <= pow(r, 2)) nearinds.push_back(i);
        }
        return nearinds;
    }

    void RRTstarPlanner::ChooseParent(const std::vector<int>& nearinds, std::vector<CELL_POS> nodeList, CELL_POS& newNode)
    {
        if(nearinds.empty()) return;

        std::vector<double> dlist;
        for(int i = 0; i < nearinds.size(); i++)
        {
            double dx = newNode.x - nodeList[nearinds[i]].x;
            double dy = newNode.y - nodeList[nearinds[i]].y;
            double d = sqrt(pow(dx, 2) + pow(dy, 2));
            double theta = atan2(dy, dx);
        }
    }

    bool RRTstarPlanner::CheckInMap(int x, int y)
    {
        bool result = true;
        if(x > width || x < 0 || y > height || y < 0) result = false;
        return result;
    }

    void RRTstarPlanner::reset()
    {
        for(int i = 0; i < size; i++)
        {
            map[i].visit = false;
            map[i].parent = -1;
            map[i].cost = std::numeric_limits<int64_t>::max();
        }
    }
};