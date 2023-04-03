#include <zm_global_planner/astar_planner.h>

namespace zm_global_planner
{
    AstarPlanner::AstarPlanner(unsigned int width_, unsigned int height_, bool* obsMap_)
    {
        size = width_ * height_;
        width = width_;
        height = height_;
        map = new CELL_INFO [size];
        memcpy(&map[0].obs, obsMap_, sizeof(bool) * size);
    }

    AstarPlanner::~AstarPlanner()
    {

    }

    std::vector<int> AstarPlanner::GlobalPlanner(int start, int goal)
    {
        std::vector<int> path;
        std::priority_queue<Score, std::vector<Score>, CompareScore> open_list;
        int current;
        CELL_POS startPos;
        startPos.x = start % width;
        startPos.y = start / width;

        memset(&map[0].visit, 0, sizeof(bool) * size);
        memset(&map[0].parent, -1, sizeof(int) * size);

        map[start].cost = 0;
        open_list.push({map[start].cost, startPos});

        while(!open_list.empty())
        {
            CELL_POS currentPos = open_list.top().second;
            open_list.pop();

            current = currentPos.y * width + currentPos.x;
            if(current == goal) break;

            map[current].visit = true;

            for(int ix = -1; ix <= 1; ix++)
            {
                for(int iy = -1; iy <= 1; iy++)
                {
                    CELL_POS calPos;
                    calPos.x = currentPos.x + ix;
                    calPos.y = currentPos.y + iy;
                    int calIdx = calPos.y * width + calPos.x;
                    if(!CheckInMap(calPos.x , calPos.y)) continue;
                    if(map[calIdx].visit || map[calIdx].obs) continue;

                    int calCost = map[current].cost + sqrt(ix * ix + iy * iy) * 10;
                    if(calCost < map[calIdx].cost)
                    {
                        map[calIdx].cost = calCost;
                        open_list.push({map[calIdx].cost, calPos});
                        map[calIdx].parent = current;
                    }
                }
            }
        }

        current = goal;
        while(current != start)
        {
            path.emplace_back(current);
            current = map[current].parent;
        }

        path.emplace_back(start);
        std::reverse(path.begin(),path.end());

        ROS_INFO("Finished Path");
        return path;
    }

    bool AstarPlanner::CheckInMap(int x, int y)
    {
        bool result = true;
        if(x > width || x < 0 || y > height || y < 0) result = false;
        return result;
    }
};