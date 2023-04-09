#include <zm_global_planner/astar_planner.h>

namespace zm_global_planner
{
    AstarPlanner::AstarPlanner(unsigned int width_, unsigned int height_, bool* obsMap_)
    {
        size = width_ * height_;
        width = width_;
        height = height_;
        map = new CELL_INFO [size];
        for(int i = 0; i < size; i++)
        {
            map[i].obs = *(obsMap_ + i);
        }
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
        CELL_POS goalPose;
        startPos.x = start % width;
        startPos.y = start / width;
        goalPose.x = goal % width;
        goalPose.y = goal / width;

        reset();

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
                        int64_t cost_h = Heuristic_function(calPos, goalPose);
                        map[calIdx].cost = calCost;
                        open_list.push({map[calIdx].cost + cost_h, calPos});
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

        return path;
    }

    int64_t AstarPlanner::Heuristic_function(CELL_POS src, CELL_POS target)
    {
        int64_t score;
        score = 10 * sqrt(pow(target.x - src.x, 2) + pow(target.y - src.y, 2));
        return static_cast<int64_t>(score);
    }

    bool AstarPlanner::CheckInMap(int x, int y)
    {
        bool result = true;
        if(x > width || x < 0 || y > height || y < 0) result = false;
        return result;
    }

    void AstarPlanner::reset()
    {
        for(int i = 0; i < size; i++)
        {
            map[i].visit = false;
            map[i].parent = -1;
            map[i].cost = std::numeric_limits<int64_t>::max();
        }
    }
};