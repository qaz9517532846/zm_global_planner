#include <zm_global_planner/dijkstra_planner.h>

namespace zm_global_planner
{
    DijkstraPlanner::DijkstraPlanner(unsigned int width_, unsigned int height_, bool* obsMap_)
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

    DijkstraPlanner::~DijkstraPlanner()
    {

    }

    std::vector<int> DijkstraPlanner::GlobalPlanner(int start, int goal)
    {
        std::vector<int> path;
        std::priority_queue<Score, std::vector<Score>, CompareScore> open_list;
        open_list = std::priority_queue<Score, std::vector<Score>, CompareScore>();
        int current;
        CELL_POS startPos;
        startPos.x = start % width;
        startPos.y = start / width;

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
                        map[calIdx].cost = calCost;
                        open_list.push({map[calIdx].cost, calPos});
                        map[calIdx].parent = current;
                    }
                }
            }
        }

        current = goal;
        while(map[current].parent != start)
        {
            path.emplace_back(current);
            current = map[current].parent;
        }

        path.emplace_back(start);
        std::reverse(path.begin(),path.end());

        return path;
    }

    bool DijkstraPlanner::CheckInMap(int x, int y)
    {
        bool result = true;
        if(x > width || x < 0 || y > height || y < 0) result = false;
        return result;
    }

    void DijkstraPlanner::reset()
    {
        for(int i = 0; i < size; i++)
        {
            map[i].visit = false;
            map[i].parent = -1;
            map[i].cost = std::numeric_limits<int64_t>::max();
        }
    }
};