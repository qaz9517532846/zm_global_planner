#include <zm_global_planner/dijkstra_planner.h>

namespace zm_global_planner
{
    DijkstraPlanner::DijkstraPlanner(unsigned int width_, unsigned int height_, bool* obsMap_)
    {
        size = width_ * height_;
        width = width_;
        height = height_;
        map = new CELL_INFO [size];
        memcpy(&map[0].obs, obsMap_, sizeof(bool) * size);
    }

    DijkstraPlanner::~DijkstraPlanner()
    {

    }

    std::vector<int> DijkstraPlanner::GlobalPlanner(int start, int goal)
    {
        int posX, posY;
        int currentPos = start;
        int nextPos, nextCost;
        std::vector<int> path;

        memset(&map[0].visit, 0, sizeof(bool) * size);
        memset(&map[0].parent, -1, sizeof(int) * size);

        while(!map[goal].visit)
        {
            map[currentPos].visit = true;
            for(int ix = -1; ix <= 1; ix++)
            {
                for(int iy = -1; iy <= 1; iy++)
                {
                    posX = currentPos % width + ix;
                    posY = currentPos / width + iy;
                    nextPos = posY * width + posY;
                    if(!CheckInMap(posX, posY)) continue;
                    if(map[nextPos].visit || map[nextPos].obs) continue;
                    nextCost = map[currentPos].cost + sqrt(ix * ix + iy * iy) * 10;
                    if(nextCost < map[nextPos].cost)
                    {
                        printf("Update\n");
                        map[nextPos].parent = currentPos;
                        map[nextPos].cost = nextCost;
                    }

                    currentPos = nextPos;
                }
            }
        }

        printf("COMPLETE\n");

        currentPos = goal;
        path.push_back(currentPos);
        while(map[currentPos].parent != start)
        {
            printf("idx = %d\n", currentPos);
            path.push_back(map[currentPos].parent);
            currentPos = map[currentPos].parent;
        }

        printf("COMPLETE 1\n");

        std::reverse(path.begin(), path.end());
        return path;
    }

    bool DijkstraPlanner::CheckInMap(int x, int y)
    {
        bool result = true;
        if(x > width || x < 0 || y > height || y < 0) result = false;
        return result;
    }
};