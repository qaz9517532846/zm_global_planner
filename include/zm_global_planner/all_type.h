#ifndef ALL_TYPE_CPP
#define ALL_TYPE_CPP

typedef struct{
    int parent;
    int64_t cost = std::numeric_limits<int64_t>::max();
    bool visit;
    bool obs;
} CELL_INFO;

struct CELL_POS
{
    int x;
    int y;
};

using Score = std::pair<int, CELL_POS>;

struct CompareScore
{
    bool operator() (const Score& a, const Score& b)
    {
        return a.first > b.first;
    }
};

#endif