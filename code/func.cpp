#include "func.h"
#include <algorithm>
#include <vector>



int manhatan_distance(int x1, int y1, int x2, int y2)
{
    return abs(x1 - x2) + abs(y1 - y2);
}

// lower point
void lpt(seg s, int &x, int &y)
{
    x = (int)(bg::get<0,0>(s) + 0.5);
    y = (int)(bg::get<0,1>(s) + 0.5);
}

// upper point
void upt(seg s, int &x, int &y)
{
    x = (int)(bg::get<1,0>(s) + 0.5);
    y = (int)(bg::get<1,1>(s) + 0.5);
}


bool intersects(seg s1, seg s2)
{
    return bg::intersects(s1, s2);
}

bool intersection(seg s1, seg s2, int &x, int &y)
{
    std::vector<pt> _intersection;
    bg::intersection(s1, s2, _intersection);
    if(_intersection.size() == 0)
        return false;
    else
    {
        x = (int)(bg::get<0>(_intersection[0]) + 0.5);
        y = (int)(bg::get<1>(_intersection[0]) + 0.5);
        return true;
    }
}

size_t GetHashKey(std::pair<std::string,int> &value)
{
    using boost::hash_value;
    using boost::hash_combine;
    size_t seed = 0;
    hash_combine(seed, hash_value(value.first));
    hash_combine(seed, hash_value(value.second));
    return seed;
}

size_t GetHashKey(std::pair<int,int> &value)
{
    using boost::hash_value;
    using boost::hash_combine;
    size_t seed = 0;
    hash_combine(seed, hash_value(value.first));
    hash_combine(seed, hash_value(value.second));
    return seed;
}



size_t GetHashKey(std::tuple<std::string,int,int> &value)
{
    using boost::hash_value;
    using boost::hash_combine;
    size_t seed = 0;
    hash_combine(seed, hash_value(std::get<0>(value)));
    hash_combine(seed, hash_value(std::get<1>(value)));
    hash_combine(seed, hash_value(std::get<2>(value)));
    return seed;
}

int GetLowerBound(std::vector<int>& target, int crd)
{
    std::vector<int>::iterator it;
    it = std::lower_bound(target.begin(), target.end(), crd);
    return (it - target.begin());
}

int GetUpperBound(std::vector<int>& target, int crd)
{
    std::vector<int>::iterator it;
    it = std::upper_bound(target.begin(), target.end(), crd);
    return (it - target.begin());
}





template <typename A, typename B>
size_t GetHashKey(std::pair<A,B> &value)
{   
    using boost::hash_value;
    using boost::hash_combine;
    size_t seed = 0;
    hash_combine(seed, hash_value(value.first));
    hash_combine(seed, hash_value(value.second));
    return seed;
}

template <typename A>
int GetLowerBound(std::vector<A>& target, A crd)
{
    typename std::vector<A>::iterator it;
    it = std::lower_bound(target.begin(), target.end(), crd);
    return (it - target.begin());
}

template <typename A>
int GetUpperBound(std::vector<A>& target, A crd)
{
    typename std::vector<A>::iterator it;
    it = std::upper_bound(target.begin(), target.end(), crd);
    return (it - target.begin());
}

