#include "func.h"
#include <algorithm>



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

