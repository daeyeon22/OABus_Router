#ifndef FUNC_H
#define FUNC_H 0
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <tuple>
#include <boost/functional/hash.hpp>

// Util functions



extern size_t GetHashKey(std::pair<std::string,int> &value);
extern size_t GetHashKey(std::tuple<std::string,int,int> &value);
extern size_t GetHashKey(std::pair<int,int> &value);
extern int GetLowerBound(std::vector<int>& target, int crd);
extern int GetUpperBound(std::vector<int>& target, int crd);
template <typename A, typename B>
extern size_t GetHashKey(std::pair<A,B> &value);

template <typename A>
extern int GetLowerBound(std::vector<A>& target, A crd);

template <typename A>
extern int GetUpperBound(std::vector<A>& target, A crd);
#endif
