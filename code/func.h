#ifndef FUNC_H
#define FUNC_H 0
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <tuple>
#include <boost/functional/hash.hpp>
#include <boost/geometry.hpp>
// Util functions
#define HORIZONTAL 222
#define VERTICAL 111 
#define PINTYPE -1212
#define OBSTACLE -1232
#define WIRETYPE -3319
#define NOT_ASSIGN 3391



namespace bg = boost::geometry;
typedef bg::model::point<float,2,bg::cs::cartesian> pt;
typedef bg::model::segment<pt> seg;
typedef bg::model::box<pt> box;

extern int manhatan_distance(int x1, int y1, int x2, int y2);
extern void lpt(seg s, int &x, int &y);
extern void upt(seg s, int &x, int &y);
extern bool intersects(seg s1, seg s2);
extern bool intersection(seg s1, seg s2, int &x, int &y);

//extern void into_array(int x1, int x2, int y1, int y2, int x[], int y[]);
//extern void into_array(int v1, int v2, int v[]);
//extern void expand_width(int x[], int y[], int width, int vertical);
//extern void pin_area(int x[], int y[], int align, int width, box& box);
extern void design_ruled_area(int x[], int y[], int width, int spacing, bool vertical);
extern double runtime(double start);
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
