#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__

#include <boost/geometry.hpp>
#include <boost/icl/interval_map.hpp>
#include <boost/icl/interval_set.hpp>
#include <boost/icl/interval_base_map.hpp>
#include <sparsehash/dense_hash_map>

#define HORIZONTAL 222
#define VERTICAL 111 
#define PINTYPE -1212
#define OBSTACLE -1232
#define WIRETYPE -3319
#define NUM_THREADS 16 




using namespace std;
using google::dense_hash_map;
namespace bi = boost::icl;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// For Interval tree library

// Boost geometries
typedef bg::model::point<float,2, bg::cs::cartesian> pt;
typedef bg::model::segment<pt> seg;
typedef bg::model::box<pt> box;
typedef bg::model::polygon<pt> polygon;

typedef bgi::rtree<pair<pt, int>, bgi::rstar<16>> PointRtree;
typedef bgi::rtree<pair<seg, int>, bgi::rstar<16>> SegRtree;
typedef bgi::rtree<pair<box, int>, bgi::rstar<16>> BoxRtree;
typedef pair<int,int> edge;


enum Direction
{
    Left,
    Right,
    Down,
    Up,
    Stack
};

#endif
