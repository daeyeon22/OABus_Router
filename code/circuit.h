#ifndef __CIRCUIT__
#define __CIRCUIT__ 0
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <map>
#include <cmath>
#include <climits>
#include <cfloat>
#include <algorithm>
#include <assert.h>
#include <cstdlib>
#include <cstring>

// BOOST Library
#include <boost/foreach.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/icl/interval_map.hpp>
#include <boost/icl/interval_set.hpp>
#include <boost/icl/interval_base_map.hpp>

#include "func.h"
//#include "flute.h"


// Dense hash map
#include <sparsehash/dense_hash_map>

// Pre-define
#ifndef PREDEF
#define PREDEF 
#define INIT_STR "INITSTR"
#define VERTICAL 111
#define HORIZONTAL 222
#endif

//#define GCELL_WIDTH 2000
//#define GCELL_HEIGHT 2000
#define ckt OABusRouter::Circuit::shared()
#define GCELL_WIDTH ckt->GCELL_WIDTH
#define GCELL_HEIGHT ckt->GCELL_HEIGHT


// Namespace
using namespace std;
using google::dense_hash_map;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
namespace bi = boost::icl;

// Boost Intervals
typedef set<int> IntSetT;
typedef bi::interval<int> IntervalT;
typedef bi::interval_set<int> IntervalSetT;
typedef bi::interval_map<int, IntSetT> IntervalMapT;
typedef bi::discrete_interval<int> DiscreteIntervalT;

// Boost Geometries

typedef pair<int,int> PairT;
typedef bg::model::point<float,2,bg::cs::cartesian> bgPointT;
typedef bg::model::box<bgPointT> bgBoxT;
typedef bg::model::segment<bgPointT> bgSegmentT;
typedef bg::model::polygon<bgPointT> bgPolygonT;
typedef bgi::rtree<pair<bgSegmentT, int>, bgi::rstar<16>> SegRtreeT;
typedef bgi::rtree<pair<bgBoxT, int>, bgi::rstar<16>> BoxRtreeT;
typedef bgi::rtree<pair<bgPointT, int>, bgi::rstar<16>> PointRtreeT;

// Obstacle-Aware On-Track Bus Router
namespace OABusRouter
{
    struct Point
    {
        int x;
        int y;

        // Constructor
        Point() : x(INT_MAX), y(INT_MAX) {}
        Point(int ix, int iy) : x(ix), y(iy) {}
        Point(const Point& pt) : x(pt.x), y(pt.y) {}

        void print();
    };
    
    struct Rect
    {
        Point ll;
        Point ur;

        // Constructor
        Rect()
        {
            ll = Point();
            ur = Point();
        }

        Rect(Point ill, Point iur) : 
            ll(ill), 
            ur(iur) {}

        Rect(const Rect& rt) : 
            ll(rt.ll), 
            ur(rt.ur) {}

        void print();
        Point center();
    };

    
    struct Layer
    {
        int id;
        int direction;
        int spacing;
        int llx, lly;
        int urx, ury;
        string name;
        //Rect boundary;
        //vector<int> tracks;
        vector<int> trackOffsets;

        //int lower_bound(int coord);
        //int upper_bound(int coord);
        
        bool is_vertical()
        { 
            return (this->direction == VERTICAL)? true : false; 
        }
        
        bool is_horizontal()
        { 
            return (this->direction == HORIZONTAL)? true : false; 
        }



        Layer() : 
            id(INT_MAX), 
            direction(INT_MAX), 
            spacing(INT_MAX),
            llx(INT_MAX),
            lly(INT_MAX),
            urx(INT_MIN),
            ury(INT_MIN),
            name(INIT_STR) 
        {

        }

        Layer(const Layer& lr) :
            id(lr.id),
            direction(lr.direction),
            spacing(lr.spacing),
            llx(lr.llx),
            lly(lr.lly),
            urx(lr.urx),
            ury(lr.ury),
            name(lr.name)
        {

        }

        void print(bool all);
    };


    struct Track
    {
        int id;
        int width;
        int offset;
        int llx, lly;
        int urx, ury;
        string layer;
        Point ll;
        Point ur;
        
        // Segment tree
        //IntervalMapT assignedIntervals;
        //IntervalSetT emptyIntervals;




        Track() : 
            id(INT_MAX), 
            width(INT_MAX), 
            llx(INT_MAX),
            lly(INT_MAX),
            urx(INT_MIN),
            ury(INT_MIN),
            layer(INIT_STR) {}

        Track(const Track& tr) :
            id(tr.id),
            width(tr.width),
            llx(tr.llx),
            lly(tr.lly),
            urx(tr.urx),
            ury(tr.ury),
            layer(tr.layer) {}

        void print();
    };



    struct Pin
    {
        int id;
        string bitName;
        string layer;
        Rect boundary;

        int leftBoundary(){ return this->boundary.ll.x; }
        int rightBoundary(){ return this->boundary.ur.x; }
        int topBoundary(){ return this->boundary.ur.y; }
        int bottomBoundary(){ return this->boundary.ll.x; }

        Pin() : 
            id(INT_MAX), 
            bitName(INIT_STR), 
            layer(INIT_STR) {}

        Pin(const Pin& p) :
            id(p.id),
            bitName(p.bitName),
            layer(p.layer)
        {
            boundary = p.boundary;
        }

        void print();
    };

    struct Bit
    {
        int id;
        string name;
        string busName;
        vector<int> pins;


        Rect boundary();

        Bit() : 
            id(INT_MAX), 
            name(INIT_STR), 
            busName(INIT_STR) {}

        Bit(const Bit& b) :
            id(b.id),
            name(b.name),
            busName(b.busName)
        {
            pins.insert(pins.end(), b.pins.begin(), b.pins.end());
        }

        void print();
    };

    struct Bus
    {
        int id;
        int numBits;
        int numPinShapes;
        int llx, lly;
        int urx, ury;
        string name;

        vector<int> bits;
        dense_hash_map<string,int> width;
        //HashMap;

        Bus() : 
            id(INT_MAX), 
            numBits(INT_MAX), 
            numPinShapes(INT_MAX), 
            llx(INT_MAX),
            lly(INT_MAX),
            urx(INT_MIN),
            ury(INT_MIN),
            name(INIT_STR)
        {
            width.set_empty_key(INIT_STR);
        }

        Bus(const Bus& b) :
            id(b.id),
            numBits(b.numBits),
            numPinShapes(b.numPinShapes),
            llx(b.llx),
            lly(b.lly),
            urx(b.urx),
            ury(b.ury),
            name(b.name)
        {
            bits = b.bits;
            width = b.width;
        }

        void print();
    };

    struct Obstacle
    {
        int id;
        int llx, lly;
        int urx, ury;
        string layer;
        //Rect boundary;

        Obstacle() : 
            id(INT_MAX), 
            llx(INT_MAX),
            lly(INT_MAX),
            urx(INT_MIN),
            ury(INT_MIN),
            layer(INIT_STR) {}

        Obstacle(const Obstacle& obs) :
            id(obs.id),
            llx(obs.llx),
            lly(obs.lly),
            urx(obs.urx),
            ury(obs.ury),
            layer(obs.layer) {}

        void print();

    };
    


    class Circuit
    {
      private:
        static Circuit* instance;

        
      public:
        static Circuit* shared();


        // Parameters
        int runtime;
        int alpha;
        int beta;
        int delta;
        int gamma;
        int epsilon;
        int llx, lly;
        int urx, ury;
        
        //Rect designBoundary;
        // 
        //int GCELL_WIDTH;
        //int GCELL_HEIGHT;

        // Objects
        vector<Layer> layers;
        vector<Track> tracks;
        vector<Bus> buses;
        vector<Obstacle> obstacles;
        vector<Bit> bits;
        vector<Pin> pins;
        
        
        // should remove //
        //vector<Gcell> gCells;
        //vector<StTree> stTrees;
        //vector<Segment> segs;
        //vector<int> gCellOffsetLX;
        //vector<int> gCellOffsetLY;
        //vector<int> gCellOffsetUX;
        //vector<int> gCellOffsetUY;
        ///////////////////





        // Hash Map
        dense_hash_map<string,int> bitHashMap;
        dense_hash_map<string,int> busHashMap;
        dense_hash_map<string,int> layerHashMap;
        dense_hash_map<size_t,int> trackHashMap;
        
        // should remove //
        //dense_hash_map<size_t,int> gCellHashMap; 
        //dense_hash_map<int,int> stTreeHashMap;
        ///////////////////



        //BoxRtreeT pinRtree;
        //SegRtreeT trackRtree;



        // Initializer
        Circuit() :
            runtime(INT_MAX), 
            alpha(INT_MAX),
            beta(INT_MAX),
            delta(INT_MAX),
            gamma(INT_MAX),
            epsilon(INT_MAX),
            llx(INT_MAX),
            lly(INT_MAX),
            urx(INT_MIN),
            ury(INT_MIN)
        {
            bitHashMap.set_empty_key(INIT_STR);
            busHashMap.set_empty_key(INIT_STR);
            layerHashMap.set_empty_key(INIT_STR);
            trackHashMap.set_empty_key(0);
            
            // should remove // 
            //stTreeHashMap.set_empty_key(0);
            //gCellHashMap.set_empty_key(0);
            ///////////////////
        }


        
        // Getter
        Bit* getBit(string& name);
        Bus* getBus(string& name);
        Layer* getLayer(string* name);

        // Parser
        bool read_iccad2018(char* fileName);
        bool getParam(char* fileName);
        bool getLayerInfo(char* fileName);
        bool getTrackInfo(char* fileName);
        bool getBusInfo(char* fileName);
        bool getObstacleInfo(char* fileName);


        // Route
        //void Init();
        //void GenBackbone();
        //void GenBackbone_v2();
        //void LayerAssignment();
        //void RoutingPoint();
        //void InitRoutingDirection();
        // Util
        
        //void GenPlot();
        //void GenTopology();
    };



};


#endif
