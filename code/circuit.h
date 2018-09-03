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
#include <cassert>

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

#define INIT_STR "INITSTR"
// Pre-define
#ifndef PREDEF
#define PREDEF 
#define VERTICAL 111
#define HORIZONTAL 222
#endif

//#define GCELL_WIDTH 2000
//#define GCELL_HEIGHT 2000
#define ckt OABusRouter::Circuit::shared()
//#define GCELL_WIDTH ckt->GCELL_WIDTH
//#define GCELL_HEIGHT ckt->GCELL_HEIGHT


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

typedef bg::model::linestring<bgPointT> linestring;
typedef bg::model::multi_linestring<linestring> mlinestring;
typedef bg::model::polygon<bgPointT> poly;
typedef bg::model::multi_polygon<poly> mpoly;

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
        bool operator==(Point &b) {
            return x == b.x && y == b.y;
        }
        bool operator!=(Point &b) {
            return x != b.x || y != b.x;
        }
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
        int min_width;
        int minpitch;
        int maxpitch;
        
        string name;
        //Rect boundary;
        vector<int> trackOffsets;
        set<int> offsets;

        //int lower_bound(int coord);
        //int upper_bound(int coord);
       
       
        // add by SGD 
        vector<int> tracks;

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
            min_width(INT_MAX),
            name(INIT_STR) 
        {
            tracks.reserve(1028);
        }

        Layer(const Layer& lr) :
            id(lr.id),
            direction(lr.direction),
            spacing(lr.spacing),
            llx(lr.llx),
            lly(lr.lly),
            urx(lr.urx),
            ury(lr.ury),
            min_width(lr.min_width),
            name(lr.name)
        {
            tracks.insert(tracks.end(),lr.tracks.begin(),lr.tracks.end());
        }

        void print(bool all);
    };

    struct Contact
    {
        Point p;
        int id;
        int trackid;
        int l;
        int connected_con; // connected contact on other layer;
        bool wire_use;
        vector<int> tracks;
        Contact() :
            id(INT_MAX),
            trackid(INT_MAX),
            l(INT_MAX),
            connected_con(INT_MAX),
            wire_use(false)
            {
                tracks.reserve(2);
            }

        Contact(const Contact &con) :
            id(con.id),
            trackid(con.trackid),
            l(con.l),
            p(con.p),
            connected_con(con.connected_con),
            wire_use(con.wire_use)
            {
                tracks.insert(tracks.end(),con.tracks.begin(),con.tracks.end());
            }
        void print();
    };

    struct Track
    {
        int id;
        int width;
        int offset;
        int llx, lly;
        int urx, ury;
        int l; // layer id

        Point start;
        Point end;

        vector<int> wires;  // rou->wires index 
        vector<int> contacts; // ckt->contacts index
        
        // Segment tree
        //IntervalMapT assignedIntervals;
        //IntervalSetT emptyIntervals;


        Track() : 
            id(INT_MAX), 
            width(INT_MAX), 
            offset(INT_MAX),
            llx(INT_MAX),
            lly(INT_MAX),
            urx(INT_MIN),
            ury(INT_MIN),
            l(INT_MAX) {}

        Track(const Track& tr) :
            id(tr.id),
            width(tr.width),
            offset(tr.offset),
            llx(tr.llx),
            lly(tr.lly),
            urx(tr.urx),
            ury(tr.ury),
            l(tr.l),
            start(tr.start),
            end(tr.end) {}

        void print();
    };



    struct Pin
    {
        int id;
        int llx, lly;
        int urx, ury;
        int l; // layer id
        int trackid;

        string bitName;
        //string layer;
        //Rect boundary;
        string direction;

        Pin() : 
            id(INT_MAX), 
            llx(INT_MAX),
            lly(INT_MAX),
            urx(INT_MIN),
            ury(INT_MIN),
            l(INT_MAX),
            trackid(INT_MAX),
            bitName(INIT_STR),
            direction("OUTPUT") {}

        Pin(const Pin& p) :
            id(p.id),
            llx(p.llx),
            lly(p.lly),
            urx(p.urx),
            ury(p.ury),
            l(p.l),
            trackid(p.trackid),
            bitName(p.bitName),
            direction(p.direction) {}

        void print();
    };

    struct MultiPin
    {
        int id;
        int busid;
        int l;
        int llx, lly;
        int urx, ury;
        int align;
        bool needVia;
        vector<int> pins;
        MultiPin() :
            id(INT_MAX),
            busid(INT_MAX),
            l(INT_MAX),
            llx(INT_MAX),
            lly(INT_MAX),
            urx(INT_MIN),
            ury(INT_MIN),
            align(INT_MAX),
            needVia(false) {}

        MultiPin(const MultiPin& mp) :
            id(mp.id),
            busid(mp.busid),
            l(mp.l),
            llx(mp.llx),
            lly(mp.lly),
            urx(mp.urx),
            ury(mp.ury),
            align(mp.align),
            needVia(mp.needVia),
            pins(mp.pins)
        {}
            
        void print();
        bool vertical_arrange();
    };

    struct Path
    {
        int x[2];
        int y[2];
        int l;
        bool via;
        Path(){}
        Path(const Path& p)
        {
            x[0] = p.x[0];
            x[1] = p.x[1];
            y[0] = p.y[0];
            y[1] = p.y[1];
            l = p.l;
            via = p.via;
        }

    };


    struct Bit
    {
        int id;
        string name;
        string busName;
        vector<int> pins;
        vector<int> wires;
        vector<int> vias;
        vector<Path> paths;
        bool assign;
        //Rect boundary();

        Bit() : 
            id(INT_MAX), 
            name(INIT_STR), 
            busName(INIT_STR),
            assign(false)
        {}

        Bit(const Bit& b) :
            id(b.id),
            name(b.name),
            busName(b.busName),
            assign(b.assign)
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
        vector<int> multipins;
        dense_hash_map<int,int> width;

        bool assign;
        //HashMap;

        Bus() : 
            id(INT_MAX), 
            numBits(INT_MAX), 
            numPinShapes(INT_MAX), 
            llx(INT_MAX),
            lly(INT_MAX),
            urx(INT_MIN),
            ury(INT_MIN),
            name(INIT_STR),
            assign(false)
        {
            width.set_empty_key(INT_MAX); // INIT_STR);
        }

        Bus(const Bus& b) :
            id(b.id),
            numBits(b.numBits),
            numPinShapes(b.numPinShapes),
            llx(b.llx),
            lly(b.lly),
            urx(b.urx),
            ury(b.ury),
            name(b.name),
            bits(b.bits),
            multipins(b.multipins),
            width(b.width),
            assign(b.assign)
        {} 

        void print();
    };

    struct Obstacle
    {
        int id;
        int llx, lly;
        int urx, ury;
        int l; // layer id

        //string layer;
        //Rect boundary;

        Obstacle() : 
            id(INT_MAX), 
            llx(INT_MAX),
            lly(INT_MAX),
            urx(INT_MIN),
            ury(INT_MIN),
            l(INT_MAX) {} 

        Obstacle(const Obstacle& obs) :
            id(obs.id),
            llx(obs.llx),
            lly(obs.lly),
            urx(obs.urx),
            ury(obs.ury),
            l(obs.l) {}

        void print();

    };
   
    struct Contact
    {
        int id;
        Contact() :
        id(INT_MAX) {}

        Contact(const Contact &con) :
            id(con.id) {}
        void print();
    };

    class Circuit
    {
      private:
        static Circuit* instance;

        
      public:
        static Circuit* shared();

        string design;

        // Parameters
        int runtime;
        int alpha;
        int beta;
        int delta;
        int gamma;
        int epsilon;
        int originX;
        int originY;
        int width;
        int height;
       
        // Objects
        vector<Layer> layers;
        vector<Track> tracks;
        vector<Bus> buses;
        vector<Obstacle> obstacles;
        vector<Bit> bits;
        vector<Pin> pins;
        vector<MultiPin> multipins;
        vector<Contact> contacts;
        
        // Hash Map
        dense_hash_map<string,int> bitHashMap;
        dense_hash_map<string,int> busHashMap;
        dense_hash_map<string,int> layerHashMap;
        dense_hash_map<size_t,int> trackHashMap;
        

        // Initializer
        Circuit() :
            design(""),
            runtime(INT_MAX), 
            alpha(INT_MAX),
            beta(INT_MAX),
            delta(INT_MAX),
            gamma(INT_MAX),
            epsilon(INT_MAX),
            originX(INT_MAX),
            originY(INT_MAX),
            width(INT_MIN),
            height(INT_MIN)
        {
            bitHashMap.set_empty_key(INIT_STR);
            busHashMap.set_empty_key(INIT_STR);
            layerHashMap.set_empty_key(INIT_STR);
            trackHashMap.set_empty_key(INT_MAX);
            
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

        // writer.cpp
        void def_write();
        void def_write(string filename);
        void lef_write();
        void lef_write(string filename);
        void out_write(string filename);

        // pin_aceess.cpp
        void pin_access();
        void pin_access(int busid);
        bool is_cross(Track* a, Track* b);
        bool is_intersect(Point _a, Point _b, Point _c, Point _d);
        bool rect_intersect(Point a, Point b, Point c, Point d);
        bool is_intersect(Track* a, Track* b);
        bool is_intersect(pair<pair<int,int>,pair<int,int> > line1, pair<pair<int,int>,pair<int,int> > line2);
        void wire_track_mapping();
        void pin_track_mapping();
        void ContactMapping();
        void debug();
        int ccw(pair<int,int> a, pair<int,int> b, pair<int,int> c);

        bgBoxT trackToBox(Track& _track);
        poly buffered_track(Track& track);

=======
>>>>>>> dykim
        // init.cpp 
        void initialize();
        void create_path();
        void print_all();

        bool is_vertical(int l){ return layers[l].is_vertical(); }

    };



};


#endif
