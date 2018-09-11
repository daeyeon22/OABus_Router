#ifndef ROUTER_H
#define ROUTER_H
// Standard
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <climits>
#include <sparsehash/dense_hash_map>

// Boost
#include <boost/icl/interval_map.hpp>
#include <boost/icl/interval_set.hpp>
#include <boost/icl/interval_base_map.hpp>
#include <boost/geometry.hpp>

#include "rtree.h"

#define HORIZONTAL 222
#define VERTICAL 111 
#define PINTYPE -1212
#define OBSTACLE -1232
#define WIRETYPE -3319
#define NOT_ASSIGN 3391

#ifndef DTYPE       // Data type used by FLUTE
#define DTYPE int
#endif


#define rou OABusRouter::Router::shared()

using namespace std;
using google::dense_hash_map;
namespace bi = boost::icl;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// For Interval tree library
typedef set<int> IDSetT;
typedef bi::interval_map<int,IDSetT> IntervalMapT;
typedef bi::interval<int> IntervalT;
typedef bi::interval_set<int> IntervalSetT;
typedef bi::discrete_interval<int> DiscreteIntervalT;
// Boost geometries
typedef bg::model::point<float,2, bg::cs::cartesian> PointBG;
typedef bg::model::segment<PointBG> SegmentBG;
typedef bg::model::box<PointBG> BoxBG;
typedef bgi::rtree<pair<PointBG, int>, bgi::rstar<16>> PointRtree;
typedef bgi::rtree<pair<SegmentBG, int>, bgi::rstar<16>> SegRtree;
typedef bgi::rtree<pair<BoxBG, int>, bgi::rstar<16>> BoxRtree;

namespace OABusRouter
{
    struct RoutingTopology
    {
        int depth;
        vector<int> traceDir;
        vector<int> traceNuml;

    };

    struct SignalGroup
    {

        int id;
        int align1;
        int align2;

        set<int> buses;
        vector<int> pin1;
        vector<int> pin2;
    };

    
    struct Segment
    {
        int id;
        int x1, x2;
        int y1, y2;
        int l;
        int bw; 

        vector<int> junctions;  // junction id
        vector<int> neighbor;   // segment id
        vector<int> wires;      // segment id 
                                // sorted increasing sequence
        bool assign;
        bool vertical;
        bool leaf;
        
        Segment():
            id(INT_MAX),
            x1(INT_MAX),
            y1(INT_MAX),
            x2(INT_MIN),
            y2(INT_MIN),
            l(INT_MAX),
            bw(INT_MAX),
            assign(false),
            vertical(false),
            leaf(false)
        {}
        
        Segment(int id,
                int x1,
                int y1,
                int x2,
                int y2,
                int l,
                int bw,
                bool assign,
                bool vertical,
                bool leaf) :
            id(id),
            x1(x1),
            y1(y1),
            x2(x2),
            y2(y2),
            l(l),
            bw(bw),
            assign(assign),
            vertical(vertical),
            leaf(leaf)
        {}

        Segment(const Segment& s) :
            id(s.id),
            x1(s.x1),
            y1(s.y1),
            x2(s.x2),
            y2(s.y2),
            l(s.l),
            bw(s.bw),
            junctions(s.junctions),
            neighbor(s.neighbor),
            wires(s.wires), 
            assign(s.assign),
            vertical(s.vertical),
            leaf(leaf)
        {}
    };

    struct Topology
    {
        int busid;
        vector<Segment> segs;
    };


    struct Wire
    {
        int id;
        int x1, y1;
        int x2, y2;
        int l;
        int seq;        // sequence
        int width;
        int busid;
        int bitid;
        int trackid;
        bool vertical;
        bool pin;
        bool via;

        vector<int> neighbor;
        dense_hash_map<int, pair<int,int>> intersection;

        Wire()
        {
            intersection.set_empty_key(INT_MAX);
        }

        bool leaf();
        void get_info(int b, int t, int x[], int y[], int curl, int s, bool accpin);
        void add_intersection(int key, pair<int,int> p);
    };

    struct Via
    {
        int id;
        int bitid;
        int busid;
        int x, y;
        int l;

        Via():
            id(INT_MAX),
            bitid(INT_MAX),
            busid(INT_MAX),
            x(INT_MAX),
            y(INT_MAX),
            l(INT_MAX)
        {}

        Via(const Via& v):
            id(v.id),
            bitid(v.bitid),
            busid(v.busid),
            x(v.x),
            y(v.y),
            l(v.l) {}
    };

    class Router
    {
      private:
        static Router* instance;

      public:
        static Router* shared();

        //RSMT        rsmt;
        //Grid3D      grid;
        //Rtree       rtree;
        TrackRtree      rtree_t;
        ObstacleRtree   rtree_o;
        PinRtree        rtree_p;
        SegmentRtree    rtree_s;


        // Created Segments
        vector<Segment>         segs;
        //vector<Junction>        junctions;
        vector<Wire>            wires;
        vector<Via>             vias;
        vector<Topology>        topologies;


        // Hash map
        //dense_hash_map<int,int> seg2multipin;   // ??
        dense_hash_map<int,int> seg2top;
        dense_hash_map<int,int> wire2seg;
        dense_hash_map<int,int> spacing;
        //dense_hash_map<int,int> seg2bus;
        dense_hash_map<int,int> wire2pin;       // ??
        
        //dense_hash_map<int,int> junc2bus;
        //dense_hash_map<int,int> via2bus;
        //dense_hash_map<int,int> multipin2seg;
        dense_hash_map<int,int> pin2wire;
        dense_hash_map<int,int> pin2bit;
        dense_hash_map<int,int> pin2bus;


        dense_hash_map<int,int> multipin2llx;
        dense_hash_map<int,int> multipin2lly;
        dense_hash_map<int,int> multipin2urx;
        dense_hash_map<int,int> multipin2ury;
        //dense_hash_map<int,int> bitwidth;
        //dense_hash_map<int,bool> assign;

        Router()
        {
            //seg2multipin.set_empty_key(0);
            spacing.set_empty_key(INT_MAX);
            //multipin2seg.set_empty_key(INT_MAX);
            pin2wire.set_empty_key(INT_MAX);
            pin2bit.set_empty_key(INT_MAX);
            pin2bus.set_empty_key(INT_MAX);
            //seg2bus.set_empty_key(INT_MAX);
            //junc2bus.set_empty_key(INT_MAX);
            //via2bus.set_empty_key(INT_MAX);
            wire2pin.set_empty_key(INT_MAX);
            //bitwidth.set_empty_key(0);
            //assign.set_empty_key(0);
            multipin2llx.set_empty_key(INT_MAX);
            multipin2lly.set_empty_key(INT_MAX);
            multipin2urx.set_empty_key(INT_MAX);
            multipin2ury.set_empty_key(INT_MAX);
        }

        // Initialize Grid3D
        void initialize();
        //void initialize_rtree();
        //void initialize_grid3d();
        void initialize_rtree_new();
        // Generate Initial Topology for each bus
        //void gen_backbone();
    
        // Mapping 3D
        //void topology_mapping3d();
        
        //void ObstacleAwareRouting(int treeid);
        
        //bool Routing();
        //bool ObstacleAwareBusRouting(int busid);
        //void pin_access(int bitid);

        // ILP
        //void CreateClips();
        //void SolveILP();
        //void SolveILP_v2();
        //void PostGlobalRouting();


        // Detailed
        //void track_assign();
        //void mapping_pin2wire();
        //void mapping_multipin2seg();
        //void cut();


        int get_congested_bus(int busid);
        int create_wire(int bitid, int trackid, int x[], int y[], int l, int seq, bool pin);


        //void RouteAll();
        //void TrackAssign();
        //void CreateVia();
        //void MappingPin2Wire();
        //void MappingMultipin2Seg();
        //void Cut();
        // Make Plot
        void route_all();
        void rip_up(int busid);
        void intersection_pin(int pinx[], int piny[], int l1, int wirex[], int wirey[], int l2, int iterx, int itery, int &x, int &y);
        void sort_pins_routing_sequence(int m1, int m2, vector<int>& sorted1, vector<int>& sorted2);
        void sort_pins_routing_sequence(int m1, int m2, bool reverse, vector<int>& sorted1, vector<int>& sorted2);
        void update_net_tp(int busid, vector<Segment>& tp);
        void create_plot(const char* benchName);
        void local_search_area(int m1, int m2, int count, int ll[], int ur[]);
        void range_of_tj(Segment& target, int ll[], int ur[]);
        void wire_reordering(int busid, vector<Segment>& tp);
        //void SetNeighbor(Wire* w1, Wire* w2, int x, int y);
        //bool Intersection(Wire* w1, Wire* w2, int &x, int &y);
        
        //Wire* CreateWire(int bitid, int trackid, int x[], int y[], int l, int seq, bool pin);
        //bool ValidUpdate(int wireid, int x[], int y[]);
        //bool UpdateWire(int wireid, int x[], int y[]);
    
        bool route_bus(int busid);
        // bus -> example1 ~ 1_4
        bool route_twopin_net(int busid, int m1, int m2, vector<Segment>& tp);
        // too many spacing violations
        bool route_twopin_net_v2(int busid, int m1, int m2, vector<Segment>& tp);
        bool route_twopin_net_v3(int busid, int m1, int m2, vector<Segment>& tp);
        bool route_twopin_net_v4(int busid, int m1, int m2, vector<Segment>& tp);
        // 
        //
        bool route_twopin_net_v5(int busid, int m1, int m2, vector<Segment>& tp);
        bool route_twopin_net_v6(int busid, int m1, int m2, vector<Segment>& tp);
        bool route_multipin_to_tp(int busid, int m, vector<Segment>& tp);
        bool reroute(int busid);
        bool set_neighbor(int w1, int w2, int x, int y);
        bool get_intersection(int w1, int w2, int &x, int &y);
        bool t_junction_available(int busid, int x[], int y[], int l);
        bool routability_check(int m, int t, int dir);
        bool get_middle_seg(int s1, int s2, int& tar); 
        void penalty_cost();
        void remove_all(int busid);
        void remove_wire(int wireid);

        bool should_stop();
        void construct_bit_rtree(int bitid, BitRtree& bitrtree);

    };

};








#endif
