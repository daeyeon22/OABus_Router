#ifndef ROUTER_H
#define ROUTER_H
// Standard
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <climits>

#include "typedef.h"
#include "tree.h"

#define rou OABusRouter::Router::shared()

namespace OABusRouter
{
    
    struct HeapNode
    {
        int id;
        int depth;
        int backtrace;
        int ref;
        
        bool isDest;

        double CR;
        double PS;
        vector<int> nodes;
        vector<int> xPrev;
        vector<int> yPrev;
        vector<int> xLast;
        vector<int> yLast;

        HeapNode()
        {}

        HeapNode(int numBits) :
            depth(0),
            backtrace(-1),
            CR(DBL_MAX),
            PS(DBL_MAX)
        {
            nodes = vector<int>(numBits, -1);
            xPrev = vector<int>(numBits, -1);
            yPrev = vector<int>(numBits, -1);
        }

        HeapNode(const HeapNode& node):
            id(node.id),
            depth(node.depth),
            backtrace(node.backtrace),
            isDest(node.isDest),
            CR(node.CR),
            PS(node.PS),
            nodes(node.nodes),
            xPrev(node.xPrev),
            yPrev(node.yPrev),
            xLast(node.xLast),
            yLast(node.yLast)
        {}

        double cost();
    };


    struct Heap
    {
        struct Comparison
        {
            bool operator() (HeapNode& n1, HeapNode& n2)
            {
                return n1.cost() > n2.cost();
            }
        };

        priority_queue<HeapNode, vector<HeapNode>, Comparison> PQ;

        // member functions
        void push(vector<HeapNode> &next);
        void push(HeapNode &next);
        void pop();
        bool empty();
        int size();
        HeapNode top();

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
        int SPACING_VIOLATION;
        int VIA_COST;
        int DEPTH_COST;
        int NOTCOMPACT;
        
        Rtree_t rtree_t;
        Rtree_o rtree_o;
        
        // Created Segments
        vector<Segment>         segs;
        vector<Wire>            wires;
        vector<Via>             vias;
        vector<Topology>        topologies;


        // Hash map
        dense_hash_map<int,int> seg2top;
        dense_hash_map<int,int> wire2seg;
        dense_hash_map<int,int> spacing;
        dense_hash_map<int,int> wire2pin;       // ??
        
        dense_hash_map<int,int> pin2wire;
        dense_hash_map<int,int> pin2bit;
        dense_hash_map<int,int> pin2bus;
        dense_hash_map<int,int> pin2align;
        dense_hash_map<int,int> bit2bus;

        dense_hash_map<int,int> multipin2llx;
        dense_hash_map<int,int> multipin2lly;
        dense_hash_map<int,int> multipin2urx;
        dense_hash_map<int,int> multipin2ury;
        //dense_hash_map<int,int> bitwidth;
        //dense_hash_map<int,bool> assign;

        Router()
        {
            spacing.set_empty_key(INT_MAX);
            pin2wire.set_empty_key(INT_MAX);
            pin2bit.set_empty_key(INT_MAX);
            pin2bus.set_empty_key(INT_MAX);
            pin2align.set_empty_key(INT_MAX);
            bit2bus.set_empty_key(INT_MAX);
            wire2pin.set_empty_key(INT_MAX);
            multipin2llx.set_empty_key(INT_MAX);
            multipin2lly.set_empty_key(INT_MAX);
            multipin2urx.set_empty_key(INT_MAX);
            multipin2ury.set_empty_key(INT_MAX);
        }

        // Initialize Grid3D
        void construct_rtree();


        // ILP
        void create_clips();
        void route_all();

        // routing
        bool route_bus(int busid);
        bool route_twopin_net(int busid, int m1, int m2, vector<Segment> &tp);
        bool route_multipin_to_tp(int busid, int m, vector<Segment> &tp);


        // find heap node
        bool get_next_node(int busid, bool fixed, HeapNode& curr, HeapNode& next);
        bool get_drive_node(int mp, bool last, HeapNode& curr);
        bool is_destination(int n, int p);


        //
        // for maze routing
        void get_access_rtree_nodes(int mp, int p, vector<int> &nodes);
        void get_access_point(int mp, bool last, HeapNode& curr);
        void pin_access_point(int xPin[], int yPin[], int lPin, int xSeg[], int ySeg[], int lSeg, int &x, int &y);

        //
        int get_routing_direction(int x1, int y1, int x2, int y2);
        int create_wire(int b, int t, int x1, int y1, int x2, int y2, int l, int seq, int width);


        void create_plots(const char* benchName);
        void create_bus_plot(bool all, int busid, const char* fileName);



    };

};








#endif
