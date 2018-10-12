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

namespace OABusRouter
{
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
        bool should_stop();


        void create_clips();
        void route_all();

        // for maze routing
        void get_drive_segment(int mp, vector<HeapNode> &nodes, vector<HeapNode> &next);
        void get_iterating_segment(HeapNode* current, vector<HeapNode> &nodes, vector<HeapNode> &next);
        void pin_access_point(int xPin[], int yPin[], int lPin, int xSeg[], int ySeg[], int lSeg, int &x, int &y);




    };

};








#endif
