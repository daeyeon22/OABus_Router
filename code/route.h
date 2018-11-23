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
#include "heap.h"
#define rou OABusRouter::Router::shared()


namespace OABusRouter
{

    struct Tile
    {
        int id;
        int row, col, l;
        int x1, y1, x2, y2;
        int edgeCap;

        vector<int> neighbor;
    };


    struct Grid3D
    {
        int originX, originY;
        int width, height;
        int tileWidth, tileHeight;
        int numRows, numCols, numLayers;
        vector<Tile> tiles;


        
        Tile* get_tile(int col, int row, int l);
        int get_index(int col, int row, int l);
        int get_colum(int index);
        int get_row(int index);
        int get_layer(int index);

    };

    struct SearchArea
    {
        bool defined; 
        vector<int> lSeq;
        vector<BoxRtree> trees;
   
        SearchArea()
        {
            defined = false;
        }

        SearchArea(int numLayers) 
        {
            defined = false;
            trees = vector<BoxRtree>(numLayers);
        }

        SearchArea(const SearchArea& SA) :
            defined(SA.defined),
            lSeq(SA.lSeq),
            trees(SA.trees)
        {}
        bool check_lseq(int dep, int l);
        bool inside(int x, int y, int l, bool lCheck);
        bool inside(int x, int y);

        /*
        bool check_lseq(int dep, int l)
        {
            if(defined)
            {
                if(dep >= lSeq.size())
                    return false;
                else
                {
                    return lSeq[dep] == l ? true : false;
                }
            }
            else
            {
                return true;
            }
        }

        bool inside(int x, int y, int l)
        {
            if(defined)
            {
                vector<pair<box,int>> queries;
                trees[l].query(bgi::intersects(pt(x,y)), back_inserter(queries));
                return queries.size() > 0 ? true : false;
            }
            else
            {
                return true;
            }
        }

        bool inside(int x, int y)
        {
            if(defined)
            {
                for(int l=0; l < trees.size(); l++)
                {
                    vector<pair<box,int>> queries;
                    trees[l].query(bgi::intersects(pt(x,y)), back_inserter(queries));
                    return queries.size() > 0 ? true : false;
                }
                return false;
            }
            else
            {
                return true;
            }
        }
        */
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

        int totalSearchCount; 


        //Grid3D global_grid;
        Rtree_t rtree_t;
        Rtree_o rtree_o;
        
        // Created Segments
        vector<int>             trial;
        vector<int>             minSPV;
        vector<Segment>         segs;
        vector<Wire>            wires;
        vector<Via>             vias;
        vector<Topology>        topologies;
        vector<Grid3D>          tileGrids;
        

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
            totalSearchCount = 0;
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

        // Local Area
        bool get_search_area(int busid, int m1, int m2, int margin, SearchArea& SA);
        
        // Initialize Grid3D
        void construct_rtree();
        void construct_3d_tile_grids();

        // ILP
        void create_clips();
        void route_all();
        void ripup_reroute();
        void bus_ordering(int& startLoc, vector<int> &busSorted);

        // routing
        bool route_bus(int busid, int startLoc, int& numSPV);
        bool reroute_bus(int busid, int startLoc, int thSPV, int& numSPV);
        bool reroute_bus(int busid, int startLoc, int numTrial, int thSPV, int& numSPV);
        
        bool route_twopin_net(int busid, int m1, int m2, bool optimal, int& numSPV, vector<Segment> &tp);
        bool route_twopin_net(int busid, int m1, int m2, int ll[], int ur[], bool optimal, int &numSPV, vector<Segment> &tp);
        bool route_twopin_net_threshold_SPV(int busid, int m1, int m2, int ll[], int ur[], bool optimal, int thSPV, int &numSPV, vector<Segment> &tp);
        bool route_twopin_net_threshold_SPV(int busid, int m1, int m2, int ll[], int ur[], bool optimal, int numTrial, int thSPV, int &numSPV, vector<Segment> &tp);
        bool route_multipin_to_tp(int busid, int m, vector<Segment> &tp);


        // find heap node
        bool get_next_node(int busid, bool fixed, double hpwl, int lbs, HeapNode& curr, HeapNode& next);
        bool get_next_node_SPV_clean(int busid, bool fixed, double hpwl, int lbs, HeapNode& curr, HeapNode& next);
        bool get_next_node_debug(int busid, bool fixed, double hpwl, int lbs, HeapNode& curr, HeapNode& next);
        bool get_drive_node(int mp, bool last, HeapNode& curr);
        void get_estimate_cost(int m, double hpwl, int lbs, dense_hash_map<int,int>& bit2pin, HeapNode& curNode);
        bool is_destination(int n, int m, int p);
        bool is_optimal(int m, double hpwl, int lbs, dense_hash_map<int,int>& bit2pin, HeapNode& curNode);
        bool leftside(int m1, int m2);
        bool downside(int m1, int m2);

        //
        // for maze routing
        void get_access_rtree_nodes(int mp, int p, vector<int> &nodes);
        void get_access_point(int mp, bool last, HeapNode& curr);
        void pin_access_point(int xPin[], int yPin[], int lPin, int xSeg[], int ySeg[], int lSeg, int &x, int &y);

        //
        int lower_bound_num_segments(int m1, int m2, vector<int> &lSeq);
        int get_refbit_index(int m1, int m2);
        int get_routing_direction(int x1, int y1, int x2, int y2);
        int create_wire(int b, int t, int x1, int y1, int x2, int y2, int l, int seq, int width);


        void create_plots(const char* benchName);
        void create_bus_plot(bool all, int busid, const char* fileName);

        // ripup and reroute 
        void remove_all(int busid);
        void remove_wire(int wireid);
        void reconstruction(int busid, vector<int> &ws);

        int get_panelty_cost(int busid);
        int get_panelty_cost(vector<int> &ps);

        void update_net_tp(int busid, vector<Segment> &tp);

        double HPWL(int p1, int p2);
        double HPWL(int x, int y, int p);

        void debug_rtree();
    
    };

};








#endif
