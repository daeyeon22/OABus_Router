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


#ifndef PREDEF
#define PREDEF
#define HORIZONTAL 222
#define VERTICAL 111 
#endif

#ifndef DTYPE       // Data type used by FLUTE
#define DTYPE int
#endif

#define rou OABusRouter::Router::shared()

using namespace std;
using google::dense_hash_map;
namespace bi = boost::icl;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

//namespace bi = boost::icl;

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
typedef pair<PointBG,int> PointValT;
typedef pair<SegmentBG,int> SegmentValT;
typedef pair<BoxBG,int> BoxValT;

typedef bgi::rtree<PointValT, bgi::rstar<16>> PointRtree;
typedef bgi::rtree<SegmentValT, bgi::rstar<16>> SegRtree;
typedef bgi::rtree<BoxValT, bgi::rstar<16>> BoxRtree;


namespace OABusRouter
{

   



    struct TreeNode
    {
        int id;             // index
        int n;              // parent node
        int x, y, l;        // 
        bool steiner;       // TRUE if Steiner point
        vector<int> neighbor;
        vector<int> edge;

        TreeNode() :
            id(INT_MAX),
            n(INT_MAX),
            x(INT_MAX),
            y(INT_MAX),
            l(INT_MAX),
            steiner(false)
        {}
        
        TreeNode(const TreeNode& node) :
            id(node.id),
            n(node.n),
            x(node.x),
            y(node.y),
            l(node.l),
            steiner(node.steiner),
            neighbor(node.neighbor),
            edge(node.edge)
        {}

    
    };

    struct TreeEdge
    {
        int length;         // manhatan distance
        int n1;             // node1
        int n2;             // node2
        
        TreeEdge() : 
            length(INT_MAX),
            n1(INT_MAX),
            n2(INT_MAX) 
        {}
    };





    struct StTree
    {
        //int index;
        int id;
        int deg;
        int numNodes;
        int numEdges;
        int length;
        
        vector<TreeNode> nodes;
        vector<TreeEdge> edges;
        vector<int> segs;
        vector<int> junctions;

        dense_hash_map<int,int> node2multipin;

        StTree() :
            id(INT_MAX),
            deg(INT_MAX),
            numNodes(INT_MAX),
            numEdges(INT_MAX),
            length(INT_MAX)
        {
            node2multipin.set_empty_key(0);
        }

        StTree(const StTree& tree) :
            id(tree.id),
            deg(tree.deg),
            numNodes(tree.numNodes),
            numEdges(tree.numEdges),
            length(tree.length),
            nodes(tree.nodes),
            edges(tree.edges),
            segs(tree.segs),
            junctions(tree.junctions),
            node2multipin(tree.node2multipin)
        {}

        void print();

    };

    struct Gcell
    {
        int id;                     // index
        int x, y, l;                // index for x,y,z axis
        int cap;                    // edge capacitance
        int direction;
       
        set<int> resources;         // 

        //IntervalSetT available;

        
        Gcell() :
            id(INT_MAX),
            x(INT_MAX),
            y(INT_MAX),
            l(INT_MAX),
            cap(INT_MIN),
            direction(INT_MAX)
        {     
        }

        void print();
    };


    
    
    struct RSMT
    {
        
        //vector<int> indices;
        vector<StTree> trees;
        dense_hash_map<int,int> treeID;  // Map bus -> tree index
        dense_hash_map<int,int> busID;

        // Default Constructor
        RSMT()
        {
            treeID.set_empty_key(0);
            busID.set_empty_key(0);
        }

        // 
        StTree* operator[] (int index)
        {
            return &trees[treeID[index]];
        }
        
        // Create RSMT and Push into vector
        void CreateTree(int id, int d, int ids[], DTYPE x[], DTYPE y[], DTYPE l[], int acc, float coeffV);
        int GetBusID(int treeid);
        int GetTreeID(int busid);
    
    };

    struct Grid3D
    {
        int numCols;
        int numRows;
        int numLayers;
        int GCELL_WIDTH;
        int GCELL_HEIGHT;
        int xoffset;            // grid offset x (llx)
        int yoffset;            // grid offset y (lly)
        int width, height;

        vector<Gcell> gcells;
        vector<int> offsetxs;   // gcell offset x
        vector<int> offsetys;   // gcell offset y
        
        dense_hash_map<int,int> direction;


        // Default constructor
        Grid3D(int nc=0, int nr=0, int nl=0, 
                int gw=0, int gh=0, int xos=0, 
                int yos=0, int wd=0, int hg=0):
            numCols(nc),
            numRows(nr),
            numLayers(nl),
            GCELL_WIDTH(gw),
            GCELL_HEIGHT(gh),
            xoffset(xos),
            yoffset(yos),
            width(wd),
            height(hg)
        {
            direction.set_empty_key(0);
            // Offsets initialize
            for(int c=0; c < numCols; c++)
            {
                int llx = c*GCELL_WIDTH;
                offsetxs.push_back(llx);
            }
       
            for(int r=0; r < numRows; r++)
            {
                int lly = r*GCELL_HEIGHT;
                offsetys.push_back(lly);
            }
       
            // initialize gcells
            gcells = vector<Gcell>(numCols*numRows*numLayers, Gcell());
        }

        // Copy constructor
        Grid3D(const Grid3D& grid):
            numCols(grid.numCols),
            numRows(grid.numRows),
            numLayers(grid.numLayers),
            GCELL_WIDTH(grid.GCELL_WIDTH),
            GCELL_HEIGHT(grid.GCELL_HEIGHT),
            xoffset(grid.xoffset),
            yoffset(grid.yoffset),
            gcells(grid.gcells),
            direction(grid.direction),
            width(grid.width),
            height(grid.height)
        {}

        // Initialize function
        void CreateGCs();
        void InitGcellCap(int l, int dir, vector<int> &offsets);


        // member functions
        int GetIndex(int col, int row, int layer);
        int GetOffset_x(int col);
        int GetOffset_y(int row);
        int GetColum(int crd);
        int GetRow(int crd);
        int Capacity(int col, int row, int layer);


        void print();

        Gcell* operator [] (int index)
        {
            return &gcells[index];   
        }

    };
   
    struct Rtree
    {
        SegRtree track;
        


        //
        dense_hash_map<int,int> trackNuml;
        dense_hash_map<int,int> trackID;
        dense_hash_map<int,int> trackDir;

        Rtree()
        {
            trackNuml.set_empty_key(0);
            trackID.set_empty_key(0);
            trackDir.set_empty_key(0);
        }
    };


    struct Segment
    {
        int id;
        int x1, x2;
        int y1, y2;
        int l;
  

        vector<int> junctions;  // junction id
        vector<int> neighbor;   // segment id
        vector<int> wires;      // segment id 
                                // sorted increasing sequence

        bool assign;
        bool vertical;

        Segment(int _id = INT_MAX,
                int x_1 = INT_MAX, 
                int y_1 = INT_MAX,
                int x_2 = INT_MAX,
                int y_2 = INT_MAX,
                int _l = INT_MAX, 
                bool assign = false, 
                bool vertical = false) :
            id(_id),
            x1(x_1),
            y1(y_1),
            x2(x_2),
            y2(y_2),
            l(_l),
            assign(assign),
            vertical(vertical) {}

        Segment(const Segment& s) :
            id(s.id),
            x1(s.x1),
            y1(s.y1),
            x2(s.x2),
            y2(s.y2),
            l(s.l),
            junctions(s.junctions),
            neighbor(s.neighbor),
            wires(s.wires), 
            assign(s.assign),
            vertical(s.vertical) {}


    };


    struct Junction
    {
        int id;
        int x, y;
        int l1, l2;
        int s1, s2;
        int bw;

        Junction() :
            id(INT_MAX),
            x(INT_MAX),
            y(INT_MAX),
            l1(INT_MAX),
            l2(INT_MAX),
            s1(INT_MAX),
            s2(INT_MAX),
            bw(INT_MAX) {}

        Junction(const Junction& j) :
            id(j.id), 
            x(j.x),
            y(j.y),
            l1(j.l1),
            l2(j.l2),
            s1(j.s1),
            s2(j.s2),
            bw(j.bw) {}
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

        Wire():
            id(INT_MAX),
            x1(INT_MAX),
            y1(INT_MAX),
            y2(INT_MAX),
            l(INT_MAX),
            seq(INT_MAX),
            width(INT_MAX),
            busid(INT_MAX),
            bitid(INT_MAX),
            trackid(INT_MAX),
            vertical(false) {}
        
        Wire(const Wire& w):
            id(w.id),
            x1(w.x1),
            y1(w.y1),
            x2(w.x2),
            y2(w.y2),
            l(w.l),
            seq(w.seq),
            width(w.width),
            busid(w.busid),
            bitid(w.bitid),
            trackid(w.trackid),
            vertical(w.vertical) {}
    };

    struct Via
    {
        int id;
        int x, y;
        int l1, l2;
        int w1, w2;

        Via():
            id(INT_MAX),
            x(INT_MAX),
            y(INT_MAX),
            l1(INT_MAX),
            l2(INT_MAX),
            w1(INT_MAX),
            w2(INT_MAX) {}

        Via(const Via& v):
            id(v.id),
            x(v.x),
            y(v.y),
            l1(v.l1),
            l2(v.l2),
            w1(v.w1),
            w2(v.w2) {}

    };

    class Router
    {
      private:
        static Router* instance;

      public:
        static Router* shared();

        RSMT    rsmt;
        Grid3D  grid;
        Rtree   rtree;


        // Created Segments
        vector<Segment>         segs;
        vector<Junction>        junctions;
        vector<Wire>            wires;
        vector<Via>             vias;


        // Hash map
        //dense_hash_map<int,int> seg2multipin;   // ??
        dense_hash_map<int,int> seg2bus;
        //dense_hash_map<int,int> wire2pin;       // ??
        dense_hash_map<int,int> junc2bus;
        dense_hash_map<int,int> via2bus;
        dense_hash_map<int,int> multipin2seg;
        dense_hash_map<int,int> pin2wire;

        dense_hash_map<int,int> bitwidth;
        dense_hash_map<int,bool> assign;

        Router()
        {
            //seg2multipin.set_empty_key(0);
            multipin2seg.set_empty_key(0);
            pin2wire.set_empty_key(0);
            seg2bus.set_empty_key(0);
            junc2bus.set_empty_key(0);
            via2bus.set_empty_key(0);
            //wire2pin.set_empty_key(0);
            bitwidth.set_empty_key(0);
            assign.set_empty_key(0);
        }

        // Initialize Grid3D
        void InitGrid3D();
        void CreateTrackRtree();
        
        SegRtree* GetTrackRtree();

        // Generate Initial Topology for each bus
        void GenBackbone();
        void GenStTree(int id, DTYPE x[], DTYPE y[], int l[]);
    
        // Mapping 3D
        void TopologyMapping3D();

        // ILP
        void CreateClips();
        void SolveILP();
        
        // Detailed
        void TrackAssign();
        void CreateVia();
        void MappingPin2Wire();
        void MappingMultipin2Seg();

        // Make Plot
        void Plot();
    };


};








#endif
