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

//namespace bi = boost::icl;

// For Interval tree library
typedef set<int> IDSetT;
typedef bi::interval_map<int,IDSetT> IntervalMapT;
typedef bi::interval<int> IntervalT;
typedef bi::interval_set<int> IntervalSetT;
typedef bi::discrete_interval<int> DiscreteIntervalT;


namespace OABusRouter
{


    struct Clip
    {
        int numPanel;
        int curLayer;
        int direction;
        vector<IntervalMapT> intervalMaps;
        
        
        
        Clip() {}
    };


    

    
    



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


    struct Segment
    {
        int id;
        int x1, x2;
        int y1, y2;
        int l;
        Segment(int _id = INT_MAX,
                int x_1 = INT_MAX, 
                int y_1 = INT_MAX,
                int x_2 = INT_MAX,
                int y_2 = INT_MAX,
                int _l = INT_MAX) :
            id(_id),
            x1(x_1),
            y1(y_1),
            x2(x_2),
            y2(y_2),
            l(_l) {}

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


        StTree() :
            id(INT_MAX),
            deg(INT_MAX),
            numNodes(INT_MAX),
            numEdges(INT_MAX),
            length(INT_MAX)
        {}

        StTree(const StTree& tree) :
            id(tree.id),
            deg(tree.deg),
            numNodes(tree.numNodes),
            numEdges(tree.numEdges),
            length(tree.length),
            nodes(tree.nodes),
            edges(tree.edges)
        {}

        void print();

    };

    struct Gcell
    {
        int id;                     // index
        int x, y, l;                // index for x,y,z axis
        int cap;                    // edge capacitance
        
        Gcell() :
            id(INT_MAX),
            x(INT_MAX),
            y(INT_MAX),
            l(INT_MAX),
            cap(INT_MIN)
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
        void CreateTree(int id, int d, DTYPE x[], DTYPE y[], DTYPE l[], int acc, float coeffV);
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
                int urx = (c+1)*GCELL_WIDTH;
                offsetxs.push_back(urx);
            }
       
            for(int r=0; r < numRows; r++)
            {
                int ury = (r+1)*GCELL_HEIGHT;
                offsetys.push_back(ury);
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

        Gcell* operator [] (int index)
        {
            return &gcells[index];   
        }

    };
    
    class Router
    {
      private:
        static Router* instance;

      public:
        static Router* shared();

        RSMT    rsmt;
        Grid3D  grid;

        // Created Segments
        vector<Segment> segs;
        dense_hash_map<int,int> seg2bus;
        dense_hash_map<int,int> bitwidth;

        Router()
        {
            seg2bus.set_empty_key(0);
            bitwidth.set_empty_key(0);
        }

        // Initialize Grid3D
        void InitGrid3D();
    
        // Generate Initial Topology for each bus
        void GenBackbone();
        void GenStTree(int id, DTYPE x[], DTYPE y[], int l[]);
    
        // Mapping 3D
        void TopologyMapping3D();
        void CreateClips();
        void SolveILP();


        // Make Plot
        void Plot();
    };


};








#endif
