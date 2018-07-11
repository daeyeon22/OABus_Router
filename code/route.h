#ifndef ROUTER_H
#define ROUTER_H


#ifndef PREDEF
#define PREDEF
#define HORIZONTAL 222
#define VERTICAL 111 
#endif


#ifndef DTYPE       // Data type used by FLUTE
#define DTYPE int
#endif

#define rou OABusRouter::Router::shared()



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
            l(node.z),
            steiner(node.isSteiner),
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
        int x1, x2;
        int y1, y2;
        int z;
        int bitID;

        Segment(){}
        Segment(int x_1 = INT_MAX, 
                int y_1 = INT_MAX,
                int x_2 = INT_MAX,
                int y_2 = INT_MAX,
                int bit_ID = INT_MAX) :
            x1(x_1),
            y1(y_1),
            x2(x_2),
            y2(y_2),
            bitID(bit_ID) 
        {}
        Segment(const Segment& seg) :
            x1(seg.x1),
            y1(seg.y1),
            x2(seg.x2),
            y2(seg.y2),
            bitID(seg.bitID)
        {}


    };


    struct StTree
    {
        //int index;
        int numNodes;
        int numEdges;
        int length;
        
        vector<TreeNode> nodes;
        vector<TreeEdge> edges;
           
        StTree() :
            //index(INT_MAX),
            numNodes(INT_MAX),
            numEdges(INT_MAX),
            length(INT_MAX)
        {}

        StTree(const StTree& tree) :
            //index(tree.index),
            numNodes(tree.numNodes),
            numEdges(tree.numEdges),
            length(tree.length),
            nodes(tree.nodes),
            edges(tree.edges)
        {}


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
        dense_hash_map<int,int> mapID;  // Map bus -> tree index

        // Default Constructor
        RSMT()
        {
            mapID.set_empty_key(0);
        }

        // 
        StTree* operator[] (int index)
        {
            return &trees[mapID[index]];
        }
        
        // Create RSMT and Push into vector
        void CreateTree(int id, int d, DTYPE x[], DTYPE y[], DTYPE l[], int acc, float coeffV);
    
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
        }

        // Copy constructor
        Grid3D(const Grid3d& grid):
            numCols(grid.numCols),
            numRows(grid.numRows),
            numLayers(grid.numLayers),
            GCELL_WIDTH(grid.GCELL_WIDTH),
            GCELL_HEIGHT(grid.GCELL_HEIGHT),
            xoffset(grid.xoffset),
            yoffset(grid.yoffset),
            gcells(grid.gcells),
            direction(grid.direction)
        {}

        // Initialize function
        void InitGcellCap(int l, int dir, vector<int> &offsets);


        // member functions
        int GetIndex(int col, int row, int layer);
        int GetOffsetx(int col);
        int GetOffsety(int row);
        int GetColum(int crd);
        int GetRow(int crd);

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

        // Initialize Grid3D
        void InitGrid3D();
    
        // Generate Initial Topology for each bus
        void GenBackbone();
        void GenStTree(int id, DTYPE x[], DTYPE y[], int l[]);

    };


};








#endif
