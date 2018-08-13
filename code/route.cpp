#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <set>
#include "circuit.h"
#include "route.h"
#include "func.h"


//#define DEBUG

#define DEBUG_ROUTE
//#define DEBUG_GRID




bool OABusRouter::Wire::leaf()
{
    return intersection.size() < 2;
}



// 3D mapping
bool OABusRouter::Router::ValidUpdate(int wireid, int x[], int y[])
{
    Wire* curwire = &wires[wireid];
    IntervalSetT empty = rtree.track_empty(curwire->trackid);
    if(curwire->vertical)
    {
        empty += IntervalT::closed(curwire->y1, curwire->y2);
#ifdef DEBUG_ROUTE
        if(!bi::within(IntervalT::open(y[0], y[1]), empty))
        {
            printf("Empty intervals ->\n{\n");
            for(auto& it : empty)
            {
                cout << it << endl;
            }
            printf("}\nQuery Interval (%d %d)\n", y[0], y[1]);

        }
#endif
        
        
        return bi::within(IntervalT::open(y[0], y[1]), empty);
    }
    else
    {
        empty += IntervalT::closed(curwire->x1, curwire->x2);
#ifdef DEBUG_ROUTE
        if(!bi::within(IntervalT::open(x[0], x[1]), empty))
        {
            printf("Empty intervals ->\n{\n");
            for(auto& it : empty)
            {
                cout << it << endl;
            }
            printf("}\nQuery Interval (%d %d)\n", x[0], x[1]);

        }
#endif
        
        
        return bi::within(IntervalT::open(x[0], x[1]), empty);
    }
}

bool OABusRouter::Router::UpdateWire(int wireid, int x[], int y[])
{
    Wire* curwire = &wires[wireid];
    int origXs[] = {curwire->x1, curwire->x2};
    int origYs[] = {curwire->y1, curwire->y2};
    
    rtree.insert_element(curwire->trackid, origXs, origYs, curwire->l, false);
    
    curwire->x1 = x[0];
    curwire->x2 = x[1];
    curwire->y1 = y[0];
    curwire->y2 = y[1];

    rtree.insert_element(curwire->trackid, x, y, curwire->l, true);
    return true;
}

OABusRouter::Wire* OABusRouter::Router::CreateWire(int bitid, int trackid, int x[], int y[], int l, int seq, bool pin)
{
    
    Wire w;
    w.id = wires.size();
    w.x1 = x[0];
    w.y1 = y[0];
    w.x2 = x[1];
    w.y2 = y[1];
    w.l = l;
    w.seq = seq;
    w.busid = ckt->busHashMap[ckt->bits[bitid].busName];
    w.bitid = bitid;
    w.width = ckt->buses[w.busid].width[l];
    w.trackid = trackid;
    w.vertical = ckt->is_vertical(l);
    w.pin = pin;
    w.via = (x[0]==x[1] && y[0]==y[1]) ? true : false;

    wires.push_back(w);

    // add into the bit and tree
    ckt->bits[w.bitid].wires.push_back(w.id);
    rsmt[rsmt.treeID[w.busid]]->wires.push_back(w.id);

    // rtree update
    rtree.insert_element(trackid, x, y, l, true);
    return &wires[w.id];
}


bool OABusRouter::Router::Intersection(Wire* w1, Wire* w2, int &x, int &y)
{
    typedef PointBG pt;
    typedef SegmentBG seg;

    vector<pt> intersection;
    seg s1(pt(w1->x1, w1->y1), pt(w1->x2, w1->y2));
    seg s2(pt(w2->x1, w2->y1), pt(w2->x2, w2->y2));
    bg::intersection(s1, s2, intersection);
    if(intersection.size() == 0)
        return false;
    else
    {
        x = (int)(bg::get<0>(intersection[0]) + 0.5);
        y = (int)(bg::get<1>(intersection[0]) + 0.5);
        return true;
    }
}

void OABusRouter::Router::SetNeighbor(Wire* w1, Wire* w2, int x, int y)
{

    int x2, y2;
    Intersection(w1, w2, x2, y2);
    if(x2 != x || y2 != y)
    {
        printf("orig (%d %d) value (%d %d)\n", x, y , x2, y2);
        printf("w1 (%d %d) (%d %d)\n", w1->x1, w1->y1, w1->x2, w1->y2);
        printf("w2 (%d %d) (%d %d)\n", w2->x1, w2->y1, w2->x2, w2->y2);
        exit(0);
    }
    
    
    w1->neighbor.push_back(w2->id);
    w2->neighbor.push_back(w1->id);
    w1->intersection[w2->id] = {x,y};
    w2->intersection[w1->id] = {x,y};
}

void OABusRouter::Router::TopologyMapping3D()
{

    // Variables
    int numTrees, numEdges, numNodes, numLayers, numNeighbor;
    int numRows, numCols; 
    int l1, l2, ubl, lbl, avgl, bw, idx;
    int x1, y1, x2, y2;
    int llx, lly, urx, ury;
    int verl, horl, curl;
    int col, row, lidx, curid;
    int segx1, segx2, segy1, segy2, segl, segid, busid;
    int numVerticals, numHorizontals;
    bool st1, st2, verline, horline, via, isVertical;
    
    TreeNode *n1, *n2, *curNode, *tarNode;
    TreeEdge *e;
    //Layer *verl, *horl, *curl;
    vector<int> verticals;
    vector<int> horizontals;
    dense_hash_map<int,int> vid;
    dense_hash_map<int,int> hid;
    dense_hash_map<int,int> direction;

    //numLayers = ckt->layers.size();
    numTrees = this->rsmt.trees.size();
    numCols = this->grid.numCols;
    numRows = this->grid.numRows;
    numLayers = this->grid.numLayers;
    vid.set_empty_key(INT_MAX);
    hid.set_empty_key(INT_MAX);
    direction.set_empty_key(INT_MAX);


    // Divide vertical, horizontal layer separately
    for(int i=0; i < numLayers; i++)
    {
        direction[i] = ckt->layers[i].direction;

        if(ckt->layers[i].is_vertical()){
            vid[ckt->layers[i].id] = verticals.size();
            verticals.push_back(ckt->layers[i].id);
        }else{
            hid[ckt->layers[i].id] = horizontals.size();
            horizontals.push_back(ckt->layers[i].id);
        }
    }

    numVerticals = verticals.size();
    numHorizontals = horizontals.size();

    // Create Segment for steiner tree
    for(int i=0; i < numTrees; i++)
    {
        StTree* tree = this->rsmt[i];
        busid = rsmt.GetBusID(tree->id); 
        bw = ckt->buses[busid].numBits;
        //
        // Assign layer for steiners
        numNodes = tree->numNodes;
        for(int d=tree->deg; d < numNodes; d++)
        {
            curNode = &tree->nodes[d];
            numNeighbor = curNode->neighbor.size();
            verline = false;
            horline = false;
            x1 = curNode->x;
            y1 = curNode->y;
            l1 = curNode->l;    // INT_MAX   
            avgl = 0;
            for(int n=0; n < numNeighbor; n++)
            {
                tarNode = &tree->nodes[curNode->neighbor[n]];
                x2 = tarNode->x;
                y2 = tarNode->y;
                l2 = tarNode->l;
                if(x1 != x2) horline = true;
                if(y1 != y2) verline = true;
                if(!tarNode->steiner) avgl += l2;
            }

            avgl = (int)(1.0*avgl/numNeighbor + 0.5);
            curNode->l = avgl;

            if(!verline || !horline)
            {
                cout << "Invalid steiner point..." << endl;
                //exit(0);
            }else{
            }
        }
       
        // Interval Sets
        // Remove overlapped Segment 
        cout << numVerticals << " " << numCols << endl;
        cout << numHorizontals << " " << numRows << endl;
        vector<IntervalSetT> verIntervalSet(numVerticals*numCols);//[numLayers][numCols];
        vector<IntervalSetT> horIntervalSet(numHorizontals*numRows);//[numLayers][numRows];

        //
        numEdges = tree->numEdges;
        for(int j=0; j < numEdges ; j++)
        {
               
            e = &tree->edges[j];
            n1 = &tree->nodes[e->n1];
            n2 = &tree->nodes[e->n2];

            //
            x1 = n1->x;
            y1 = n1->y;
            l1 = n1->l;
            x2 = n2->x;
            y2 = n2->y;
            l2 = n2->l;

            verline = false;
            horline = false;
            via = false;
            if(x1 != x2) horline = true;
            if(y1 != y2) verline = true;

            auto lamb = [&, l1, l2](int left, int right){
                int ldist, rdist;
                ldist = abs(l1-left) + abs(l2-left);
                rdist = abs(l1-right) + abs(l2-right);
                return ldist < rdist;
            };


            if(horline && !verline)
            {
                sort(horizontals.begin(), horizontals.end(), lamb);

                curl = horizontals[0];
                if(curl == l1){
                    segx1 = x1;
                    segy1 = y1;
                    segx2 = x2;
                    segy2 = y1;
                    segl = curl;
                }else if(curl == l2){
                    segx1 = x2;
                    segy1 = y2;
                    segx2 = x1;
                    segy2 = y2;
                    segl = curl;
                }else{
                    segx1 = x2;
                    segy1 = y2;
                    segx2 = x1;
                    segy2 = y2;
                    segl = curl;
                }
                
                llx = min(segx1, segx2);
                urx = max(segx1, segx2);
                lly = min(segy1, segy2);
                ury = max(segy1, segy2);
                if(lly != ury)
                {
                    cout << "Invalid line type.." << endl;
                    exit(0);
                }

                row = ury;
                curid = hid[segl]*numRows + row;                
                horIntervalSet[curid] += IntervalT::closed(llx, urx);

            }
            else if(!horline && verline)
            {
                sort(verticals.begin(), verticals.end(), lamb);

                curl = verticals[0];
                if(curl == l1){
                    segx1 = x1;
                    segy1 = y1;
                    segx2 = x1;
                    segy2 = y2;
                    segl = curl;

                }else if(curl == l2){
                    segx1 = x2;
                    segy1 = y2;
                    segx2 = x2;
                    segy2 = y1;
                    segl = curl;
                }else{
                    segx1 = x2;
                    segy1 = y2;
                    segx2 = x2;
                    segy2 = y1;
                    segl = curl;
                }


                llx = min(segx1, segx2);
                urx = max(segx1, segx2);
                lly = min(segy1, segy2);
                ury = max(segy1, segy2);
                if(llx != urx)
                {
                    cout << "Invalid line type.." << endl;
                    exit(0);
                }

                col = urx; // llx == urx
                curid = vid[segl]*numCols + col;
                verIntervalSet[curid] += IntervalT::closed(lly, ury);


            }
            else if(horline && verline)
            {
               sort(horizontals.begin(), horizontals.end(), lamb);

                bool n1first;
                //= true;
                curl = horizontals[0];
                if(curl == l1){
                    segx1 = x1;
                    segy1 = y1;
                    segx2 = x2;
                    segy2 = y1;
                    segl = curl;
                    n1first = true;
                }else if(curl == l2){
                    segx1 = x2;
                    segy1 = y2;
                    segx2 = x1;
                    segy2 = y2;
                    segl = curl;
                    n1first = false;
                }else{
                    segx1 = x2;
                    segy1 = y2;
                    segx2 = x1;
                    segy2 = y2;
                    segl = curl;
                    n1first = false;
                }
                
                llx = min(segx1, segx2);
                urx = max(segx1, segx2);
                lly = min(segy1, segy2);
                ury = max(segy1, segy2);
                if(lly != ury)
                {
                    cout << "Invalid line type.." << endl;
                    exit(0);
                }

                row = ury;
                curid = hid[segl]*numRows + row;                
                horIntervalSet[curid] += IntervalT::closed(llx, urx);

                sort(verticals.begin(), verticals.end(), [segl, l1, l2, n1first] (int left, int right){
                        if(n1first)
                            return (abs(segl - left) + abs(segl - right)) < (abs(l2 - left) + abs(l2 - right));
                        else
                            return (abs(segl - left) + abs(segl - right)) < (abs(l1 - left) + abs(l1 - right));
                        });


                curl = verticals[0];
                if(n1first)
                {
                    segx1 = x2;
                    segy1 = y1;
                    segx2 = x2;
                    segy2 = y2;
                    segl = curl;
                }
                else
                {
                    segx1 = x1;
                    segy1 = y2;
                    segx2 = x1;
                    segy2 = y1;
                    segl = curl;
                }

                llx = min(segx1, segx2);
                urx = max(segx1, segx2);
                lly = min(segy1, segy2);
                ury = max(segy1, segy2);
                if(llx != urx)
                {
                    cout << "Invalid line type.." << endl;
                    exit(0);
                }

                col = urx; // llx == urx
                curid = vid[segl]*numCols + col;
                verIntervalSet[curid] += IntervalT::closed(lly, ury);
            }
            else
            {
                cout << "no horizontal, vertical line" << endl;

            }


        }
        //
        //

       
        
        int numQ, segindex;
        int jid, x, y, l1, l2, s1, s2;
        bool vertical_s1, vertical_s2;
        SegmentBG seg1, seg2;
        PointBG pt;
        vector<PointBG> intersection;
        SegRtree sRtree;
        Segment* curS, *tarS;
        //Bus* curBus;
        vector<SegmentValT> queries;
        dense_hash_map<int,int>     segNuml;
        segNuml.set_empty_key(INT_MAX);
        //bw = bitwidth[busid];

        //
        //
        // Create segment
        
        for(lidx = 0; lidx < numLayers; lidx++)
        {
             
            isVertical = (direction[lidx] == VERTICAL)?true:false;

            if(isVertical)
            {
                // vertical lines
                for(col = 0; col < numCols; col++)
                {

                    idx = vid[lidx]*numCols + col;
                    // Start iterating
                    IntervalSetT &curSet = verIntervalSet[idx];
                    IntervalSetT::iterator it = curSet.begin();
                    DiscreteIntervalT intv;
                    while(it != curSet.end())
                    {
                        intv = (*it++);
                        segid = this->segs.size();
                        segx1 = col;
                        segy1 = intv.lower();
                        segx2 = col;
                        segy2 = intv.upper();
                        segl = lidx;

                        // Mapping the bitwith for each segment
                        busid = this->rsmt.GetBusID(tree->id);
                        //curBus = &ckt->buses[busid];
                        //bw = curBus->numBits;

                        Segment seg(segid, segx1, segy1, segx2, segy2, segl, bw, false, isVertical);
                        this->segs.push_back(seg);
                        tree->segs.push_back(segid);

                        //this->bitwidth[segid] = bw;
                        this->seg2bus[segid] = busid;
                        //this->assign[segid] = false;

                        // Rtree
                        SegmentBG segbg(PointBG(segx1, segy1), PointBG(segx2, segy2));
                        segNuml[segid] = segl;
                        sRtree.insert(SegmentValT(segbg, segid));
                    }

                }
            }else{
                // horizontal lines
                for(row = 0; row < numRows; row++)
                {
                    idx = hid[lidx]*numRows + row;
                    // Start iterating
                    IntervalSetT &curSet = horIntervalSet[idx];
                    IntervalSetT::iterator it = curSet.begin();
                    DiscreteIntervalT intv;
                    while(it != curSet.end())
                    {
                        intv = (*it++);
                        segid = this->segs.size();
                        segx1 = intv.lower();
                        segy1 = row;
                        segx2 = intv.upper();
                        segy2 = row;
                        segl = lidx;

                        // Mapping the bitwith for each segment
                        busid = this->rsmt.GetBusID(tree->id);
                        //curBus = &ckt->buses[busid];
                        //bw = curBus->numBits;
                        
                        Segment seg(segid, segx1, segy1, segx2, segy2, segl,bw, false, isVertical);
                        this->segs.push_back(seg);
                        tree->segs.push_back(segid);

                        //this->bitwidth[segid] = bw;
                        this->seg2bus[segid] = busid;
                        //this->assign[segid] = false;
                    
                    
                        // Rtree
                        SegmentBG segbg(PointBG(segx1, segy1), PointBG(segx2, segy2));
                        segNuml[segid] = segl;
                        sRtree.insert(SegmentValT(segbg, segid));
                    }

                }
            }

        //


        }
        //

#ifdef DEBUG
        if(tree->segs.size() != sRtree.size())
        {
            cout << "differenct size..." << endl;
            exit(0);
        }

        cout << "Segment list = ";
        for(auto& it : tree->segs)
        {
            cout << it << " ";   
        }

        cout << endl << endl;
#endif

        // Create junction
        for(segindex=0; segindex < tree->segs.size(); segindex++)
        {
            queries.clear();
            s1 = tree->segs[segindex];
            l1 = segNuml[s1];            
            
            curS = &segs[s1];
            segl = curS->l;
            segx1 = curS->x1;
            segy1 = curS->y1;
            segx2 = curS->x2;
            segy2 = curS->y2;
            seg1 = SegmentBG(PointBG(segx1, segy1), PointBG(segx2, segy2));
            
            //
            sRtree.remove(SegmentValT(seg1, s1));
            sRtree.query(bgi::intersects(seg1), back_inserter(queries));
            
            numQ = queries.size();

            // 
            for(int k=0; k < numQ; k++)
            {
                jid = junctions.size();
                seg2 = queries[k].first;
                s2 = queries[k].second;
                l2 = segNuml[s2];
                tarS = &segs[s2];

                //bool adj = ( (l1+1 == l2) || (l1 == l2+1) )? true : false;
                //if(!adj) continue;
                if(s2 == s1) continue;
                //if(l1+1 == l2) continue;

                intersection.clear();
                bg::intersection(seg1, seg2, intersection);

                if(intersection.size() == 0)
                {
                    cout << "Invalid segments..." << endl;
                    exit(0);
                }

                pt = intersection[0];
                x = (int)(bg::get<0>(pt) + 0.5);
                y = (int)(bg::get<1>(pt) + 0.5);
                if(l1 > l2)
                {
                    swap(s1, s2);
                    swap(l1, l2);
                }
                
                // Junction
                Junction junc;
                junc.id = jid;
                junc.x = x;
                junc.y = y;
                junc.l1 = l1;
                junc.l2 = l2;
                junc.s1 = s1;
                junc.s2 = s2;
                junc.bw = bw;

                // Add junction into the vector, and mapping
                junctions.push_back(junc);
                tree->junctions.push_back(junc.id);
                junc2bus[jid] = busid;
                
                // Add neighbor 
                curS->neighbor.push_back(s2);
                tarS->neighbor.push_back(s1);
                curS->junctions.push_back(jid);
                tarS->junctions.push_back(jid);
            }
            //sRtree.remove(SegmentValT(seg1, s1));
        }


        sRtree.clear();
        //printf("\n\n");
#ifdef DEBUG_ROUTE
        tree->print();
#endif
        //
    }


}

// Print StTree
void OABusRouter::StTree::print()
{

    int i;
   

    printf("Current tree(%d) -> %s\n\n", id, ckt->buses[rou->rsmt.busID[id]].name.c_str());
    for (i=0; i<deg; i++)
        printf("(p)%-2d:  x=%4g  y=%4g  z=%4g  e=%d\n",
                i, (float) nodes[i].x, (float) nodes[i].y, (float) nodes[i].l, nodes[i].n);
    for (i=deg; i<2*deg-2; i++)
        printf("(s)%-2d:  x=%4g  y=%4g  z=%4g  e=%d\n",
                i, (float) nodes[i].x, (float) nodes[i].y, (float) nodes[i].l, nodes[i].n);
    
    for(i=0; i < segs.size(); i++)
    {
        Segment* curS = &rou->segs[segs[i]];
        if(curS->x1 == curS->x2)
        {
            printf("(%d) s %d V-Line (%2d %2d) (%2d %2d) m%d -> b%d\n", i, curS->id, curS->x1, curS->y1, curS->x2, curS->y2, curS->l, rou->seg2bus[curS->id]);
        }else{
            printf("(%d) s %d H-Line (%2d %2d) (%2d %2d) m%d -> b%d\n", i, curS->id, curS->x1, curS->y1, curS->x2, curS->y2, curS->l, rou->seg2bus[curS->id]);
        }
    }
    

    for(i=0; i < junctions.size(); i++)
    {
        Junction* curJ = &rou->junctions[junctions[i]];
        printf("(%d) Junction (%2d %2d) M%d M%d Edge (%d %d)\n", i, curJ->x, curJ->y, curJ->l1, curJ->l2, curJ->s1, curJ->s2);

    }

    printf("\n\n\n");


}


// member functions for Grid3D
int OABusRouter::Grid3D::GetIndex(int col, int row, int layer)
{
    return col + numCols*row + (numCols*numRows)*layer;
}

int OABusRouter::Grid3D::GetOffset_x(int col)
{
    return offsetxs[col]; // - GCELL_WIDTH; //*col + xoffset;
}

int OABusRouter::Grid3D::GetOffset_y(int row)
{
    return offsetys[row]; // - GCELL_HEIGHT; //*row + yoffset;
}

int OABusRouter::Grid3D::GetColum(int xcrd)
{
    return max(GetLowerBound(offsetxs, xcrd) - 1, 0);
}

int OABusRouter::Grid3D::GetRow(int ycrd)
{
    return max(GetLowerBound(offsetys, ycrd) - 1, 0);
}

int OABusRouter::Grid3D::llx(int gcellid)
{
    return GetOffset_x(gcells[gcellid].x);
}

int OABusRouter::Grid3D::lly(int gcellid)
{
    return GetOffset_y(gcells[gcellid].y);
}

int OABusRouter::Grid3D::urx(int gcellid)
{
    return GetOffset_x(gcells[gcellid].x + 1);
}

int OABusRouter::Grid3D::ury(int gcellid)
{
    return GetOffset_y(gcells[gcellid].y + 1);
}



int OABusRouter::Grid3D::Capacity(int col, int row, int layer)
{
    return gcells[GetIndex(col, row, layer)].cap;
}


void OABusRouter::Router::Init()
{
    for(auto& mp : ckt->multipins)
    {
        int llx, lly, urx, ury;
        llx = INT_MAX;
        lly = INT_MAX;
        urx = INT_MIN;
        ury = INT_MIN;
        for(auto& pinid : mp.pins)
        {
            Pin* curpin = &ckt->pins[pinid];
            llx = min(llx, curpin->llx);
            lly = min(lly, curpin->lly);
            urx = max(urx, curpin->urx);
            ury = max(ury, curpin->ury);
        }


        multipin2llx[mp.id] = llx;
        multipin2lly[mp.id] = lly;
        multipin2urx[mp.id] = urx;
        multipin2ury[mp.id] = ury;
    }

    InitRtree();
    InitGrid3D();
}


// Initialize Grid3D
void OABusRouter::Router::InitGrid3D()
{
    int minV, minH, minNumBit, maxNumBit;
    int numlayers, numbuses;
    int l_ver, l_hor;
    vector<int> xoffsets;
    vector<int> yoffsets;

    minV = INT_MAX;
    minH = INT_MAX;
    minNumBit = INT_MAX;
    maxNumBit = INT_MIN;
    numlayers = ckt->layers.size();
    numbuses = ckt->buses.size();

    for(int i=0; i < numlayers; i++)
    {
        Layer* curl = &ckt->layers[i];
        spacing[curl->id] = curl->spacing;
        
        if(curl->is_vertical())
        {

            if(minV > curl->offsets.size())
            {
                xoffsets.clear();
                minV = curl->offsets.size();
                xoffsets.insert(xoffsets.end(), curl->offsets.begin(), curl->offsets.end()); // = curl->offsets;
                l_ver = curl->id;
            }
        }
        else
        {

            if(minH > curl->offsets.size())
            {
                yoffsets.clear();
                minH = curl->offsets.size();
                yoffsets.insert(yoffsets.end(), curl->offsets.begin(), curl->offsets.end());
                //hoffsets = curl->offsets;
                l_hor = curl->id;
            }
        }
    }

    sort(xoffsets.begin(), xoffsets.end());
    sort(yoffsets.begin(), yoffsets.end());
    for(int i=0; i < numbuses; i++)
    {
        Bus* curB = &ckt->buses[i];
        minNumBit = min(curB->numBits, minNumBit);
        maxNumBit = max(curB->numBits, maxNumBit);
    }




    int numcols = (int)(1.0* minV / maxNumBit);
    int numrows = (int)(1.0* minH / maxNumBit);
    //int numcols = (int)(1.0* minH / maxNumBit);
    //int numrows = (int)(1.0* minV / maxNumBit);
    int originX = ckt->originX;
    int originY = ckt->originY;

#ifdef DEBUG_GRID
    for(auto& it : xoffsets) printf("x offset %d\n", it);
    for(auto& it : yoffsets) printf("y offset %d\n", it);
    printf("max num bits %d\nmin num bits %d\nmin vertical offsets %d\nmin horizontal offsets %d\n",
            maxNumBit, minNumBit, minV, minH);
    printf("#cols %d #rows %d\n", numcols, numrows);
#endif


    for(int i=0; i < numcols; i++)
    {
        int lower = maxNumBit*i;
        int lx = (lower==0)? originX : (int)(1.0*(xoffsets[lower] + xoffsets[lower-1])/2);
        grid.offsetxs.push_back(lx);
        //printf("x offset %d\n",lx);
    }

    for(int i=0; i < numrows; i++)
    {
        int lower = maxNumBit*i;
        int ly = (lower==0)? originY : (int)(1.0*(yoffsets[lower] + yoffsets[lower-1])/2);
        grid.offsetys.push_back(ly);
        //printf("y offset %d\n", ly);
    }

    grid.offsetxs.push_back(originX + ckt->width);
    grid.offsetys.push_back(originY + ckt->height);

    grid.numCols = numcols;
    grid.numRows = numrows;
    grid.numLayers = numlayers;
    grid.xoffset = originX;
    grid.yoffset = originY;
    grid.width = ckt->width;
    grid.height = ckt->height;
    grid.gcells = vector<Gcell>(numcols*numrows*numlayers, Gcell());

#ifdef DEBUG
    ////////////////////////////////////
    printf("GCell (%4d %4d) col: %3d row: %3d\n", GCELL_WIDTH, GCELL_HEIGHT, numCols, numRows);
    for(auto& os : this->grid.offsetxs)
    {
        printf("OffsetX %4d\n", os);
    }

    for(auto& os : this->grid.offsetys)
    {
        printf("OffsetY %4d\n", os);
    }
    printf("\n\n");
    
#endif

    // Create Gcells
    //this->grid.CreateGCs();

    //CreateTrackRtree();
    // Initialize Gcell Capacitance
    for(int l=0; l < numlayers; l++)
    {
        Layer* curL = &ckt->layers[l];
        this->grid.InitGcellCap(curL->id, curL->direction, curL->trackOffsets);
    }

    //exit(0);

}

void OABusRouter::Grid3D::InitGcellCap(int layer, int dir, vector<int> &offsets)
{

    int GCllx, GClly, GCurx, GCury;
    //int originX, originY, width, height; //, lower;
    int cap, tid, curl, idx;
    int x1, x2, y1, y2;
    bool adj;
    vector<SegmentValT> queries;
    vector<int> resources;
    SegRtree* trackrtree;
    BoxBG queryBox;
    SegmentBG tmpseg;
    Rtree* rtree;
    rtree = &rou->rtree;
    trackrtree = &rtree->track;
    //dense_hash_map<int,int> &trackNuml= rou->rtree.trackNuml;
    //dense_hash_map<int,int> &trackID = rou->rtree.trackID;
    curl = layer;



    // HashMap preffered direction
    this->direction[layer] = dir;

    for(int col=0; col < numCols; col++)
    {
        for(int row=0; row < numRows; row++)
        {

            cap = 0;
            queries.clear();
            resources.clear();

            GCllx = GetOffset_x(col);
            GClly = GetOffset_y(row);
            GCurx = (col == numCols-1) ? xoffset + width : GetOffset_x(col+1);// + GCELL_WIDTH, xoffset + width) ;
            GCury = (row == numRows-1) ? yoffset + height : GetOffset_y(row+1);// + GCELL_HEIGHT, yoffset + height);

            //printf("box (%d %d) (%d %d)\n", GCllx, GClly, GCurx, GCury);

            queryBox = BoxBG(PointBG(GCllx, GClly), PointBG(GCurx, GCury));
            trackrtree->query(
                    bgi::intersects(queryBox) && // && !bgi::overlaps(queryBox), 
                    bgi::satisfies([&, curl, queryBox, rtree](const SegmentValT& val){
                        float llx, lly, urx, ury;
                        int tarl;
                        int idx;
                        bool fallLL, fallUR, targetL;
                        idx = val.second;
                        SegmentBG curSeg = val.first;
                        tarl = rtree->layer(idx); //trackNuml[idx];
                        llx = bg::get<0,0>(curSeg);
                        lly = bg::get<0,1>(curSeg);
                        urx = bg::get<1,0>(curSeg);
                        ury = bg::get<1,1>(curSeg);
                        fallLL = (bg::within(PointBG(llx,lly),queryBox))?true:false;
                        fallUR = (bg::within(PointBG(urx,ury),queryBox))?true:false;
#ifdef DEBUG_GRID
                        if(fallLL || fallUR)
                        {
                            cout << "Box -> " << bg::dsv(queryBox) << endl;
                            cout << "LL  -> " << bg::dsv(PointBG(llx,lly)) << endl;
                            cout << "UR  -> " << bg::dsv(PointBG(urx,ury)) << endl;
                        }
#endif
                        
                        targetL = (curl == tarl)?true:false;
                        return (!fallLL && !fallUR && targetL);
                    }), back_inserter(queries));

            //cout << "Queries size : " << queries.size() << endl;
            cap = queries.size();
                /*
            for(int i=0; i < queries.size(); i++)
            {
             
                tmpseg = queries[i].first;
                idx = queries[i].second;
                curl = trackNuml[idx];
                tid = trackID[idx];
                adj = (layer == curl)? true:false;

                x1 = (int)(bg::get<0,0>(tmpseg) + 0.5);
                y1 = (int)(bg::get<0,1>(tmpseg) + 0.5);
                x2 = (int)(bg::get<1,0>(tmpseg) + 0.5);
                y2 = (int)(bg::get<1,1>(tmpseg) + 0.5);

                PointBG pt1(x1,y1);
                PointBG pt2(x2,y2);

                if(bg::within(pt1,queryBox) || bg::within(pt2,queryBox))
                {
                    continue;
                }

                if(adj)
                {
                    resources.push_back(tid);
                    cap++;
                }
                
            }
                */
            
            //sort(resources.begin(), resources.end());

            Gcell* curGC = &gcells[GetIndex(col,row,layer)];
            for(int i=0; i < cap; i++)
            {
                int idx = queries[i].second;
                tid = rtree->trackid(idx); //trackID[idx];
                curGC->resources.insert(tid);   
            }
            //sort(curGC->resources.begin(), curGC->resources.end());

            curGC->id = GetIndex(col,row,layer);
            curGC->x = col;
            curGC->y = row;
            curGC->l = layer;
            curGC->cap = cap;
            curGC->direction = dir;
            //curGC->resources = resources;
            
#ifdef DEBUG_GRID
            printf("g%d (%d %d %d) cap %d\n", curGC->id, curGC->x, curGC->y, curGC->l, curGC->cap);
#endif
        }
    }


}


