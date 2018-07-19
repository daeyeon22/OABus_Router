#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "circuit.h"
#include "route.h"
#include "func.h"



// ILP formulation
void OABusRouter::Router::CreateClips()
{
    int numClips, numLayers, numCols, numRows;
    int numTrees, numSegs;
    int x1, y1, x2, y2, l;
    bool isVertical;
    StTree* tree;
    Segment* curS;
    IntervalMapT* curiMap;
    IntervalT intv;
    IDSetT idset;

    numLayers = this->grid.numLayers;
    numCols = this->grid.numCols;
    numRows = this->grid.numRows;
    numClips = numLayers;
    numTrees = this->rsmt.trees.size();
    
    vector<Clip> clips(numClips, Clip());
    Clip* curClip;

    // Clip initialize
    for(int i=0; i < numClips; i++)
    {
        curClip = &clips[i];         
        isVertical= (this->grid.direction[i] == VERTICAL)? true:false;
        if(isVertical)
        {
            curClip->numPanel = numCols;
            curClip->curLayer = i;
            curClip->direction = VERTICAL;
            curClip->intervalMaps = vector<IntervalMapT>(curClip->numPanel);
        }
        else
        {
            curClip->numPanel = numRows;
            curClip->curLayer = i;
            curClip->direction = HORIZONTAL;
            curClip->intervalMaps = vector<IntervalMapT>(curClip->numPanel);
        }
    }


    for(int i=0; i < numTrees; i++)
    {
        tree = this->rsmt[i];
        numSegs = tree->segs.size();
        for(int j=0; j < numSegs; j++)
        {
            curS = &this->segs[tree->segs[j]];
            curClip = &clips[curS->l];
            isVertical = (curClip->direction == VERTICAL)? true: false;

            x1 = curS->x1;
            y1 = curS->y1;
            x2 = curS->x2;
            y2 = curS->y2;
            l = curS->l;
            if(isVertical)
            {
                curClip->intervalMaps[x1] += 
                    make_pair(IntervalT::closed(min(y1,y2), max(y1,y2)), IDSetT({curS->id}));

                //curiMap = &curClip->intervalMaps[x1];
                //intv = IntervalT::closed(min(y1,y2), max(y1,y2));
            }
            else
            {
                curClip->intervalMaps[x1] += 
                    make_pair(IntervalT::closed(min(x1,x2), max(x1,x2)), IDSetT({curS->id}));
                
                //curiMap = &curClip->intervalMaps[y1];
                //intv = IntervalT::closed(min(x1,x2), max(x1,x2));
            }
            //idset = IDSetT({curS->id});
            //*curiMap += make_pair(intv,idset);
        }
    }


}





// 3D mapping
void OABusRouter::Router::TopologyMapping3D()
{

    //RSMT* _rsmt = &this->rsmt;
    // Variables
    int numTrees, numEdges, numNodes, numLayers, numNeighbor;
    int l1, l2, ubl, lbl, avgl, bw;
    int x1, y1, x2, y2;
    int verl, horl, curl;
    int segx1, segx2, segy1, segy2, segl, segid, busid;
    bool st1, st2, verline, horline, via;
    
    TreeNode *n1, *n2, *curNode, *tarNode;
    TreeEdge *e;
    //Layer *verl, *horl, *curl;
    vector<int> verticals;
    vector<int> horizontals;


    numLayers = ckt->layers.size();
    numTrees = this->rsmt.trees.size();

    for(int i=0; i < numLayers; i++)
    {
        if(ckt->layers[i].is_vertical()){
            verticals.push_back(ckt->layers[i].id);
        }else{
            horizontals.push_back(ckt->layers[i].id);
        }
    }


    
    // Print all
    for(int i=0; i < numTrees; i++)
    {
        StTree* tree = this->rsmt[i];
        //
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
                exit(0);
            }else{
            }
        }

        
        //
        //tree->print();

        
        numEdges = tree->numEdges;

        for(int j=0; j < numEdges ; j++)
        {

            e = &tree->edges[j];
            n1 = &tree->nodes[e->n1];
            n2 = &tree->nodes[e->n2];


            //
            //
            verline = false;
            horline = false;
            via = false;
            
            x1 = n1->x;
            y1 = n1->y;
            l1 = n1->l;
            x2 = n2->x;
            y2 = n2->y;
            l2 = n2->l;

            if(x1 != x2) horline = true;
            if(y1 != y2) verline = true;

            auto lamb = [&, l1, l2](int left, int right){
                int ldist, rdist;
                ldist = abs(l1-left) + abs(l2-left);
                rdist = abs(l1-right) + abs(l2-right);
                return ldist < rdist;
            };


            if(horline)
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
                
                
                //printf("Create segment H (%2d %2d) (%2d %2d) m%d\n", segx1, segy1, segx2, segy2, segl);
                
                // Create segment
                segid = this->segs.size();
                Segment seg(segid,segx1, segy1, segx2, segy2, segl);
                this->segs.push_back(seg);
                tree->segs.push_back(segid);

                // Mapping the bitwith for each segment
                busid = this->rsmt.GetBusID(tree->id);
                bw = ckt->buses[busid].numBits;
                this->bitwidth[segid] = bw;
            
            }


            if(verline)
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
                
                //printf("Create segment V (%2d %2d) (%2d %2d) m%d\n", segx1, segy1, segx2, segy2, segl);
                segid = this->segs.size();
                Segment seg(segid,segx1, segy1, segx2, segy2, segl);
                this->segs.push_back(seg);
                tree->segs.push_back(segid);
            
                // Mapping the bitwith for each segment
                busid = this->rsmt.GetBusID(tree->id);
                bw = ckt->buses[busid].numBits;
                this->bitwidth[segid] = bw;
            
            }
            //l1 = &this->layers[n1->l];
            //l2 = &this->layers[n2->l];
            //printf("Edge (%d %d %d) -> (%d %d %d)\n", 
        }
   
        //printf("\n\n");
        tree->print();

    }


}

// Print StTree
void OABusRouter::StTree::print()
{

    int i;
    
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
            printf("(%d) V-Line (%2d %2d) (%2d %2d) m%d\n", i,curS->x1, curS->y1, curS->x2, curS->y2, curS->l);
        }else{
            printf("(%d) H-Line (%2d %2d) (%2d %2d) m%d\n", i,curS->x1, curS->y1, curS->x2, curS->y2, curS->l);
        }


    }
    printf("\n\n");


}


// member functions for Grid3D
int OABusRouter::Grid3D::GetIndex(int col, int row, int layer)
{
    return col + numCols*row + (numCols*numRows)*layer;
}

int OABusRouter::Grid3D::GetOffset_x(int col)
{
    return GCELL_WIDTH*col + xoffset;
}

int OABusRouter::Grid3D::GetOffset_y(int row)
{
    return GCELL_HEIGHT*row + yoffset;
}

int OABusRouter::Grid3D::GetColum(int xcrd)
{
    return GetLowerBound(offsetxs, xcrd);
}

int OABusRouter::Grid3D::GetRow(int ycrd)
{
    return GetLowerBound(offsetys, ycrd);
}

int OABusRouter::Grid3D::Capacity(int col, int row, int layer)
{
    return gcells[GetIndex(col, row, layer)].cap;
}

// Initialize Grid3D
void OABusRouter::Router::InitGrid3D()
{

    int tmpmax = INT_MIN;
    int tmpmin = INT_MAX;
    int minpitchV = INT_MAX;
    int maxpitchV = INT_MIN;
    int minpitchH = INT_MAX;
    int maxpitchH = INT_MIN;
    int numBuses = ckt->buses.size();
    int numLayers = ckt->layers.size();
    int dir[numLayers];
    // Get maximum, minimum bitwidth
    for(int i=0; i < numBuses; i++)
    {
        Bus* curB = &ckt->buses[i];
        tmpmax = max(curB->numBits, tmpmax);
        tmpmin = min(curB->numBits, tmpmin);
    }


    // Get maximum, minimum wire pitch
    for(int i=0; i < numLayers; i++)
    {
        Layer* curL = &ckt->layers[i];
        cout << "#tracks : " << curL->trackOffsets.size() << endl;
        int wirepitch = abs(curL->trackOffsets[0] - curL->trackOffsets[1]);
        dir[i] = curL->direction;
        if(curL->is_vertical())
        {
            maxpitchH = max(wirepitch, maxpitchH);
            minpitchH = min(wirepitch, minpitchH);   
        }
        else
        {
            maxpitchV = max(wirepitch, maxpitchV);
            minpitchV = min(wirepitch, minpitchV);
        }
    }

    int GCELL_WIDTH = minpitchH * tmpmax * 2;
    int GCELL_HEIGHT = minpitchV * tmpmax * 2;
    int offset_x = ckt->originX;
    int offset_y = ckt->originY;
    int sizeH = ckt->height;
    int sizeV = ckt->width;
    int numCols = ceil(1.0*sizeH/GCELL_WIDTH);
    int numRows = ceil(1.0*sizeV/GCELL_HEIGHT);


    this->grid = 
        Grid3D(numCols, 
            numRows, 
            numLayers, 
            GCELL_WIDTH, 
            GCELL_HEIGHT, 
            offset_x, 
            offset_y,
            sizeH,
            sizeV);

    // Create Gcells
    //this->grid.CreateGCs();

    // Initialize Gcell Capacitance
    for(int l=0; l < numLayers; l++)
    {
        Layer* curL = &ckt->layers[l];
        this->grid.InitGcellCap(curL->id, curL->direction, curL->trackOffsets);
    }
}

void OABusRouter::Grid3D::InitGcellCap(int layer, int dir, vector<int> &offsets)
{
    // HashMap preffered direction
    this->direction[layer] = dir;

    for(int col=0; col < numCols; col++)
    {
        for(int row=0; row < numRows; row++)
        {
            int GCllx, GClly, GCurx, GCury;
            int lowerIndex, upperIndex;
            int cap;
            GCllx = GetOffset_x(col);
            GClly = GetOffset_y(row);
            GCurx = GetOffset_x(col) + GCELL_WIDTH;
            GCury = GetOffset_y(row) + GCELL_HEIGHT;

            if(dir == VERTICAL)
            {
                lowerIndex = GetLowerBound(offsets, GCllx);
                upperIndex = GetUpperBound(offsets, GCurx);
            }
            else
            {
                lowerIndex = GetLowerBound(offsets, GClly);
                upperIndex = GetUpperBound(offsets, GCury);
            }
            
            // Edge capacitance
            cap = upperIndex - lowerIndex;
            Gcell* curGC = &gcells[GetIndex(col,row,layer)];
            curGC->id = GetIndex(col,row,layer);
            curGC->x = col;
            curGC->y = row;
            curGC->l = layer;
            curGC->cap = cap;
        }
    }
}


