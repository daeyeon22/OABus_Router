#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "circuit.h"
#include "route.h"
#include "func.h"



// 3D mapping
void OABusRouter::Router::TopologyMapping3D()
{

    //RSMT* _rsmt = &this->rsmt;
    int numTrees = this->rsmt.trees.size();
    
    // Print all
    for(int i=0; i < numTrees; i++)
    {
        StTree* tree = this->rsmt[i];
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


