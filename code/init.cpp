#include "func.h"
#include "circuit.h"
#include <tuple>

using namespace OABusRouter;


void OABusRouter::Circuit::InitGcellParameters()
{
    int tmpmax = INT_MIN;
    int tmpmin = INT_MAX;
    int numBuses = this->buses.size();
    

    for(int i=0; i < numBuses; i++)
    {
        Bus* curB = &this->buses[i];
        tmpmax = max(curB->numBits, tmpmax);
        tmpmin = min(curB->numBits, tmpmin);
    }

    int minpitchV = INT_MAX;
    int maxpitchV = INT_MIN;
    int minpitchH = INT_MAX;
    int maxpitchH = INT_MIN;
    int numLayers = this->layers.size();

    for(int i=0; i < numLayers; i++)
    {
        Layer* curL = &this->layers[i];
        int wirepitch = abs(curL->trackOffsets[0] - curL->trackOffsets[1]);
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

    this->GCELL_WIDTH = minpitchH * tmpmax * 2;
    this->GCELL_HEIGHT = minpitchV * tmpmax * 2;

}


void OABusRouter::Circuit::Init()
{
    printf("Design Boundary (%d %d) (%d %d)\n", this->designBoundary.ll.x, this->designBoundary.ll.y,
            this->designBoundary.ur.x, this->designBoundary.ur.y);
    printf("# of layers : %d\n", this->layers.size());
    printf("# of tracks : %d\n", this->tracks.size());

    
    int numGcell_W = ceil(1.0*(this->designBoundary.ur.x - this->designBoundary.ll.x)/GCELL_WIDTH);
    int numGcell_H = ceil(1.0*(this->designBoundary.ur.y - this->designBoundary.ll.y)/GCELL_HEIGHT);
    int designOffsetX = this->designBoundary.ll.x;
    int designOffsetY = this->designBoundary.ll.y;

    cout << "Design Offset (" << designOffsetX << " " << designOffsetY << ")" << endl;


    // Initialize Gcell Offsets
    for(int w=0; w < numGcell_W; w++)
    {
        int offsetLX = w*GCELL_WIDTH + designOffsetX;
        int offsetUX = min(this->designBoundary.ur.x, (w+1)*GCELL_WIDTH + designOffsetX);
        this->gCellOffsetLX.push_back(offsetLX);
        this->gCellOffsetUX.push_back(offsetUX);
    }

    for(int h=0; h < numGcell_H; h++)
    {
        int offsetLY = h*GCELL_HEIGHT + designOffsetY;
        int offsetUY = min(this->designBoundary.ur.y, (h+1)*GCELL_HEIGHT + designOffsetY);
        this->gCellOffsetLY.push_back(offsetLY);
        this->gCellOffsetUY.push_back(offsetUY);
    }



    // Gcell construction
    for(int i=0; i < this->layers.size() ; i++){
        Layer* layer = &this->layers[i];
        //layer->print(false); 
        
        


        if(layer->is_vertical())
        {
            for(int w=0; w < numGcell_W; w++)
            {
                int x1 = w*GCELL_WIDTH + designOffsetX;
                int x2 = min(this->designBoundary.ur.x, (w+1)*GCELL_WIDTH + designOffsetX);
                
                int lowerBound = GetLowerBound(layer->trackOffsets, x1);
                int upperBound = GetUpperBound(layer->trackOffsets, x2);
                //((w+1)*GCELL_WIDTH));
                int cap = upperBound - lowerBound;
                for(int h=0; h < numGcell_H; h++)
                {
                    int y1 = h*GCELL_HEIGHT + designOffsetY;
                    int y2 = min(this->designBoundary.ur.y, (h+1)*GCELL_HEIGHT + designOffsetY);


                    Gcell gcell;
                    gcell.id = this->gCells.size();
                    gcell.direction = layer->direction;
                    gcell.cap = cap;
                    gcell.layer = layer->name;
                    gcell.boundary = Rect(Point(x1, y1), Point(x2, y2));

                    //gcell.boundary.print();
                    //gcell.boundary.ur.x = min(this->designBoundary.ur.x, gcell.boundary.ur.x);
                    //gcell.boundary.ur.y = min(this->designBoundary.ur.y, gcell.boundary.ur.y);
                    
                    gcell.trackOffsets.insert(
                            gcell.trackOffsets.end(), 
                            (layer->trackOffsets.begin() + lowerBound), 
                            (layer->trackOffsets.begin() + upperBound)
                            );

                    //cout << "Bound (" << lowerBound << " " << upperBound << ") # tracks " << layer->trackOffsets.size() <<  endl;
                    /*
                    gcell.tracks.insert(
                            gcell.tracks.end(), 
                            (layer->tracks.begin() + lowerBound), 
                            (layer->tracks.begin() + upperBound)
                            );
                    */

                    this->gCells.push_back(gcell);
                    tuple<string,int,int> keyValue = make_tuple(layer->name,x1,y1);
                    this->gCellHashMap[GetHashKey(keyValue)] = gcell.id;
                
                }
            }
        }else{

            for(int h=0; h < numGcell_H; h++)
            {

                int y1 = h*GCELL_HEIGHT + designOffsetY;
                int y2 = min(this->designBoundary.ur.y, (h+1)*GCELL_HEIGHT + designOffsetY);
                
                int lowerBound = GetLowerBound(layer->trackOffsets, y1);
                int upperBound = GetUpperBound(layer->trackOffsets, y2);
                int cap = upperBound - lowerBound;

                for(int w=0; w < numGcell_W; w++)
                {
                    int x1 = w*GCELL_WIDTH + designOffsetX;
                    int x2 = min(this->designBoundary.ur.x, (w+1)*GCELL_WIDTH + designOffsetX);
                    
                    Gcell gcell;
                    gcell.id = this->gCells.size();
                    gcell.direction = layer->direction;
                    gcell.cap = cap;
                    gcell.layer = layer->name;
                    gcell.boundary = Rect(Point(x1,y1), Point(x2,y2));
                   
                    gcell.trackOffsets.insert(
                            gcell.trackOffsets.end(), 
                            (layer->trackOffsets.begin() + lowerBound), 
                            (layer->trackOffsets.begin() + upperBound)
                            );
                    this->gCells.push_back(gcell);
                    tuple<string,int,int> keyValue = make_tuple(layer->name,x1,y1);
                    this->gCellHashMap[GetHashKey(keyValue)] = gcell.id;
                }
            }
        }
    
    }
}


void OABusRouter::Circuit::InitRoutingDirection()
{
    int numBuses = this->buses.size();
    int numLayers = this->layers.size();
    int numBits, numPinShapes;

    enum RouDir
    {
        Downward,
        Upward,
        Leftward,
        Rightward
    };

    

    for(int i=0; i < numBuses; i++)
    {   
        Bus* curBus = &this->buses[i];
        numBits = curBus->numBits;
        numPinShapes = curBus->numPinShapes;
        
        int busLLX = curBus->llx;
        int busLLY = curBus->lly;
        int busURX = curBus->urx;
        int busURY = curBus->ury;

        // Sequence array 
        // bottom to top
        // left to right
        int seq[numBits][numPinShapes];
        int tidx_Xs[numBits][numPinShapes];
        int tidx_Ys[numBits][numPinShapes];
        int lidx[numPinShapes];
        int dir[numPinShapes];
        int placeDir[numPinShapes];


        for(int p=0; p < numPinShapes; p++)
        {

            vector<pair<int,int>> indices(numBits, {0,0});
            bool verAligned;

            // S
            for(int b=0; b < numBits; b++)
            {
                Bit* curBit = &this->bits[curBus->bits[b]];
                Pin* curPin = &this->pins[curBit->pins[p]];
                Layer* curLayer = &this->layers[this->layerHashMap[curPin->layer]];
            
                int llx = curPin->boundary.ll.x;
                int lly = curPin->boundary.ll.y;
                int urx = curPin->boundary.ur.x;
                int ury = curPin->boundary.ur.y;
                int direction = curLayer->direction;
                lidx[p] = curLayer->id;

                Layer* verL;
                Layer* horL;

                // Base
                if(b == 0)
                {
                    Bit* nextBit = &this->bits[curBus->bits[1]];
                    Pin* nextPin = &this->pins[nextBit->pins[p]];
                    int llx2 = nextPin->boundary.ll.x;
                    int lly2 = nextPin->boundary.ll.y;
                    int urx2 = nextPin->boundary.ur.x;
                    int ury2 = nextPin->boundary.ur.y;
                    

                    if((llx == llx2) && (urx == urx2))
                    {
                        verAligned = false;
                    }
                    else if((lly == lly2) && (ury == ury2))
                    {
                        verAligned = true;
                    }
                    else
                    {
                        printf("invalid aligned pins...\n");
                        exit(0);
                    }
                    
                    
                    Layer* adjLayer;
                    if(curLayer->id == 0)
                    {
                        adjLayer = &this->layers[curLayer->id+1];
                    }
                    else if(curLayer->id == numLayers-1)
                    {
                        adjLayer = &this->layers[curLayer->id-1];
                    }
                    else
                    {
                        adjLayer = &this->layers[curLayer->id+1];
                    }
                    
                    // Require HV or VH
                    if(curLayer->direction == adjLayer->direction)
                    {
                        printf("wrong layer sequence...\n");
                        exit(0);
                    }
                    else
                    {
                        if(curLayer->is_vertical())
                        {
                            verL = curLayer;
                            horL = adjLayer;
                        }
                        else
                        {
                            verL = adjLayer;
                            horL = curLayer;
                        }
                        

                    }

                    if(verAligned)
                    {
                        int abs1 = abs(busURY - ury);
                        int abs2 = abs(busLLY - lly);
                        if(abs2 > abs1)
                        {
                            dir[p] = RouDir::Downward;
                            tidx_Ys[0][p] = GetLowerBound(horL->trackOffsets, lly) - 1;
                            
                        }
                        else
                        {
                            dir[p] = RouDir::Upward;
                            tidx_Ys[0][p] = GetLowerBound(horL->trackOffsets, ury) + 1;
                        }
                    }
                    else
                    {
                        int abs1 = abs(busURX - urx);
                        int abs2 = abs(busLLX - llx);
                        if(abs2 > abs1)
                        {
                            dir[p] = RouDir::Leftward;
                            tidx_Xs[0][p] = GetLowerBound(verL->trackOffsets, llx) - 1;
                        
                        }
                        else
                        {
                            dir[p] = RouDir::Rightward;
                            tidx_Xs[0][p] = GetLowerBound(verL->trackOffsets, urx) + 1;
                        }
                    }
                }
                //////////////


                if(verAligned)
                {
                    tidx_Xs[b][p] = GetLowerBound(verL->trackOffsets,llx); 
                    
                    if(dir[p] == RouDir::Downward)
                    {
                        tidx_Ys[b][p] = tidx_Ys[0][p] - b;
                    }
                    else
                    {
                        tidx_Ys[b][p] = tidx_Ys[0][p] + b;
                    }
                    indices.push_back({b, tidx_Xs[b][p]});
                }
                else
                {
                    tidx_Ys[b][p] = GetLowerBound(horL->trackOffsets,lly);
                    if(dir[p] == RouDir::Leftward)
                    {
                        tidx_Xs[b][p] = tidx_Xs[0][p] - b;
                    }
                    else
                    {
                        tidx_Xs[b][p] = tidx_Xs[0][p] + b;
                    }
                    indices.push_back({b, tidx_Ys[b][p]});
                }
            }
    


            sort(indices.begin(), indices.end(), [](const pair<int,int>& left, const pair<int,int>& right){
                    return left.second < right.second;
                    });
            

            for(int b=0; b < numBits; b++)
            {
                pair<int,int> ind = indices[b];
                seq[ind.first][p] = b;
            }
        }

        for(int p=0; p < numPinShapes; p++)
        {
            Layer* curL = &this->layers[lidx[p]];
            if(curL->is_vertical())
            {
                printf("(%2d) Pin Shape %s VERTICAL \n", p, curL->name.c_str());

            }
            else
            {
                printf("(%2d) Pin Shape %s HORIZONTAL \n", p, curL->name.c_str());
            }
            for(int b=0; b < numBits; b++)
            {
                Bit* curBit = &this->bits[curBus->bits[b]];
                Pin* curPin = &this->pins[curBit->pins[p]];
                int llx = curPin->boundary.ll.x;
                int lly = curPin->boundary.ll.y;
                int urx = curPin->boundary.ur.x;
                int ury = curPin->boundary.ur.y;


                printf("    (%2d) Track Index (%d %d) Pin (%d %d) (%d %d) Seq %d\n", b, tidx_Xs[b][p], tidx_Ys[b][p], 
                        llx, lly, urx, ury, seq[b][p]);
            }
            printf("\n\n");
        }
    }



}



void OABusRouter::Circuit::RoutingPoint()
{

    int ublidx = this->layers.size();
    int lblidx = 0;
    int numBuses = this->buses.size();
    for(int i=0; i < numBuses; i++)
    {
        Bus* curBus = &this->buses[i];
        int busLLX = curBus->llx;
        int busLLY = curBus->lly;
        int busURX = curBus->urx;
        int busURY = curBus->ury;
        
        int numPinShapes = curBus->numPinShapes;
        int numBits = curBus->numBits;

        printf("%s\n", curBus->name.c_str());
        for(int j=0; j< numBits; j++)
        {
            Bit* curBit = &this->bits[curBus->bits[j]];
            printf("    (%d)%s\n", j, curBit->name.c_str());
            for(int k=0; k < numPinShapes; k++)
            {
                Pin* curPin = &this->pins[this->bits[curBus->bits[j]].pins[k]];
                Layer* curLayer = &this->layers[this->layerHashMap[curPin->layer]];

                if(curLayer->is_vertical())
                {
                    printf("        (%d %d) (%d %d) %s VERTICAL\n", 
                            curPin->boundary.ll.x, curPin->boundary.ll.y,
                            curPin->boundary.ur.x, curPin->boundary.ur.y,
                            curPin->layer.c_str());
                }else{
                    printf("        (%d %d) (%d %d) %s HORIZONTAL\n", 
                            curPin->boundary.ll.x, curPin->boundary.ll.y,
                            curPin->boundary.ur.x, curPin->boundary.ur.y,
                            curPin->layer.c_str());

                }
            }
        }

        printf("\n\n");
    }

}
