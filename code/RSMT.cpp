#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "circuit.h"
#include "flute.h"
#include "global.h"


namespace br = OABusRouter;


struct pnt
{
        DTYPE x, y;
            int o;
};

int orderx(const void *a, const void *b)
{
    struct pnt *pa, *pb;

    pa = *(struct pnt**)a;
    pb = *(struct pnt**)b;

    if (pa->x < pb->x) return -1;
    if (pa->x > pb->x) return 1;
    return 0;
    //return Y(*(struct Segment*)a.x1-*(struct Segment*)b.x1);
}


static int ordery(const void *a, const void *b)
{
    struct pnt *pa, *pb;

    pa = *(struct pnt**)a;
    pb = *(struct pnt**)b;

    if (pa->y < pb->y) return -1;
    if (pa->y > pb->y) return 1;
    return 0;
}




void FluteNormal(int d, DTYPE x[], DTYPE y[], int acc, float coeffV, Tree *t)
{
    DTYPE *xs, *ys, minval, x_max, x_min, x_mid, y_max, y_min, y_mid, *tmp_xs, *tmp_ys;
    int *s;
    int i, j, k, minidx;
    struct pnt *pt, **ptp, *tmpp;

	if (d==2) {
        t->deg = 2;
        t->length = ADIFF(x[0], x[1]) + ADIFF(y[0], y[1]);
        t->branch = (Branch *) malloc(2*sizeof(Branch));
        t->branch[0].x = x[0];
        t->branch[0].y = y[0];
        t->branch[0].n = 1;
        t->branch[1].x = x[1];
        t->branch[1].y = y[1];
        t->branch[1].n = 1;
    } else if (d==3) {
        t->deg = 3;
        if(x[0]<x[1])
        {
            if(x[0]<x[2])
            {
                x_min = x[0];
                x_mid = min(x[1], x[2]);
                x_max = max(x[1], x[2]);
            }
            else
            {
                x_min = x[2];
                x_mid = x[0];
                x_max = x[1];
            }
        }
        else
        {
            if(x[0]<x[2])
            {
                x_min = x[1];
                x_mid = x[0];
                x_max = x[2];
            }
            else
            {
                x_min = min(x[1], x[2]);
                x_mid = max(x[1], x[2]);
        
            }
        }                                                                                         
        if(y[0]<y[1])
        {
            if(y[0]<y[2])
            {
                y_min = y[0];
                y_mid = min(y[1], y[2]);
                y_max = max(y[1], y[2]);
            }
            else
            {
                y_min = y[2];
                y_mid = y[0];
                y_max = y[1];
            }
        }
        else
        {
            if(y[0]<y[2])
            {
                y_min = y[1];
                y_mid = y[0];
                y_max = y[2];
            }
            else
            {
                y_min = min(y[1], y[2]);
                y_mid = max(y[1], y[2]);
                y_max = y[0];
            }
        }
           
        t->length = ADIFF(x_max, x_min) + ADIFF(y_max, y_min);
        t->branch = (Branch *) malloc(4*sizeof(Branch));
        t->branch[0].x = x[0];
        t->branch[0].y = y[0];
        t->branch[0].n = 3;
        t->branch[1].x = x[1];
        t->branch[1].y = y[1];
        t->branch[1].n = 3;
        t->branch[2].x = x[2];
        t->branch[2].y = y[2];
        t->branch[2].n = 3;
        t->branch[3].x = x_mid;
        t->branch[3].y = y_mid;
        t->branch[3].n = 3;
    }
    else {
        xs = (DTYPE *)malloc(sizeof(DTYPE)*(d));
        ys = (DTYPE *)malloc(sizeof(DTYPE)*(d));

		tmp_xs = (DTYPE *)malloc(sizeof(DTYPE)*(d));
        tmp_ys = (DTYPE *)malloc(sizeof(DTYPE)*(d));

        s = (int *)malloc(sizeof(int)*(d));
        pt = (struct pnt *)malloc(sizeof(struct pnt)*(d+1));
        ptp = (struct pnt **)malloc(sizeof(struct pnt*)*(d+1));

        for (i=0; i<d; i++) {
            pt[i].x = x[i];
            pt[i].y = y[i];
            ptp[i] = &pt[i];
        }
        //printf("OK here\n");
        // sort x
        
      
	if (d<1000) {
        for (i=0; i<d-1; i++) {
            minval = ptp[i]->x;
            minidx = i;
            for (j=i+1; j<d; j++) {
                if (minval > ptp[j]->x) {
                    minval = ptp[j]->x;
                    minidx = j;
                }
            }
            tmpp = ptp[i];
            ptp[i] = ptp[minidx];
            ptp[minidx] = tmpp;
        }
	} else {
		qsort(ptp, d, sizeof(struct point *), orderx);
	}
        
#if REMOVE_DUPLICATE_PIN==1
        ptp[d] = &pt[d];
        ptp[d]->x = ptp[d]->y = -999999;
        j = 0;
        for (i=0; i<d; i++) {
            for (k=i+1; ptp[k]->x == ptp[i]->x; k++)
                if (ptp[k]->y == ptp[i]->y)  // pins k and i are the same
                    break;
            if (ptp[k]->x != ptp[i]->x)
                ptp[j++] = ptp[i];
        }
        d = j;
#endif
        
        for (i=0; i<d; i++) {
            xs[i] = ptp[i]->x;
            ptp[i]->o = i;
        }
        
        // sort y to find s[]
		if (d<1000) {
				for (i=0; i<d-1; i++) {
					minval = ptp[i]->y;
					minidx = i;
					for (j=i+1; j<d; j++) {
						if (minval > ptp[j]->y) {
							minval = ptp[j]->y;
							minidx = j;
						}
					}
					ys[i] = ptp[minidx]->y;
					s[i] = ptp[minidx]->o;
					ptp[minidx] = ptp[i];
				}
				ys[d-1] = ptp[d-1]->y;
				s[d-1] = ptp[d-1]->o;
		} else {
				qsort(ptp, d, sizeof(struct point *), ordery);
				for (i=0; i<d; i++) {
					ys[i] = ptp[i]->y;
					s[i] = ptp[i]->o;
				}
		}
	        
		//gxs[netID] = (DTYPE*) malloc(d*sizeof(DTYPE));
		//gys[netID] = (DTYPE*) malloc(d*sizeof(DTYPE));
		//gs[netID]  = (DTYPE*) malloc(d*sizeof(DTYPE));


		for(i=0; i<d; i++)
		{
			//gxs[netID][i] = xs[i];
			//gys[netID][i] = ys[i];
			//gs[netID][i]  = s[i];

			tmp_xs[i]= xs[i]*100;
			tmp_ys[i] = ys[i]*((int)(100*coeffV));
		}


        *t = flutes(d, tmp_xs, tmp_ys, s, acc);

		for(i=0; i<2*d-2; i++)
        {
            t->branch[i].x = t->branch[i].x/100;
            t->branch[i].y = t->branch[i].y/((int)(100*coeffV));
        }

        free(xs);
        free(ys);
		free (tmp_xs);
		free(tmp_ys);
        free(s);
        free(pt);
        free(ptp);
    }




}

void CopyStTree(int bitID, Tree *rsmt, br::StTree *sttree)
{
    int numNodes = 2*rsmt->deg -2;
    int numEdges = 2*rsmt->deg -3;
    int nbrcnt[numNodes] = {0};
    for(int i=0; i < numNodes; i++){
        cout << nbrcnt[i] << " ";
    }
    cout << endl;
    
    int edgecnt =0;
 
    sttree->index = bitID;
    sttree->numNodes = numNodes;
    sttree->numEdges = numEdges;
    sttree->nodes = vector<br::TreeNode>(numNodes, br::TreeNode());
    sttree->edges = vector<br::TreeEdge>(numEdges, br::TreeEdge());
    sttree->length = 0;

    for(int i=0; i< numNodes; i++)
    {   
        int x1 = rsmt->branch[i].x;
        int y1 = rsmt->branch[i].y;
        int n = rsmt->branch[i].n;
        int x2 = rsmt->branch[n].x;
        int y2 = rsmt->branch[n].y;

        br::TreeNode* n1 = &sttree->nodes[i];
        n1->index = i;
        n1->x = x1;
        n1->y = y1;
        n1->parent = n;

        if(n != i) // Not root
        {
            br::TreeNode* n2 = &sttree->nodes[n];
            if(i < rsmt->deg)
            {
                n1->isPin = true;
                n1->isSteiner = false;
            }else{
                n1->isPin = false;
                n1->isSteiner = true;
            }



            br::TreeEdge* e = &sttree->edges[edgecnt];
            e->n1 = i;
            e->n2 = n;
            e->length = abs(x2-x1) + abs(y2-y1);
            sttree->length += e->length;

            n1->neighbor.push_back(n);
            n1->edge.push_back(edgecnt);
            n2->neighbor.push_back(i);
            n2->edge.push_back(edgecnt);

            nbrcnt[i]++;
            nbrcnt[n]++;
            edgecnt++;

            if(nbrcnt[i] > 3 || nbrcnt[n] > 3)
            {
                cout << nbrcnt[i] << " " << nbrcnt[n] << endl;
                printf("wrong\n");
                exit(0);
            }
        }
    }

}

void OABusRouter::Circuit::GenBackbone_v2()
{
    // Steiner Tree count
    int stcnt =0;
    readLUT();

    // Params
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

    

    // Index Align
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
        int seq[numBits][numPinShapes];         // Sequence
        int tidx_Xs[numBits][numPinShapes];     // Track index on vertical layer index
        int tidx_Ys[numBits][numPinShapes];     // Track index on horizontal layer index
        int horLidx[numPinShapes];              // Starting point horizontal layer index
        int verLidx[numPinShapes];              // Starting point vertical layer index
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
                        
                        verLidx[p] = verL->id;
                        horLidx[p] = horL->id;

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

        // Gen StTree
    
        for(int b=0; b< numBits; b++)
        {
            Bit* curBit = &this->bits[curBus->bits[b]];
            
            DTYPE x[MAXD], y[MAXD], z[MAXD];
            int deg=0;
            Tree rsmt;
            StTree sttree;
            for(int p=0; p < numPinShapes; p++)
            {
                x[p] = this->layers[verLidx[p]].trackOffsets[tidx_Xs[b][p]];
                y[p] = this->layers[horLidx[p]].trackOffsets[tidx_Ys[b][p]];
                deg++;
            }

            FluteNormal(deg, x, y, ACCURACY, 1.2, &rsmt);
            printtree(rsmt);
            
            CopyStTree(curBit->id, &rsmt, &sttree);
            
            
            this->stTreeHashMap[curBit->id] = stcnt++;
            this->stTrees.push_back(sttree);
        }
        //////////////////////////////////////////////////////////////////////
    
    }


       

}
void OABusRouter::Circuit::GenBackbone()
{


    int stcnt =0;
    readLUT();

    int numLayers = this->layers.size();

    for(int i=0; i < this->buses.size(); i++)
    {
        br::Bus* bus = &this->buses[i];

        for(int j=0; j < bus->bits.size(); j++)
        {
            DTYPE x[MAXD], y[MAXD], z[MAXD];
            int deg=0;
            Tree rsmt;
            StTree sttree;

            br::Bit* bit = &this->bits[bus->bits[j]];
            cout << "Bit (" << bit->name << ")" << endl;
            for(int k=0; k<bit->pins.size(); k++)
            {

                Pin* curPin = &this->pins[bit->pins[k]];
                Layer* curLayer = &this->layers[this->layerHashMap[curPin->layer]];
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
                
                
                int pinLLX = curPin->boundary.ll.x;
                int pinLLY = curPin->boundary.ll.y;
                int pinURX = curPin->boundary.ur.x;
                int pinURY = curPin->boundary.ur.y; 
                
                // Get aligned offset of track 
                // Only Upper or Right direction 
                if(curLayer->is_vertical())
                {
                    int xidx = GetLowerBound(curLayer->trackOffsets, pinLLX);
                    int yidx = GetLowerBound(adjLayer->trackOffsets, pinURY);
                    x[k] = curLayer->trackOffsets[xidx];
                    y[k] = adjLayer->trackOffsets[yidx];
                    z[k] = curLayer->id;
                }else{
                    int xidx = GetLowerBound(adjLayer->trackOffsets, pinURX);
                    int yidx = GetLowerBound(curLayer->trackOffsets, pinLLY);
                    x[k] = adjLayer->trackOffsets[xidx];
                    y[k] = curLayer->trackOffsets[yidx];
                    z[k] = curLayer->id;
                }
                //x[k] = this->pins[bit->pins[k]].boundary.center().x;
                //y[k] = this->pins[bit->pins[k]].boundary.center().y;
                deg++;
                printf("Coord (%d %d)\n", x[k], y[k]);
            
            }

            
            FluteNormal(deg, x, y, ACCURACY, 1.2, &rsmt);
            printtree(rsmt);
            
            CopyStTree(j, &rsmt, &sttree);
            
            int wireLength = 0;
            
            for(int idx=0; idx < sttree.numEdges; idx++)
            {
                //printf("# of edges %d\n", sttree.numEdges);
                TreeEdge* edge = &sttree.edges[idx];
                TreeNode* n1 = &sttree.nodes[edge->n1];
                TreeNode* n2 = &sttree.nodes[edge->n2];
                int x1 = n1->x;
                int x2 = n2->x;
                int y1 = n1->y;
                int y2 = n2->y;
                

                if(x1 > x2) swap(x1,x2);
                if(y1 > y2) swap(y1,y2);

                wireLength += (x2 - x1) + (y2 - y1);

                if(x1!=x2 && y1!=y2)
                {
                    br::Segment seg1(x1, y1, x2, y1, bit->id);
                    br::Segment seg2(x2, y1, x2, y2, bit->id);
                    this->segs.push_back(seg1);
                    this->segs.push_back(seg2);
                }else{
                    br::Segment seg(x1, y1, x2, y2, bit->id);
                    this->segs.push_back(seg);
                }
            }

            sttree.length = wireLength;
            this->stTreeHashMap[bit->id] = stcnt++;
            this->stTrees.push_back(sttree);
        }
        cout << endl << endl;
    }
    cout << "End Backbone" << endl;
}



void OABusRouter::Circuit::LayerAssignment()
{
   







}

