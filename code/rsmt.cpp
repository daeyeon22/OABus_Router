#include "circuit.h"
#include "route.h"
#include "func.h"
#include "./flute/flute.h"
#include "./flute/global.h"

namespace br = OABusRouter;
int orderx(const void *a, const void *b);
int ordery(const void *a, const void *b);
void FluteNormal(int d, DTYPE x[], DTYPE y[], int acc, float coeffV, Tree *t);


void OABusRouter::Router::GenBackbone()
{
    enum Dir
    {
        Downward,
        Upward,
        Leftward,
        Rightward
    };


    int numBuses = ckt->buses.size();

    // read lookup table
    readLUT();


    // make a initial topology for each bus
    // not obstacle-aware
    for(int i=0; i < numBuses; i++)
    {
        // Datas
        Bus* curBus;
        Bit* curBit;
        Pin* curPin;
        Layer* curLayer;
        MultiPin* curMultipin;

        int numMultipins;
        //int numPinShapes;
        int *x, *y, *l, *ids;
        int treellx, treelly;
        int treeurx, treeury;
        Tree tree;
        StTree sttree;

        // make a topology only first bit
        curBus = &ckt->buses[i];
        curBit = &ckt->bits[curBus->bits[0]];
        //numPinShapes = curBus->numPinShapes;
        numMultipins = curBus->multipins.size();
        treellx = curBus->llx;
        treeurx = curBus->urx;
        treelly = curBus->lly;
        treeury = curBus->ury;

        // mapped index of gcells
        x = new int[numMultipins];
        y = new int[numMultipins];
        l = new int[numMultipins]; 
        ids = new int[numMultipins];

        for(int p=0; p < numMultipins;  p++)
        {
            curMultipin = &ckt->multipins[curBus->multipins[p]];
            curPin = &ckt->pins[curMultipin->pins[0]];
            curLayer = &ckt->layers[curMultipin->l];
            int pinllx = curPin->llx;
            int pinlly = curPin->lly;
            int pinurx = curPin->urx;
            int pinury = curPin->ury;

            int col = grid.GetColum(pinllx);
            int row = grid.GetRow(pinlly);
            
            
            ids[p] = curMultipin->id;
            x[p] = col;
            y[p] = row;
            l[p] = curLayer->id;
        }

        rsmt.CreateTree(curBus->id, numMultipins, ids, x, y, l, ACCURACY, 1.2);

        
        delete x;
        delete y;
        delete l;
    }
}


void OABusRouter::RSMT::CreateTree(int id, int d, int mps[], DTYPE x[], DTYPE y[], DTYPE l[], int acc, float coeffV)
{

    Tree tree;
    StTree sttree;
    int degree = d;
    int numNodes = 2*d - 2;
    int numEdges = 2*d - 3;
    int nbrcnt[numNodes] = {0};
    int edgecnt = 0;
    int treeid = trees.size();

    // Flute
    FluteNormal(d, x, y, acc, coeffV, &tree);

    // make StTree
    sttree.id = treeid;
    sttree.deg = degree;
    sttree.numNodes = numNodes;
    sttree.numEdges = numEdges;
    sttree.nodes = vector<br::TreeNode>(numNodes, br::TreeNode());
    sttree.edges = vector<br::TreeEdge>(numEdges, br::TreeEdge());
    sttree.length = 0;

    for(int i=0; i< numNodes; i++)
    {   

        int x1 = tree.branch[i].x;
        int y1 = tree.branch[i].y;
        int n = tree.branch[i].n;
        int x2 = tree.branch[n].x;
        int y2 = tree.branch[n].y;

        br::TreeNode* n1 = &sttree.nodes[i];
        n1->id = i;
        n1->x = x1;
        n1->y = y1;
        n1->n = n;
        
        // layer
        if(i < d)
        {
            n1->l = l[i];
            n1->steiner = false;
            sttree.node2multipin[i] = mps[i];       
        }
        else
        {
            n1->l = INT_MAX;
            n1->steiner = true;
            sttree.node2multipin[i] = INT_MAX;
        }



        if(n != i) // Not root
        {
            br::TreeNode* n2 = &sttree.nodes[n];
            br::TreeEdge* e = &sttree.edges[edgecnt];
            e->n1 = i;
            e->n2 = n;
            e->length = abs(x2-x1) + abs(y2-y1);
            sttree.length += e->length;

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

    //int treeid = trees.size();
    treeID[id] = treeid;
    busID[treeid] = id;
    trees.push_back(sttree);
}


int OABusRouter::RSMT::GetBusID(int treeid)
{
    return busID[treeid];
}

int OABusRouter::RSMT::GetTreeID(int busid)
{
    return treeID[busid];
}


//////////////// FLUTE //////////////////

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


int ordery(const void *a, const void *b)
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
