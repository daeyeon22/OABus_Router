#include "circuit.h"
#include "route.h"
#include "typedef.h"
#include "func.h"
#include <tuple>

//#define DEBUG_INIT

using namespace OABusRouter;
using OABusRouter::intersection;

void OABusRouter::Circuit::initialize()
{
    int i, j, k, l, align;
    int numpins, numbits, numbuses, numlayers;
    Bus* curbus;
    Bit* curbit;
    Pin* curpin;
    numbuses = buses.size();
    numlayers = layers.size();
    numpins = pins.size();

    // spacing
    for(i=0; i < numlayers; i++)
        rou->spacing[i] = layers[i].spacing;
    
    // mapping
    for(i=0; i < numpins ; i++)
    {
        curpin = &pins[i];
        curbit = &bits[bitHashMap[curpin->bitName]];
        curbus = &buses[busHashMap[curbit->busName]];
        rou->pin2bit[curpin->id] = curbit->id;
        rou->pin2bus[curpin->id] = curbus->id;
        //bitHashMap[curpin->bitName];
    }

    for(i=0; i < numbuses; i++)
    {
        curbus = &buses[i];
        numpins = curbus->numPinShapes;
        numbits = curbus->numBits;
        vector<vector<int>> unused_pins(numbits);

        for(j=0; j < numbits; j++)
        {
            // added
            rou->bit2bus[curbus->bits[j]] = curbus->id;

            unused_pins[j] = bits[curbus->bits[j]].pins;
        }

        for(j=0; j < numpins; j++)
        {
            MultiPin mp;
            mp.id = multipins.size();
            mp.busid = curbus->id;

            for(k=0; k < numbits; k++)
            {
                curbit = &bits[curbus->bits[k]];
                curpin = &pins[curbit->pins[j]];
                //
                //printf("p%d -> bitid %d\n", curpin->id, curbit->id);
                //rou->pin2bit[curpin->id] = curbit->id;
                if(k==0)
                {
                    mp.l = curpin->l;
                    mp.llx = curpin->llx;
                    mp.lly = curpin->lly;
                    mp.urx = curpin->urx;
                    mp.ury = curpin->ury;
                    mp.pins.push_back(curpin->id);
                }
                else
                {
                    vector<int> &sorted = unused_pins[k];
                    box mpbb(pt(mp.llx, mp.lly), pt(mp.urx, mp.ury));
                    l = mp.l;
                    if(sorted.size() > 1)
                    {
                        sort(sorted.begin(), sorted.end(), [&,this, mpbb,l](int left, int right){
                                Pin* p1 = &this->pins[left];
                                Pin* p2 = &this->pins[right];
                                float dist1 = (p1->l != l) ? FLT_MAX : bg::distance(mpbb, box(pt(p1->llx,p1->lly), pt(p1->urx, p1->ury)));
                                float dist2 = (p2->l != l) ? FLT_MAX : bg::distance(mpbb, box(pt(p2->llx,p2->lly), pt(p2->urx, p2->ury))); 
                                return dist1 < dist2;
                                });
                    }
                    curpin = &pins[sorted[0]];
                    mp.llx = min(curpin->llx, mp.llx); 
                    mp.lly = min(curpin->lly, mp.lly); 
                    mp.urx = max(curpin->urx, mp.urx); 
                    mp.ury = max(curpin->ury, mp.ury); 
                    mp.pins.push_back(sorted[0]);
                    sorted.erase(sorted.begin());
                }
            }
            
            if(abs(mp.urx - mp.llx) > abs(mp.ury - mp.lly))
                mp.align = HORIZONTAL;
            else
                mp.align = VERTICAL;

            if(mp.align == layers[mp.l].direction)
                mp.needVia = true;

            /*
            align = mp.align;
            sort(mp.pins.begin(), mp.pins.end(), [&,this,align](int left, int right){
                if(align == VERTICAL)
                    return this->pins[left].lly < this->pins[right].lly;
                else

                    });
            */

            curbus->multipins.push_back(mp.id);
            multipins.push_back(mp);
            //
            rou->multipin2llx[mp.id] = mp.llx;
            rou->multipin2lly[mp.id] = mp.lly;
            rou->multipin2urx[mp.id] = mp.urx;
            rou->multipin2ury[mp.id] = mp.ury;
        }
    }

    for(i=0; i < multipins.size(); i++)
    {
        MultiPin* mp = &multipins[i];
        for(j=0; j < mp->pins.size(); j++)
            rou->pin2align[mp->pins[j]] = mp->align;
    }


    rou->DEPTH_COST = (height + width)*5/100;
    rou->VIA_COST = (height + width)*10/1000;
    rou->SPACING_VIOLATION = (height + width)*50/100;
    rou->NOTCOMPACT = (height + width)/5;
}


void OABusRouter::Router::construct_rtree()
{
    
    int numTracks, numLayers, trackid;

    numTracks = ckt->tracks.size();
    numLayers = ckt->layers.size();
    vector<PointRtree> ptrtree(numLayers);

    rtree_t.trees = vector<SegRtree>(numLayers);
    rtree_t.nodes = vector<RtreeNode>(numTracks);

    #pragma omp parallel for num_threads(NUM_THREADS)
    for(int i=0; i < numTracks; i++)
    {
        Track* curtrack = &ckt->tracks[i];
        RtreeNode* n1 = rtree_t.get_node(i);
        n1->id = i;
        n1->offset = curtrack->offset;
        n1->x1 = curtrack->llx;
        n1->y1 = curtrack->lly;
        n1->x2 = curtrack->urx;
        n1->y2 = curtrack->ury;
        n1->l = curtrack->l;
        n1->vertical = is_vertical(curtrack->l);
        n1->width = curtrack->width;

        seg s1(pt(n1->x1, n1->y1), pt(n1->x2, n1->y2));
        pt p1((n1->x1 + n1->x2)/2, (n1->y1 + n1->y2)/2);

        #pragma omp critical(GLOBAL)
        {
            rtree_t[n1->l]->insert({s1, i});
            ptrtree[n1->l].insert({p1, i});
        }
    }

    #pragma omp parallel for num_threads(NUM_THREADS)
    for(int i=0; i < numTracks; i++)
    {
        RtreeNode* n1 = rtree_t.get_node(i);
        seg s1(pt(n1->x1, n1->y1), pt(n1->x2, n1->y2));

        vector<pair<seg,int>> queries;
        for(int j=max(0, n1->l-1); j <= min(numLayers-1, n1->l+1); j++)
        {
            rtree_t[j]->query(bgi::intersects(s1), back_inserter(queries));
        }

        for(int j=0; j < queries.size(); j++)
        {
            int x, y;
            RtreeNode* n2 = rtree_t.get_node(queries[j].second);
            seg s2 = queries[j].first;
            
            if(n1->id == n2->id)
                continue;

            if(!intersection(s1, s2, x, y))
                continue;

            #pragma omp critical(GLOBAL)
            {
                n1->neighbor.push_back(n2->id);
                n1->intersection[n2->id] = {x,y};
                
                rtree_t.edges.insert( {min(n1->id, n2->id), max(n1->id, n2->id)} );
            }
        }
    }


    #pragma omp parallel for num_threads(NUM_THREADS) 
    for(int i=0; i < numTracks; i++)
    {
        int l, x1, y1, x2, y2;
        RtreeNode *n1, *n2;
        n1 = rtree_t.get_node(i);
        l = n1->l;
        x1 = (n1->x1 + n1->x2)/2;
        y1 = (n1->y1 + n1->y2)/2;


        for(PointRtree::const_query_iterator it = ptrtree[l].qbegin(bgi::nearest(pt(x1,y1), 10)); it != ptrtree[l].qend(); it++)
        {
            if(n1->upper != nullptr && n1->lower != nullptr)
                break;

            n2 = rtree_t.get_node(it->second);
            x2 = (n2->x1 + n2->x2)/2;
            y2 = (n2->y1 + n2->y2)/2;
            
            if(n1->id == n2->id)
                continue;

            if(n1->vertical)
            {
                if(x1 == x2)
                    continue;
                else if(x1 < x2)
                {
                    if(n1->upper == nullptr)
                        n1->upper = n2;
                }
                else if(x1 > x2)
                {
                    if(n1->lower == nullptr)
                        n1->lower = n2;
                }
                
            }
        }
    }
    
    rtree_o.pins = vector<BoxRtree>(numLayers);
    rtree_o.wires = vector<BoxRtree>(numLayers);
    rtree_o.obstacles = vector<BoxRtree>(numLayers);

    rtree_o.db[0] = ckt->originX;
    rtree_o.db[1] = ckt->originY;
    rtree_o.db[2] = ckt->originX + ckt->width;
    rtree_o.db[3] = ckt->originY + ckt->height;

    // for pins
    for(int i=0; i < ckt->pins.size(); i++)
    {
        Pin* curPin = &ckt->pins[i];
        box pinShape(pt(curPin->llx, curPin->lly), pt(curPin->urx, curPin->ury));
        int bitid = rou->pin2bit[i];
        rtree_o.pins[curPin->l].insert( {pinShape, bitid} );
    }

    // for obstacles
    for(int i=0; i < ckt->obstacles.size(); i++)
    {
        Obstacle* curObs = &ckt->obstacles[i];
        box obsShape(pt(curObs->llx, curObs->lly), pt(curObs->urx, curObs->ury));
        rtree_o.obstacles[curObs->l].insert( {obsShape, OBSTACLE} );
    }

    // for wires






#ifdef DEBUG_INIT
    for(int i=0; i < numTracks; i++)
    {
        printf("\n");
        RtreeNode* node = rtree_t.get_node(i);
        printf("Node %d (%d %d) (%d %d) M%d #neighbors %d\n", i, node->x1, node->y1, node->x2, node->y2, node->l, node->neighbor.size());
        if(node->lower != nullptr)
        {
            printf("Lower (%d %d) (%d %d) M%d\n", node->lower->x1, node->lower->y1, node->lower->x2, node->lower->y2, node->lower->l);
        }

        if(node->upper != nullptr)
        {
            printf("Upper (%d %d) (%d %d) M%d\n", node->upper->x1, node->upper->y1, node->upper->x2, node->upper->y2, node->upper->l);
        }
        
    }
#endif











}


