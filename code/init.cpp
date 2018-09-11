#include "func.h"
#include "circuit.h"
#include "route.h"
#include <tuple>

using namespace OABusRouter;



void OABusRouter::Circuit::initialize()
{
    typedef BoxBG box;
    typedef PointBG pt;
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
            unused_pins[j] = bits[curbus->bits[j]].pins;

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

    rou->initialize_rtree_new();
    //rou->initialize_grid3d();
}



void OABusRouter::Router::initialize()
{
    for(auto& mp : ckt->multipins)
    {
        /*
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
        */

        multipin2llx[mp.id] = mp.llx;
        multipin2lly[mp.id] = mp.lly;
        multipin2urx[mp.id] = mp.urx;
        multipin2ury[mp.id] = mp.ury;
    }

    //initialize_rtree();
    initialize_rtree_new();
    //initialize_grid3d();
}


void OABusRouter::Router::initialize_rtree_new()
{
    typedef SegmentBG seg;
    typedef PointBG pt;
    typedef BoxBG box;

    int numTracks, numObstacles, numLayers, numPins, tid, curl;
    int trackid, l; 
    int x1, y1, x2, y2, i, dir;
    int offset;
    int& elemindex = rtree_t.elemindex;
    bool isVertical;
    SegmentBG curS;
    Track* curtrack;
    Pin* curpin;
    Obstacle* curobs;
    Interval* interval;

    numTracks = ckt->tracks.size(); //interval.numtracks;
    numObstacles = ckt->obstacles.size();
    numLayers = ckt->layers.size();
    numPins = ckt->pins.size();
    elemindex=0;

    // Initialize Intervals.
    vector<SegRtree> trackrtree(numLayers);
  
    rtree_s = SegmentRtree(numLayers);
    rtree_t = TrackRtree(numLayers, numTracks);
    rtree_o = ObstacleRtree(numLayers);
    rtree_p = PinRtree(numLayers);
    rtree_p.elem2bus = pin2bus;
    rtree_p.elem2bit = pin2bit;

    rtree_o.db[0] = ckt->originX;
    rtree_o.db[1] = ckt->originY;
    rtree_o.db[2] = ckt->originX + ckt->width;
    rtree_o.db[3] = ckt->originY + ckt->height;


    // track interval
    for(i=0; i < numTracks; i++)
    {
        curtrack = &ckt->tracks[i];
        trackid = curtrack->id;
        interval = rtree_t.get_interval(trackid);
        interval->trackid = trackid;
        interval->offset = curtrack->offset;
        interval->l = curtrack->l;
        interval->vertical = ckt->is_vertical(curtrack->l);
        interval->width = curtrack->width;

        x1 = curtrack->llx;
        y1 = curtrack->lly;
        x2 = curtrack->urx;
        y2 = curtrack->ury;
        l = curtrack->l;

        seg s(pt(x1,y1), pt(x2,y2));
        rtree_t[l]->insert({s, trackid});
        
        interval->empty += (interval->vertical)?
            IntervalT::open(y1,y2) : IntervalT::open(x1,x2);
    }


    // obstacle
    for(i=0; i < numObstacles; i++)
    {
        curobs = &ckt->obstacles[i];
        x1 = curobs->llx;
        y1 = curobs->lly;
        x2 = curobs->urx;
        y2 = curobs->ury;
        l = curobs->l;

        box b(pt(x1,y1), pt(x2,y2));
        rtree_o[l]->insert({b, OBSTACLE});
    

        /* 
        vector<pair<seg,int>> queries;
        rtree_t[l]->query(bgi::intersects(b), back_inserter(queries));

        for(auto& it : queries)
        {
            trackid = it.second;
            interval = rtree_t.get_interval(trackid);
            interval->empty -= interval->vertical ?
                IntervalT::open(y1,y2) : IntervalT::open(x1,x2);
        }
        */
    }

    // Pin
    for(i=0; i < numPins; i++)
    {
        curpin = &ckt->pins[i];
        x1 = curpin->llx;
        y1 = curpin->lly;
        x2 = curpin->urx;
        y2 = curpin->ury;
        l = curpin->l;

        box b(pt(x1,y1), pt(x2,y2));
        rtree_o[l]->insert({b, pin2bit[curpin->id]});
        //
        rtree_p.elems.push_back(b);
        rtree_p[l]->insert({b, curpin->id});

        vector<pair<seg, int>> queries;
        
        rtree_t[l]->query(bgi::intersects(b), back_inserter(queries));
        
        for(auto& it : queries)
        {
            trackid = it.second;
            interval = rtree_t.get_interval(trackid);
            interval->empty -= interval->vertical ?
                IntervalT::open(y1,y2) : IntervalT::open(x1,x2);           
        }
    }


    // remove all
    for(i=0; i < numLayers; i++)
        rtree_t[i]->clear();

    // update
    for(i=0; i < numTracks; i++)
    {
        interval = rtree_t.get_interval(i);
        IntervalSetT::iterator it = interval->empty.begin();
        while(it != interval->empty.end())
        {
            x1 = interval->vertical ? interval->offset : it->lower();
            x2 = interval->vertical ? interval->offset : it->upper();
            y1 = interval->vertical ? it->lower() : interval->offset;
            y2 = interval->vertical ? it->upper() : interval->offset;
            l = interval->l;
            seg s(pt(x1,y1), pt(x2,y2));
            interval->segs.push_back(s);
            interval->elems.push_back(elemindex);
            
            rtree_t[l]->insert({s, elemindex});
            rtree_t.elem2track[elemindex] = i;
            elemindex++;
            it++;
        }
    }
}


