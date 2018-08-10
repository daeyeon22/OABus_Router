#include "route.h"
#include "circuit.h"

#define DEBUG_RTREE

SegRtree* OABusRouter::Router::GetTrackRtree()
{
    return &rtree.track;
}
int OABusRouter::Rtree::layer(int elemid)
{
    return containers[elem2track[elemid]].l; //trackNuml[elemid];
}

int OABusRouter::Rtree::trackid(int elemid)
{
    return elem2track[elemid]; //trackID[elemid];
}

int OABusRouter::Rtree::direction(int elemid)
{
    return containers[elem2track[elemid]].vertical ? VERTICAL : HORIZONTAL; //trackDir[elemid];
}

int OABusRouter::Rtree::offset(int elemid)
{
    return containers[elem2track[elemid]].offset;
}

int OABusRouter::Rtree::track_offset(int trackid)
{
    return containers[trackid].offset;
}

int OABusRouter::Rtree::track_direction(int trackid)
{
    return containers[trackid].vertical ? VERTICAL : HORIZONTAL;
}

int OABusRouter::Rtree::track_layer(int trackid)
{
    return containers[trackid].l;
}

IntervalSetT OABusRouter::Rtree::track_empty(int trackid)
{
    return containers[trackid].empty;
}

bool OABusRouter::Rtree::insert_element(int trackid, int x[], int y[], int l, bool remove)
{
    int lower, upper;
    int i;
    int offset;
    int numElems;
    bool vertical;
    
    typedef SegmentBG seg;
    typedef PointBG pt;
    typedef BoxBG box;

    Container* curct;
    seg s;
    curct = &containers[trackid];
    
    numElems = curct->elems.size();
    offset = curct->offset;
    vertical = curct->vertical;

    //
    for(i=0; i < numElems; i++)
        track.remove({curct->segs[i], curct->elems[i]});

    curct->segs.clear();
    curct->elems.clear();

    // Updated interval set
    if(remove)
        curct->empty -= (vertical) ? IntervalT::closed(y[0], y[1]) : IntervalT::closed(x[0], x[1]);
    else
        curct->empty += (vertical) ? IntervalT::closed(y[0], y[1]) : IntervalT::closed(x[0], x[1]);

    IntervalSetT::iterator it = curct->empty.begin();
    while(it != curct->empty.end())
    {
        s = vertical ?
            seg(pt(offset, it->lower()), pt(offset, it->upper())) :
            seg(pt(it->lower(), offset), pt(it->upper(), offset));

        track.insert({s,elemindex});
        elem2track[elemindex] = trackid;
        curct->segs.push_back(s);
        curct->elems.push_back(elemindex);
        elemindex++;
        it++;
    }
    
    return true;
}



void OABusRouter::Router::InitRtree()
//CreateTrackRtree()
{
    int numTracks, numObstacles, numLayers, numPins, tid, curl;
    int trackid, l; 
    int x1, y1, x2, y2, i, dir;
    int offset;
    int& elemindex = rtree.elemindex;
    bool isVertical;
    SegmentBG curS;
    Track* curT;
    Pin* curpin;
    Obstacle* curobs;
    SegRtree* curRtree;
    Container* curct;

    typedef SegmentBG seg;
    typedef PointBG pt;
    typedef BoxBG box;


    //curRtree = GetTrackRtree();
    curRtree = &rtree.track;
    numTracks = ckt->tracks.size(); //interval.numtracks;
    numObstacles = ckt->obstacles.size();
    numLayers = ckt->layers.size();
    numPins = ckt->pins.size();
    elemindex=0;

#ifdef DEBUG_RTREE
    if(numTracks > ckt->tracks.size())
    {
        printf("Invalid #tracks %d %d\n", numTracks, ckt->tracks.size());
        exit(0);
    }
#endif

    // Initialize Intervals.
    
    vector<SegRtree> trackrtree(numLayers);
    rtree.containers = vector<Container>(numTracks);
    for(i=0; i < numTracks; i++)
    {
        curT = &ckt->tracks[i];
        trackid = curT->id;
        curct = &rtree.containers[trackid];
        curct->trackid = trackid;
        curct->offset = curT->offset;
        curct->l = curT->l;
        curct->vertical = ckt->is_vertical(curT->l);

        x1 = curT->llx;
        y1 = curT->lly;
        x2 = curT->urx;
        y2 = curT->ury;
        l = curT->l;

        seg s(pt(x1, y1), pt(x2, y2));
        trackrtree[l].insert({s, trackid});

        curct->empty += (curct->vertical) ?
            IntervalT::open(y1, y2) : IntervalT::open(x1, x2);
    }
    
 
    for(i=0; i < numObstacles; i++)
    {
        curobs = &ckt->obstacles[i];
        x1 = curobs->llx;
        y1 = curobs->lly;
        x2 = curobs->urx;
        y2 = curobs->ury;
        l = curobs->l;

        box b(pt(x1,y1), pt(x2,y2));
        vector<pair<seg, int>> queries;
        trackrtree[l].query(bgi::intersects(b), back_inserter(queries));
        for(auto& it : queries)
        {
            trackid = it.second;
            curct = &rtree.containers[trackid];

            curct->empty -= (curct->vertical)?
                IntervalT::closed(y1, y2) : IntervalT::closed(x1, x2);
        }
    }

    for(i=0; i < numPins; i++)
    {
        curpin = &ckt->pins[i];
        x1 = curpin->llx;
        y1 = curpin->lly;
        x2 = curpin->urx;
        y2 = curpin->ury;
        l = curpin->l;

        box b(pt(x1,y1), pt(x2,y2));
        vector<pair<seg, int>> queries;
        trackrtree[l].query(bgi::intersects(b), back_inserter(queries));
        for(auto& it : queries)
        {
            trackid = it.second;
            curct = &rtree.containers[trackid];

            curct->empty -= (curct->vertical)?
                IntervalT::closed(y1, y2) : IntervalT::closed(x1, x2);
        }
    }


    
    
    
    for(i=0; i < numTracks; i++)
    {
        curct = &rtree.containers[i];
        IntervalSetT::iterator it = curct->empty.begin();
        while(it != curct->empty.end())
        {
            x1 = curct->vertical ? curct->offset : it->lower();
            x2 = curct->vertical ? curct->offset : it->upper();
            y1 = curct->vertical ? it->lower() : curct->offset;
            y2 = curct->vertical ? it->upper() : curct->offset;
            seg s(pt(x1,y1), pt(x2,y2));
            curct->segs.push_back(s);
            curct->elems.push_back(elemindex);

            curRtree->insert({s, elemindex});

            rtree.elem2track[elemindex] = curct->trackid;
            elemindex++;
            it++;
        }
    }
}





