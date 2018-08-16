#include "route.h"
#include "circuit.h"

#define DEBUG_RTREE


bool OABusRouter::Wire::leaf()
{
    return intersection.size() < 2;
}

// return true if updating is valid. 
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
    int xs[2], ys[2];
    if(curwire->vertical)
    {
        xs[0] = curwire->x1 - (int)(1.0*curwire->width / 2);
        xs[1] = curwire->x2 + (int)(1.0*curwire->width / 2);
        ys[0] = curwire->y1;
        ys[1] = curwire->y2;
    }
    else
    {
        xs[0] = curwire->x1;
        xs[1] = curwire->x2;
        ys[0] = curwire->y1 - (int)(1.0*curwire->width / 2);
        ys[1] = curwire->y2 + (int)(1.0*curwire->width / 2);
    }
   
    rtree.update_wire(wireid, xs, ys, curwire->l, true);
    
    
    
    rtree.insert_element(curwire->trackid, origXs, origYs, curwire->l, false);
    
    curwire->x1 = x[0];
    curwire->x2 = x[1];
    curwire->y1 = y[0];
    curwire->y2 = y[1];
    if(curwire->vertical)
    {
        xs[0] = curwire->x1 - (int)(1.0*curwire->width / 2);
        xs[1] = curwire->x2 + (int)(1.0*curwire->width / 2);
        ys[0] = curwire->y1;
        ys[1] = curwire->y2;
    }
    else
    {
        xs[0] = curwire->x1;
        xs[1] = curwire->x2;
        ys[0] = curwire->y1 - (int)(1.0*curwire->width / 2);
        ys[1] = curwire->y2 + (int)(1.0*curwire->width / 2);
    }

    rtree.update_wire(wireid, xs, ys, curwire->l, false);
    
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

    // rtree update for track
    rtree.insert_element(trackid, x, y, l, true);
    
    // rtree update for wire
    int xs[2], ys[2];
    if(w.vertical)
    {
        xs[0] = x[0] - (int)(1.0*w.width / 2);
        xs[1] = x[1] + (int)(1.0*w.width / 2);
        ys[0] = y[0];
        ys[1] = y[1];
    }
    else
    {
        xs[0] = x[0];
        xs[1] = x[1];
        ys[0] = y[0] - (int)(1.0*w.width / 2);
        ys[1] = y[1] + (int)(1.0*w.width / 2);
    }
    
    rtree.update_wire(w.id, xs, ys, w.l, false);

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

bool OABusRouter::Rtree::update_wire(int wireid, int x[], int y[], int l, bool remove)
{
    if(remove)
        obstacle[l].remove({ box(pt(x[0], y[0]), pt(x[1], y[1])), wireid } );
    else
        obstacle[l].insert({ box(pt(x[0], y[0]), pt(x[1], y[1])), wireid } );
    return true;
}

bool OABusRouter::Rtree::intersects(int x[], int y[], int l)
{
   vector<pair<box,int>> queries;
   obstacle[l].query(bgi::intersects(box(pt(x[0], y[0]), pt(x[1], y[1]))), back_inserter(queries));
   return queries.size() > 0;
}

bool OABusRouter::Rtree::spacing_violations(int bitid, int x[], int y[], int l)
{
    vector<pair<box,int>> queries;
    obstacle[l].query(bgi::overlaps(box(pt(x[0], y[0]), pt(x[1], y[1]))), back_inserter(queries));
    for(auto& it : queries)
    {
        if(it.second == OBSTACLE)
        {
            return true;
        }
        else
        {
            Wire* curwire = &rou->wires[it.second];
            if(curwire->bitid != bitid)
                return true;
        }
    }

    return false;
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
    //
    rtree.wire = vector<BoxRtree>(numLayers);
    rtree.obstacle = vector<BoxRtree>(numLayers);

    //
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
        // add into the obstacle tree
        rtree.obstacle[l].insert({b, OBSTACLE});
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





