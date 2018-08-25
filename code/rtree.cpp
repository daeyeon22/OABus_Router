#include "route.h"
#include "circuit.h"

//#define DEBUG_RTREE
//#define DEBUG_CREATE_WIRE


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
   
    rtree.update_wire(curwire->bitid, xs, ys, curwire->l, true);
    
    
    
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

    rtree.update_wire(curwire->bitid, xs, ys, curwire->l, false);
    
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
    //rsmt[rsmt.treeID[w.busid]]->wires.push_back(w.id);
#ifdef DEBUG_CREATE_WIRE
    printf("Created Wire (%d %d) (%d %d) M%d\n",
            w.x1, w.y1, w.x2, w.y2, w.l);

#endif
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
    
    rtree.update_wire(bitid, xs, ys, w.l, false);

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

void OABusRouter::Router::intersection_pin(int pinx[], int piny[], int l1,  int wirex[], int wirey[], int l2, int iterx, int itery, int &x, int &y)
{
    typedef PointBG pt;
    typedef SegmentBG seg;
    typedef BoxBG box;

    bool vertical = ckt->is_vertical(l2);
    if(l1 != l2)
    {
        int minOffset;
        int minDist = INT_MAX;
        int dist;
        vector<int> &offsets = ckt->layers[l1].trackOffsets;
        vector<int>::iterator lower, upper;

        if(vertical){
            lower = lower_bound(offsets.begin(), offsets.end(), piny[0]);
            upper = upper_bound(offsets.begin(), offsets.end(), piny[1]);
            x = wirex[0];
        }else{
            lower = lower_bound(offsets.begin(), offsets.end(), pinx[0]);
            upper = upper_bound(offsets.begin(), offsets.end(), pinx[1]);
            y = wirey[0];
        }


        if(iterx == -1 && itery == -1)
        {
            y = (vertical)? *lower : y;
            x = (vertical)? x : *lower;
        }
        else
        {
            while(lower != upper)
            {
                y = (vertical)? *lower : y;
                x = (vertical)? x : *lower;
                lower++;
                dist = abs(iterx - x) + abs(itery - y);

                if(dist < minDist)
                {
                    minOffset = (vertical) ? y : x;
                    minDist = dist;
                }
            }

            y = (vertical)? minOffset : y;
            x = (vertical)? x : minOffset;
        }
    }
    else
    {
        box pin(pt(pinx[0], piny[0]), pt(pinx[1], piny[1]));
        if(bg::intersects(pt(wirex[0], wirey[0]), pin))
        {
            x = wirex[0];
            y = wirey[0];
        }
        else if(bg::intersects(pt(wirex[1], wirey[1]), pin))
        {
            x = wirex[1];
            y = wirey[1];
        }
        else
        {
            if(iterx == -1 && itery == -1)
            {
                x = vertical ? wirex[0] : (int)(1.0*(pinx[0] + pinx[1])/2);
                y = vertical ? (int)(1.0*(piny[0] + piny[1])/2) : wirey[0];
            }
            else
            {
                int dist1 = vertical ? abs(itery - piny[0]) : abs(iterx - pinx[0]);
                int dist2 = vertical ? abs(itery - piny[1]) : abs(iterx - pinx[1]);
                if(dist1 < dist2)
                {
                    x = vertical? wirex[0] : pinx[0];
                    y = vertical? piny[0] : wirey[0];
                }
                else
                {
                    x = vertical? wirex[0] : pinx[1];
                    y = vertical? piny[1] : wirey[0];
                }
            }
        }
    }   


}


bool OABusRouter::Rtree::intersection(int t1, int t2, int& x, int&y)
{
    if(abs(track_layer(t1) - track_layer(t2)) != 1)
    {
        return false;
    }

    if(track_vertical(t1) && !track_vertical(t2))
    {
        x = track_offset(t1);
        y = track_offset(t2);
    }
    else if(!track_vertical(t1) && track_vertical(t2))
    {
        x = track_offset(t2);
        y = track_offset(t1);
    }
    else
    {
        return false;
    }

    return true;
}


int OABusRouter::Rtree::width(int elemid)
{
    return containers[elem2track[elemid]].width;
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

int OABusRouter::Rtree::track_width(int trackid)
{
    return containers[trackid].width;
}


bool OABusRouter::Rtree::vertical(int elemid)
{
    return containers[elem2track[elemid]].vertical;
}

bool OABusRouter::Rtree::track_vertical(int trackid)
{
    return containers[trackid].vertical;
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

bool OABusRouter::Rtree::update_wire(int bitid, int x[], int y[], int l, bool remove)
{
    if(remove)
        obstacle[l].remove({ box(pt(x[0], y[0]), pt(x[1], y[1])), bitid } );
    else
        obstacle[l].insert({ box(pt(x[0], y[0]), pt(x[1], y[1])), bitid } );
    return true;
}

bool OABusRouter::Rtree::intersects(int x[], int y[], int l)
{
   vector<pair<box,int>> queries;
   obstacle[l].query(bgi::intersects(box(pt(x[0], y[0]), pt(x[1], y[1]))), back_inserter(queries));
   return queries.size() > 0;
}



void OABusRouter::Rtree::design_ruled_area(int x[], int y[], int width, int spac, bool vertical)
{   
    if(vertical)
    {   
        x[0] -= ((int)(1.0*width / 2) + spac);
        x[1] += ((int)(1.0*width / 2) + spac);
        y[0] -= spac;
        y[1] += spac;
    }
    else
    {   
        y[0] -= ((int)(1.0*width / 2) + spac);
        y[1] += ((int)(1.0*width / 2) + spac);
        x[0] -= spac;
        x[1] += spac;
    }
} 


bool OABusRouter::Rtree::spacing_violations(int bitid, int x[], int y[], int l, int width, int spacing, bool vertical)
{
    design_ruled_area(x, y, width, spacing, vertical);

    vector<pair<box,int>> queries;
    box area(pt(x[0], y[0]), pt(x[1], y[1]));
    

    // Design boundary spacing violations
    int db[4] = { ckt->originX, ckt->originY, ckt->originX + ckt->width, ckt->originY + ckt->height };
    seg s1( pt(db[0], db[1]), pt(db[0], db[3]) );
    seg s2( pt(db[2], db[1]), pt(db[2], db[3]) );
    seg s3( pt(db[0], db[1]), pt(db[2], db[1]) );
    seg s4( pt(db[0], db[3]), pt(db[2], db[3]) );

    if(bg::intersects(s1, area) || bg::intersects(s2, area) || 
       bg::intersects(s3, area) || bg::intersects(s4, area))
        return true;

    
    
    // Wire to wire spacing violations
    obstacle[l].query(bgi::intersects(area) , back_inserter(queries));
    for(auto& it : queries)
    {
        if(bg::touches(it.first , area))
            continue;

        if(it.second == OBSTACLE)
            return true;
        
        if(it.second != bitid)
            return true;
    }

    return false;
}


bool OABusRouter::Rtree::compactness(int numbits, int mx[], int my[], int x,  int y, int l, int align, int direction, int width, int spacing)
{
    enum Direction
    {
        Left,
        Right,
        Up,
        Down,
        Point,
        // target wire routing topologies
        T_Junction,
        EndPointLL,
        EndPointUR
    };

    int xs[2], ys[2];

    if(align == VERTICAL)
    {
        xs[0] = x;
        xs[1] = x;
        ys[0] = my[0];
        ys[1] = my[1];
        
        if(direction == Direction::Left)
            xs[0] -= numbits*(width + spacing);
        if(direction == Direction::Right)
            xs[1] += numbits*(width + spacing);
    }
    else
    {
        xs[0] = mx[0];
        xs[1] = mx[1];
        ys[0] = y;
        ys[1] = y;
        
        if(direction == Direction::Down)
            ys[0] -= numbits*(width + spacing);
        if(direction == Direction::Up)
            ys[1] += numbits*(width + spacing);       

    }


#ifdef DEBUG_COMPACTNESS
    bool hasSpacingViolation = spacing_violations(-1, xs, ys, l);
    if(hasSpacingViolation)
    {
        switch(direction){
            case Direction::Left:
                printf("Left");
                break;

            case Direction::Right:
                printf("Right");
                break;

            case Direction::Down:
                printf("Down");
                break;

            case Direction::Up:
                printf("Up");
                break;

            default:
                break;
        } 
        printf("\nmultipin bound (%d %d) (%d %d)\ncurrent iterating (%d %d)\n", mx[0], my[0], mx[1], my[1], x, y);
        printf("check area (%d %d) (%d %d)", xs[0], ys[0], xs[1], ys[1]);
        printf(" -> violations...");
        printf("\n\n");
    }
#endif

    return !spacing_violations(-1, xs, ys, l);
}





bool OABusRouter::Rtree::spacing_violations(int bitid, int x[], int y[], int l)
{
    box area(pt(x[0], y[0]), pt(x[1], y[1])); 
    // Design boundary spacing violations
    int db[4] = { ckt->originX, ckt->originY, ckt->originX + ckt->width, ckt->originY + ckt->height };
    seg s1( pt(db[0], db[1]), pt(db[0], db[3]) );
    seg s2( pt(db[2], db[1]), pt(db[2], db[3]) );
    seg s3( pt(db[0], db[1]), pt(db[2], db[1]) );
    seg s4( pt(db[0], db[3]), pt(db[2], db[3]) );

    if(bg::intersects(s1, area) || bg::intersects(s2, area) || 
       bg::intersects(s3, area) || bg::intersects(s4, area))
        return true;



    vector<pair<box,int>> queries;
    obstacle[l].query(bgi::intersects(area), back_inserter(queries));
    for(auto& it : queries)
    {
        if(bg::touches(it.first , area))
            continue;
        if(it.second == OBSTACLE)
            return true;
        if(it.second != bitid)
            return true;
    }

    return false;
}


