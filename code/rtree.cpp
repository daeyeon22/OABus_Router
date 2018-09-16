#include "func.h"
#include "rtree.h"
#include "route.h"
#include "circuit.h"

//#define DEBUG_RTREE
//#define DEBUG_CREATE_WIRE

enum Direction
{
    Left = 10,
    Right = 11,
    Up = 12,
    Down = 13,
    Point = 14,
    // target wire routing topologies
    T_Junction = 15,
    EndPointLL = 16,
    EndPointUR = 17
};


void design_ruled_area(int x[], int y[], int width, int spac, bool vertical)
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


/*
void into_array(int x1, int x2, int y1, int y2, int x[], int y[]);
void into_array(int v1, int v2, int v[]);
void expand_width(int x[], int y[], int width, int vertical);
void pin_area(int x[], int y[], int align, int width, box& box);
void design_ruled_area(int x[], int y[], int width, int spacing, bool vertical);
*/


bool OABusRouter::Wire::leaf()
{
    return intersection.size() < 2;
}
int OABusRouter::TrackRtree::get_width(int e)
{
    return tracks[elem2track[e]].width;
}

int OABusRouter::TrackRtree::get_width_t(int t)
{
    return tracks[t].width;
}

int OABusRouter::TrackRtree::get_layer(int e)
{
    return tracks[elem2track[e]].l; 
}

int OABusRouter::TrackRtree::get_layer_t(int t)
{
    return tracks[t].l; 
}

int OABusRouter::TrackRtree::get_trackid(int e)
{
    return elem2track[e]; 
}

int OABusRouter::TrackRtree::get_direction(int e)
{
    return tracks[elem2track[e]].vertical ? VERTICAL : HORIZONTAL; 
}

int OABusRouter::TrackRtree::get_direction_t(int t)
{
    return tracks[t].vertical ? VERTICAL : HORIZONTAL; 
}

int OABusRouter::TrackRtree::get_offset(int e)
{
    return tracks[elem2track[e]].offset;
}

int OABusRouter::TrackRtree::get_offset_t(int t)
{
    return tracks[t].offset;
}

bool OABusRouter::TrackRtree::is_vertical(int e)
{
    return tracks[elem2track[e]].vertical;
}

bool OABusRouter::TrackRtree::is_vertical_t(int t)
{
    return tracks[t].vertical;
}

bool OABusRouter::TrackRtree::insert_element(int trackid, int x1, int y1, int x2, int y2, int l, bool remove)
{
    int x[2] = {x1, x2};
    int y[2] = {y1, y2};
    return insert_element(trackid, x, y, l, remove);
}

bool OABusRouter::TrackRtree::insert_element(int trackid, int x[], int y[], int l, bool remove)
{
    typedef SegmentBG seg;
    typedef PointBG pt;
    typedef BoxBG box;
    int i, offset, numelems;
    bool vertical;
    Interval* interval;

    seg s;
    interval = get_interval(trackid);
    numelems = interval->elems.size();
    offset = interval->offset;
    vertical = interval->vertical;

    //
    for(i=0; i < numelems; i++)
        rtree[l].remove({interval->segs[i], interval->elems[i]});


    interval->segs.clear();
    interval->elems.clear();

    // Updated interval set
    if(remove)
        interval->empty -= (vertical) ? IntervalT::closed(y[0], y[1]) : IntervalT::closed(x[0], x[1]);
    else
        interval->empty += (vertical) ? IntervalT::closed(y[0], y[1]) : IntervalT::closed(x[0], x[1]);

    IntervalSetT::iterator it = interval->empty.begin();
    while(it != interval->empty.end())
    {
        s = vertical ?
            seg(pt(offset, it->lower()), pt(offset, it->upper())) :
            seg(pt(it->lower(), offset), pt(it->upper(), offset));

        rtree[l].insert({s,elemindex});

        elem2track[elemindex] = trackid;

        interval->segs.push_back(s);
        interval->elems.push_back(elemindex);
        elemindex++;
        it++;
    }
    
    return true;
}

void OABusRouter::TrackRtree::query(int mode, seg geo, int l, vector<pair<seg,int>>& queries)
{
    if(l < 0 || l > rtree.size()-1)
        return;

    
    switch(mode)
    {
        case QueryMode::Intersects:
            rtree[l].query(bgi::intersects(geo), back_inserter(queries));
            break;

        case QueryMode::Overlaps:
            //rtree[l].query(bgi::overlaps(geo), back_inserter(queries));
            break;

        case QueryMode::Covered:
            //rtree[l].query(bgi::covered_by(geo), back_inserter(queries));
            break;

        case QueryMode::Within:
            //rtree[l].query(bgi::within(geo), back_inserter(queries));
            break;

        case QueryMode::Disjoint:
            //rtree[l].query(bgi::disjoint(geo), back_inserter(queries));
            break;
        default:
            break;
    }
}


void OABusRouter::TrackRtree::query(int mode, box geo, int l, vector<pair<seg,int>>& queries)
{
    if(l < 0 || l > rtree.size()-1)
        return;
    
    switch(mode)
    {
        case QueryMode::Intersects:
            rtree[l].query(bgi::intersects(geo), back_inserter(queries));
            break;

        case QueryMode::Overlaps:
            //rtree[l].query(bgi::overlaps(geo), back_inserter(queries));
            break;

        case QueryMode::Covered:
            //rtree[l].query(bgi::covered_by(geo), back_inserter(queries));
            break;

        case QueryMode::Within:
            //rtree[l].query(bgi::within(geo), back_inserter(queries));
            break;

        case QueryMode::Disjoint:
            //rtree[l].query(bgi::disjoint(geo), back_inserter(queries));
            break;
        default:
            break;
    }
}


void OABusRouter::TrackRtree::query(int mode, seg geo, int lower, int upper, vector<pair<seg, int>> &queries)
{
    lower = max(0, lower);
    upper = min(upper, (int)rtree.size()-1);

    while(lower <= upper)
    {
        int l = lower;
        switch(mode)
        {
            case QueryMode::Intersects:
                rtree[l].query(bgi::intersects(geo), back_inserter(queries));
                break;

            case QueryMode::Overlaps:
                //rtree[l].query(bgi::overlaps(geo), back_inserter(queries));
                break;

            case QueryMode::Covered:
                //rtree[l].query(bgi::covered_by(geo), back_inserter(queries));
                break;
            
            case QueryMode::Within:
                //rtree[l].query(bgi::within(geo), back_inserter(queries));
                break;

            case QueryMode::Disjoint:
                //rtree[l].query(bgi::disjoint(geo), back_inserter(queries));
                break;
            default:
                break;
        }
        lower++;
    }
}

void OABusRouter::TrackRtree::query(int mode, box geo, int lower, int upper, vector<pair<seg, int>> &queries)
{
    lower = max(0, lower);
    upper = min(upper, (int)rtree.size()-1);

    while(lower <= upper)
    {
        int l = lower;
        switch(mode)
        {
            case QueryMode::Intersects:
                rtree[l].query(bgi::intersects(geo), back_inserter(queries));
                break;

            case QueryMode::Overlaps:
                break;

            case QueryMode::Covered:
                break;
            
            case QueryMode::Within:
                break;

            case QueryMode::Disjoint:
                break;

            default:
                break;
        }
        lower++;
    }
}

bool OABusRouter::ObstacleRtree::insert_obstacle(int bitid, int x[], int y[], int l, bool remove)
{
    if(remove)
        rtree[l].remove({ box(pt(x[0], y[0]), pt(x[1], y[1])), bitid } );
    else
        rtree[l].insert({ box(pt(x[0], y[0]), pt(x[1], y[1])), bitid } );
    return true;
}

bool OABusRouter::ObstacleRtree::short_violation(int bitid, int wirex[], int wirey[], int wl, int tarx[], int tary[], int tl)
{

    bool hasShort = false;
    box area(pt(wirex[0], wirey[0]), pt(wirex[1], wirey[1]));
    if(wl == tl)
        insert_obstacle(bitid, tarx, tary, tl, true);


    vector<pair<box,int>> queries;
    rtree[wl].query(bgi::intersects(area), back_inserter(queries));
    for(auto& it : queries)
    {
        if(it.second == bitid)
        {
            hasShort = true;
            break;
        }
    }

    if(wl == tl)
        insert_obstacle(bitid, tarx, tary, tl, false);
    
    return hasShort;
}

int OABusRouter::ObstacleRtree::num_spacing_violations(int bitid, int x[], int y[], int l, int width, int spacing, bool vertical)
{
    int xs[] = {x[0], x[1]};
    int ys[] = {y[0], y[1]};
    design_ruled_area(xs, ys, width, spacing, vertical);
    /*
    int totalSV = num_spacing_violations_ndr(bitid, xs, ys, l);
        printf("(%d %d) (%d %d) M%d spacing violations occurs #SPV %d\n",
                x[0], y[0], x[1], y[1], l, totalSV);
    */
    return num_spacing_violations_ndr(bitid, xs, ys, l);
}

int OABusRouter::ObstacleRtree::num_spacing_violations_ndr(int bitid, int x[], int y[], int l)
{
    
    int totalSV=0;
    box area(pt(x[0], y[0]), pt(x[1], y[1]));
    // Design boundary spacing violations
    box design_boundary(pt(db[0], db[1]), pt(db[2], db[3]));

    seg s1( pt(db[0], db[1]), pt(db[0], db[3]) );
    seg s2( pt(db[2], db[1]), pt(db[2], db[3]) );
    seg s3( pt(db[0], db[1]), pt(db[2], db[1]) );
    seg s4( pt(db[0], db[3]), pt(db[2], db[3]) );

    //if(bg::intersects(s1, area) || bg::intersects(s2, area) || 
    //   bg::intersects(s3, area) || bg::intersects(s4, area))
    if(!bg::within(area, design_boundary))
    {
#ifdef DEBUG_SPACING_VIOLATION
        printf("spacing violation occurs (%d %d) (%d %d) m%d -> DESIGN_BOUNDARY (%d %d) (%d %d)\n",
                x[0], y[0], x[1], y[1], l, db[0], db[1], db[2], db[3]);
#endif
        totalSV++;
    }

    // Wire to wire spacing violations
    vector<pair<box,int>> queries;
    rtree[l].query(bgi::intersects(area) , back_inserter(queries));
    for(auto& it : queries)
    {
        // ???
        if(bg::touches(it.first , area))
            continue;

        if(it.second == OBSTACLE)
        {
#ifdef DEBUG_SPACING_VIOLATION
            printf("spacing violation occurs (%d %d) (%d %d) m%d -> OBSTACLE\n", x[0], y[0], x[1], y[1], l);
#endif
            totalSV++;
        }
        else
        {
            if(it.second != bitid)
            {
#ifdef DEBUG_SPACING_VIOLATION
                printf("spacing violation occurs (%d %d) (%d %d) m%d -> bitid %d\n", x[0], y[0], x[1], y[1], l, it.second);
#endif
                totalSV++;
            }
        }
    }

    return totalSV;
}

bool OABusRouter::ObstacleRtree::spacing_violations(int busid, int l, polygon& poly)
{
    
    box design_boundary(pt(db[0], db[1]), pt(db[2], db[3]));
    box poly_envelope;
    bg::envelope(poly, poly_envelope);
    if(!bg::within(poly_envelope, design_boundary))
        return true;


    vector<pair<box,int>> queries;
    rtree[l].query(bgi::intersects(poly), back_inserter(queries));
    for(auto& it : queries)
    {
        //if(bg::disjoint(it.first, poly))
        //    continue;

        if(it.second == OBSTACLE)
            //continue;
            return true;

        if(rou->bit2bus[it.second] != busid)
            return true;
    }

    return false;
}

bool OABusRouter::ObstacleRtree::spacing_violations(int bitid, int x[], int y[], int l, int width, int spacing, bool vertical)
{
    int xs[] = {x[0], x[1]};
    int ys[] = {y[0], y[1]};
    design_ruled_area(xs, ys, width, spacing, vertical);
    return spacing_violations_ndr(bitid, xs, ys, l);
}

    
bool OABusRouter::ObstacleRtree::spacing_violations_ndr(int bitid, int x[], int y[], int l)
{
    box area(pt(x[0], y[0]), pt(x[1], y[1]));
    // Design boundary spacing violations
    box design_boundary(pt(db[0], db[1]), pt(db[2], db[3]));

    seg s1( pt(db[0], db[1]), pt(db[0], db[3]) );
    seg s2( pt(db[2], db[1]), pt(db[2], db[3]) );
    seg s3( pt(db[0], db[1]), pt(db[2], db[1]) );
    seg s4( pt(db[0], db[3]), pt(db[2], db[3]) );

    //if(bg::intersects(s1, area) || bg::intersects(s2, area) || 
    //   bg::intersects(s3, area) || bg::intersects(s4, area))
    if(!bg::within(area, design_boundary))
    {
#ifdef DEBUG_SPACING_VIOLATION
        printf("spacing violation occurs (%d %d) (%d %d) m%d -> DESIGN_BOUNDARY (%d %d) (%d %d)\n",
                x[0], y[0], x[1], y[1], l, db[0], db[1], db[2], db[3]);
#endif
        return true;
    }

    // Wire to wire spacing violations
    vector<pair<box,int>> queries;
    rtree[l].query(bgi::intersects(area) , back_inserter(queries));
    for(auto& it : queries)
    {
        // ???
        if(bg::touches(it.first , area))
            continue;

        if(it.second == OBSTACLE)
        {
#ifdef DEBUG_SPACING_VIOLATION
            printf("spacing violation occurs (%d %d) (%d %d) m%d -> OBSTACLE\n", x[0], y[0], x[1], y[1], l);
#endif
            return true;
        }

        if(it.second != bitid)
        {
#ifdef DEBUG_SPACING_VIOLATION
            printf("spacing violation occurs (%d %d) (%d %d) m%d -> bitid %d\n", x[0], y[0], x[1], y[1], l, it.second);
#endif
            return true;
        }
    }
    return false;
}

int OABusRouter::PinRtree::num_diff_pins_on_track(int bitid, int x[], int y[], int l, int width, int spacing)
{
    int xExt[2] = {x[0], x[1]};
    int yExt[2] = {y[0], y[1]};
    if(ckt->is_vertical(l))
    {
        xExt[0] -= width/2 + spacing;
        xExt[1] += width/2 + spacing;
    }
    else
    {
        yExt[0] -= width/2 + spacing;
        yExt[1] += width/2 + spacing;
    }
    int diffCnt = 0;
    box area(pt(xExt[0], yExt[0]), pt(xExt[1], yExt[1]));
    for(int i=l-1; i <= l+1; i++)
    {
        vector<pair<box,int>> queries;
        if(i < 0 || i >= ckt->layers.size())
            continue;
        rtree[i].query(bgi::intersects(area), back_inserter(queries));
        for(auto& it : queries)
        {
            if((rou->pin2align[it.second] == VERTICAL) ==  ckt->is_vertical(i))
                continue;
            if(rou->pin2bus[it.second] == rou->bit2bus[bitid])
                continue;

            if(rou->pin2bit[it.second] != bitid)
                diffCnt++;
        }


    }
    return diffCnt;
}
int OABusRouter::PinRtree::num_diff_pins_on_track(int bitid, int t, int width, int spacing)
{
    Track* curt = &ckt->tracks[t];
    int x[2] = {curt->llx, curt->urx};
    int y[2] = {curt->lly, curt->ury};
    if(ckt->is_vertical(curt->l))
    {
        x[0] -= width/2 + spacing;
        x[1] += width/2 + spacing;
    }
    else
    {
        y[0] -= width/2 + spacing;
        y[1] += width/2 + spacing;
    }
    int diffCnt = 0;
    box area(pt(x[0], y[0]), pt(x[1], y[1]));
    vector<pair<box,int>> queries;
    for(int i=curt->l-1; i <= curt->l+1; i++)
    {
        if(i < 0 || i >= ckt->layers.size())
            continue;
        rtree[i].query(bgi::intersects(area), back_inserter(queries));

    }
    for(auto& it : queries)
    {
        if((rou->pin2align[it.second] == VERTICAL) ==  ckt->is_vertical(curt->l))
            continue;

        if(rou->pin2bit[it.second] != bitid)
            diffCnt++;
    }

    return diffCnt;
}

int OABusRouter::PinRtree::num_diff_pins_on_track(int bitid, seg elem, int l)
{
    int diffCnt = 0;
    vector<pair<box,int>> queries;
    rtree[l].query(bgi::intersects(elem), back_inserter(queries));
    for(auto& it : queries)
    {
        if(rou->pin2bit[it.second] != bitid)
            diffCnt++;
    }

    return diffCnt;
}

bool OABusRouter::BitRtree::short_violation(int x[], int y[], int l, set<int>& except1, set<int>& except2)
{
    box area(pt(x[0], y[0]), pt(x[1], y[1]));
    vector<pair<box,int>> queries;
    rtree[l].query(bgi::intersects(area), back_inserter(queries));
    for(auto& it : queries)
    {
        int elemid = it.second;
        if(elem2type[elemid] == PINTYPE)
        {
            if(except1.find(elem2pin[elemid]) == except1.end())
                return true;
        }
        
        if(elem2type[elemid] == WIRETYPE)
        {
            if(except2.find(elem2wire[elemid]) == except2.end())
                return true;
        }
    }
    return false;
}

int OABusRouter::PinRtree::num_diff_bus_between_twopins(int busid, int p1, int p2, int l)
{
    int x[2], y[2];
    int numPins = 0;
    box b1 = elems[p1];
    box b2 = elems[p2];
    x[0] = min((int)bg::get<0,0>(b1), (int)bg::get<0,0>(b2));
    x[1] = max((int)bg::get<1,0>(b1), (int)bg::get<1,0>(b2));
    y[0] = min((int)bg::get<0,1>(b1), (int)bg::get<0,1>(b2));
    y[1] = max((int)bg::get<1,1>(b1), (int)bg::get<1,1>(b2));

    box envelope(pt(x[0], y[0]), pt(x[1], y[1]));
    vector<pair<box,int>> queries;
    rtree[l].query(bgi::intersects(envelope), back_inserter(queries));

    
    for(auto& it : queries)
    {
        if(busid != elem2bus[it.second])
            numPins++;
    }
    return numPins;
}

void OABusRouter::PinRtree::remove_pins(vector<int> &pins)
{
    int l;
    for(auto& pinid : pins)
    {
        l = ckt->pins[pinid].l;
        rtree[l].remove({elems[pinid], pinid});
    }
}

void OABusRouter::Router::construct_bit_rtree(int bitid, BitRtree& bitrtree)
{
    int i, l, pinid, wireid, elemid, numPins, numWires;
    int x[2], y[2];
    bool vertical;
    bitrtree = BitRtree(ckt->layers.size());
    Bit* curbit = &ckt->bits[bitid];
    dense_hash_map<int,int> width = ckt->buses[ckt->busHashMap[curbit->busName]].width;

    numPins = curbit->pins.size();
    numWires = curbit->wires.size();

    for(i=0; i < numPins; i++)
    {
        Pin* curpin = &ckt->pins[curbit->pins[i]];
        pinid = curpin->id;
        l = curpin->l;
        elemid = bitrtree.elems.size();

        box pb(pt(curpin->llx, curpin->lly), pt(curpin->urx, curpin->ury));
        bitrtree.elems.push_back(pb);
        bitrtree.rtree[l].insert({pb,elemid});
        bitrtree.elem2type[elemid] = PINTYPE;
        bitrtree.elem2pin[elemid] = pinid;
        bitrtree.elemindex++;
    }

    /*
    for(i=0; i < numWires; i++)
    {
        Wire* curw = &wires[curbit->wires[i]];
        wireid = curw->id;
        l = curw->l;
        elemid = bitrtree.elems.size();
        vertical = curw->vertical;
        x[0] = vertical ? curw->x1 - (int)(1.0*width[l]/2) : curw->x1;
        x[1] = vertical ? curw->x2 + (int)(1.0*width[l]/2) : curw->x2;
        y[0] = vertical ? curw->y1 : curw->y1 - (int)(1.0*width[l]/2);
        y[1] = vertical ? curw->y2 : curw->y2 + (int)(1.0*width[l]/2);
        
        box pb(pt(x[0], y[0]), pt(x[1], y[1]));
        bitrtree.elems.push_back(pb);
        bitrtree.rtree[l].insert({pb,elemid});
        bitrtree.elem2type[elemid] = WIRETYPE;
        bitrtree.elem2wire[elemid] = wireid;
        bitrtree.elemindex++;
    }
    */
}
bool OABusRouter::Router::routability_check(int m, int t, int dir)
{
/*
    enum Direction
    {
        Left,
        Right,
        Up,
        Down
    };
*/
    int l, numbits;
    int t_offset;
    int lower, upper;
    MultiPin* mp = &ckt->multipins[m];
    Layer* layer = &ckt->layers[l];
    l = rtree_t.get_layer(t);
    numbits = mp->pins.size();
    t_offset = rtree_t.get_offset_t(t);


    vector<int> offsets;
    offsets.insert(offsets.end(), layer->offsets.begin(), layer->offsets.end());
    sort(offsets.begin(), offsets.end());
    
    cout << "offsets size   : " << offsets.size();
    cout << "current offset : " << t_offset << endl;
    if(dir == Direction::Up || dir == Direction::Right)
    {
        lower = t_offset;
        upper = *offsets.end(); 
        //lower = GetLowerBound(offsets, t_offset) + 1;
        //upper = offsets.size();
    }
    
    if(dir == Direction::Down || dir == Direction::Left)
    {
        lower = *offsets.begin();
        upper = t_offset;
        //lower = 0;
        //upper = GetUpperBound(offsets, t_offset);
    }



    cout << "available track : " << upper - lower << endl;
    if(numbits > upper-lower)
    {
        return false;
    }
    else
    {
        int required = numbits*(spacing[l] + ckt->buses[mp->busid].width[l]);
        if(required < abs(upper-lower))
        {
            return false;
        }
        return true;
    }
}

bool OABusRouter::ObstacleRtree::bending_available_at_source(int busid, int m, int x, int y, int l, int rDir)
{
    int xs[2], ys[2];
    Bus* curbus = &ckt->buses[busid];
    MultiPin* mp = &ckt->multipins[m];
    int numBits = curbus->numBits;
    int lenReq = numBits * (rou->spacing[l] + curbus->width[l]);


    polygon area1, area2;
    if(rDir == Direction::Left)
    {
        bg::append(area1.outer(), pt(x, mp->lly));
        bg::append(area1.outer(), pt(x, mp->ury));
        bg::append(area1.outer(), pt(x - lenReq, mp->lly));
        
        bg::append(area1.outer(), pt(x, mp->lly));
        bg::append(area1.outer(), pt(x, mp->ury));
        bg::append(area1.outer(), pt(x - lenReq, mp->ury));
    }
    else if(rDir == Direction::Right)
    {

        bg::append(area1.outer(), pt(x, mp->lly));
        bg::append(area1.outer(), pt(x, mp->ury));
        bg::append(area1.outer(), pt(x + lenReq, mp->lly));
        
        bg::append(area1.outer(), pt(x, mp->lly));
        bg::append(area1.outer(), pt(x, mp->ury));
        bg::append(area1.outer(), pt(x + lenReq, mp->ury));
    }
    else if(rDir == Direction::Down)
    {
        bg::append(area1.outer(), pt(mp->llx, y));
        bg::append(area1.outer(), pt(mp->urx, y));
        bg::append(area1.outer(), pt(mp->llx, y - lenReq));

        bg::append(area1.outer(), pt(mp->llx, y));
        bg::append(area1.outer(), pt(mp->urx, y));
        bg::append(area1.outer(), pt(mp->urx, y - lenReq));
    }
    else if(rDir == Direction::Up)
    {
        bg::append(area1.outer(), pt(mp->llx, y));
        bg::append(area1.outer(), pt(mp->urx, y));
        bg::append(area1.outer(), pt(mp->llx, y + lenReq));

        bg::append(area1.outer(), pt(mp->llx, y));
        bg::append(area1.outer(), pt(mp->urx, y));
        bg::append(area1.outer(), pt(mp->urx, y + lenReq));
    }
    else
    {
        return true;
    }
        
    if(!spacing_violations(busid, l, area1) || !spacing_violations(busid, l, area2))
        return true;
    else
        return false;    

}

bool OABusRouter::ObstacleRtree::bending_available_at_sink(int busid, int m, int x, int y, int l, int rDir)
{

    return true;
}

bool OABusRouter::ObstacleRtree::bending_available(int busid, int x, int y, int l1, int l2, int rDir)
{
    
    Bus* curbus = &ckt->buses[busid];
    int numBits = curbus->numBits;
    int lenReq1 = numBits * (rou->spacing[l1] + curbus->width[l1]);
    int lenReq2 = numBits * (rou->spacing[l2] + curbus->width[l2]);


    polygon area1, area2;
    if(rDir == Direction::Left)
    {
        bg::append(area1.outer(), pt(x, y));
        bg::append(area1.outer(), pt(x, y - lenReq2));
        bg::append(area1.outer(), pt(x - lenReq1, y - lenReq2));

        bg::append(area2.outer(), pt(x, y));
        bg::append(area2.outer(), pt(x, y + lenReq2));
        bg::append(area2.outer(), pt(x - lenReq1, y + lenReq2));

    }
    else if(rDir == Direction::Right)
    {
        bg::append(area1.outer(), pt(x, y));
        bg::append(area1.outer(), pt(x, y - lenReq2));
        bg::append(area1.outer(), pt(x + lenReq1, y - lenReq2));

        bg::append(area2.outer(), pt(x, y));
        bg::append(area2.outer(), pt(x, y + lenReq2));
        bg::append(area2.outer(), pt(x + lenReq1, y + lenReq2));

    }
    else if(rDir == Direction::Down)
    {
        bg::append(area1.outer(), pt(x, y));
        bg::append(area1.outer(), pt(x - lenReq2, y));
        bg::append(area1.outer(), pt(x - lenReq2, y - lenReq1));

        bg::append(area2.outer(), pt(x, y));
        bg::append(area2.outer(), pt(x + lenReq2, y));
        bg::append(area2.outer(), pt(x + lenReq2, y - lenReq1));
    }
    else if(rDir == Direction::Up)
    {
        bg::append(area1.outer(), pt(x, y));
        bg::append(area1.outer(), pt(x - lenReq2, y));
        bg::append(area1.outer(), pt(x - lenReq2, y + lenReq1));

        bg::append(area2.outer(), pt(x, y));
        bg::append(area2.outer(), pt(x + lenReq2, y));
        bg::append(area2.outer(), pt(x + lenReq2, y + lenReq1));
    }
    else
    {
        return true;
    }
        
    if(!spacing_violations(busid, l1, area1) || !spacing_violations(busid, l1, area2))
        return true;
    else
        return false;
}



bool OABusRouter::ObstacleRtree::compactness_check(int busid, int l, polygon& geo)
{
    return !spacing_violations(busid, l, geo); 
}

bool OABusRouter::ObstacleRtree::compactness(int numbits, int mx[], int my[], int x, int y, int l1, int l2, int align, int dir, int width, int spacing)
{
/*
    enum Direction
    {
        Left,
        Right,
        Up,
        Down
    };
*/
    int xs[2], ys[2];
    if(align == VERTICAL)
    {
        xs[0] = x;
        xs[1] = x;
        ys[0] = my[0];
        ys[1] = my[1];
        
        if(dir == Direction::Left)
            xs[0] -= numbits*(width + spacing);
        if(dir == Direction::Right)
            xs[1] += numbits*(width + spacing);
    }
    else
    {
        xs[0] = mx[0];
        xs[1] = mx[1];
        ys[0] = y;
        ys[1] = y;
        
        if(dir == Direction::Down)
            ys[0] -= numbits*(width + spacing);
        if(dir == Direction::Up)
            ys[1] += numbits*(width + spacing);       

    }
    
    return !spacing_violations_ndr(-1, xs, ys, l1) && !spacing_violations_ndr(-1, xs, ys, l2) ? true : false;
}


/*
void OABusRouter::Router::change_track(int wireid, int trackid)
{
    Wire* curw = &wires[wireid];
    curw->trackid = trackid;
    int offset = ckt->tracks[trackid].offset;

    curw->x[0] = curw->vertical ? offset : x[0];
    curw->x[1] = curw->vertical ? offset : x[1];
    curw->y[0] = curw->vertical ? y[0] : offset;
    curw->y[1] = curw->vertical ? y[1] : offset;
    rtree_t.insert_element(trackid, curw->x, curw->y, curw->l, true);

    for(auto& it : curw->intersection)
    {
        Wire* tarw = &wires[it.first];
        int x = it.second.first;
        int y = it.second.second;
    }
}
*/

int OABusRouter::Router::create_wire(int bitid, int trackid, int x[], int y[], int l, int seq, bool pin)
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

    // add into the bit
    ckt->bits[w.bitid].wires.push_back(w.id);
    // rtree update for track
    //rtree_t.insert_element(trackid, x, y, l, true);
    // rtree update for wire
    int xs[2], ys[2];
    xs[0] = (w.vertical) ? x[0] - (int)(1.0*w.width / 2) : x[0];
    xs[1] = (w.vertical) ? x[1] + (int)(1.0*w.width / 2) : x[1];
    ys[0] = (w.vertical) ? y[0] : y[0] - (int)(1.0*w.width / 2);
    ys[1] = (w.vertical) ? y[1] : y[1] + (int)(1.0*w.width / 2);
    // rtree update for obstacle
    rtree_o.insert_obstacle(bitid, xs, ys, l, false);
    assert (w.id != INT_MAX);
    return w.id;
}

bool OABusRouter::Router::set_neighbor(int w1, int w2, int x, int y)
{
    Wire* wire1 = &wires[w1];
    Wire* wire2 = &wires[w2];
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    //if(abs(wire1->l - wire2->l) > 1)
    //{
    //    cout << "Invalid set neighbor..." << endl;
    //    printf("current bus : %s\n", ckt->bits[wire1->bitid].name.c_str());
    //    printf("w%d (%d %d) (%d %d) M%d\n", wire1->id, wire1->x1, wire1->y1, wire1->x2, wire1->y2, wire1->l);
    //    printf("w%d (%d %d) (%d %d) M%d\n", wire2->id, wire2->x1, wire2->y1, wire2->x2, wire2->y2, wire2->l);
    //    exit(0);
    //}
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    
    wire1->neighbor.push_back(w2);
    wire2->neighbor.push_back(w1);
    wire1->intersection[w2] = {x,y};
    wire2->intersection[w1] = {x,y};
    return true;
}


void OABusRouter::TrackRtree::get_intersection(int t1, int t2, int &x, int &y)
{
    if(is_vertical_t(t1))
        x = get_offset_t(t1);
    else
        y = get_offset_t(t1);

    if(is_vertical_t(t2))
        x = get_offset_t(t2);
    else
        y = get_offset_t(t2);
}

bool OABusRouter::Router::get_intersection(int w1, int w2, int &x, int& y)
{
    typedef PointBG pt;
    typedef SegmentBG seg;
    Wire* wire1 = &wires[w1];
    Wire* wire2 = &wires[w2];
    vector<pt> intersection;
    seg s1(pt(wire1->x1, wire1->y1), pt(wire1->x2, wire1->y2));
    seg s2(pt(wire2->x1, wire2->y1), pt(wire2->x2, wire2->y2));

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

    /*
    if(x != wirex[0] && y != wirey[0])
    {
        cout << "illegal intersection pin..." << endl;
        printf("(%d %d) (%d %d) -> (%d %d)\n", wirex[0], wirey[0], wirex[1], wirey[1], x, y);
        exit(0);
    }
    if(x != wirex[1] && y != wirey[1])
    {
        cout << "illegal intersection pin..." << endl;
        printf("(%d %d) (%d %d) -> (%d %d)\n", wirex[0], wirey[0], wirex[1], wirey[1], x, y);
        exit(0);
    }
    */
}


