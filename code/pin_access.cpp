#include "circuit.h"
#include "route.h"
#include "func.h"

#include <queue>
#include <functional>

#define DEBUG

using namespace std;
using namespace OABusRouter;


bool OABusRouter::MultiPin::vertical_arrange()
{
    int minllx, minlly, maxurx, maxury;
    minllx = INT_MAX;
    minlly = INT_MAX;
    maxurx = INT_MIN;
    maxury = INT_MIN;
    for(auto& pinid : pins)
    {
        Pin* curpin = &ckt->pins[pinid];
        minllx = min(curpin->llx, minllx);
        minlly = min(curpin->lly, minlly);
        maxurx = max(curpin->urx, maxurx);
        maxury = max(curpin->ury, maxury);
    }

    return (maxury - minlly) > (maxurx - minllx);
}

void OABusRouter::Router::PinAccess(int busid)
{
    
    int DEPTH_COST = 1000;
    int VIA_COST = 5000;
    int i, j, wireid;
    int nummultipins, numwires, numpins;
    int numlayers, cost;
    int x1, y1, x2, y2;
    int llx, lly, urx, ury;
    int pinl, curl, tarl, l1, l2;
    int bitid, trackid, l, seq, width;
    bool vertical_arrange;
    bool vertical_segment;
    Bus* curbus;
    Bit* curbit;
    Pin* curpin;
    Wire* curwire, *targetwire;
    MultiPin* curmultipin;
    Track* curtrack;
    Segment* curseg;
    SegRtree* trackrtree;
    StTree* curtree;

    
    
    curtree = rsmt[rsmt.treeID[busid]];
    trackrtree = &rtree.track;

    //cout << "rtree size : " << trackrtree->size() << endl;



    curbus = &ckt->buses[busid];
    nummultipins = curbus->multipins.size();

    //printf("\n\n%s pin access\n", curbus->name.c_str());

    for(i=0; i < nummultipins; i++)
    {
        curmultipin = &ckt->multipins[curbus->multipins[i]];
        vertical_arrange = curmultipin->vertical_arrange();
        numpins = curmultipin->pins.size();

        int maxDepth;
        vector<int> tracel;
        for(j=0; j < numpins; j++)
        {
            curpin = &ckt->pins[curmultipin->pins[j]];
            targetwire = &wires[pin2wire[curmultipin->pins[j]]]; //curpin->id]];       
            tarl = targetwire->l;
            bitid = ckt->bitHashMap[curpin->bitName];
            curbit = &ckt->bits[bitid];
            
            llx = curpin->llx;
            lly = curpin->lly;
            urx = curpin->urx;
            ury = curpin->ury;
            pinl = curpin->l;
            //curl = curpin->l;


            typedef SegmentBG seg;
            typedef PointBG pt;
            typedef BoxBG box;
            typedef pair<int,int> ipair;
            typedef tuple<int,int,int> ituple;
            // { elem id, trace distance, target distance, depth }


            int e1, e2, t1, t2;
            int c1, c2; 
            //, depth;
            //
            int minElem = INT_MAX;
            int minCost = INT_MAX;
            bool hasMinElem = false;

            seg wireseg(pt(targetwire->x1, targetwire->y1), pt(targetwire->x2, targetwire->y2));
            vector<int> backtrace(rtree.elemindex, -1);
            vector<int> depth(rtree.elemindex, -1);
            vector<int> iterPtx(rtree.elemindex, -1);
            vector<int> iterPty(rtree.elemindex, -1);
            vector<seg> element(rtree.elemindex); 
            vector<pair<seg, int>> queries;
            


            auto cmp = [](const ipair &left, const ipair &right) { return left.second > right.second; };
            auto cmp2 = [](const ituple &left, const ituple &right){
                return (get<1>(left) + get<2>(left) > get<1>(right) + get<2>(right));
            };
            //priority_queue<ipair , vector<ipair>, decltype(cmp)> PQ(cmp);
            priority_queue<ituple , vector<ituple>, decltype(cmp2)> PQ(cmp2);
           


            box b(PointBG(llx,lly), PointBG(urx,ury));
            
            queries.clear();    
            trackrtree->query(bgi::intersects(b), back_inserter(queries));
            
            //cout << "queries size : " << queries.size() << endl;
            
            for(auto& it : queries)
            {
                
                e1 = it.second;
                t1 = rtree.trackID[e1];
                l1 = rtree.trackNuml[e1];
                //it.second];
                if(abs(pinl - l1) < 2) //pinl+1 >= l1 && l1 >= pinl-1)
                {
                    backtrace[e1] = e1;
                    element[e1] = it.first;
                    depth[e1] = 0;
                    
                    x1 = (int)(bg::get<0,0>(element[e1]) + 0.5);
                    y1 = (int)(bg::get<0,1>(element[e1]) + 0.5);
                    x2 = (int)(bg::get<1,0>(element[e1]) + 0.5);
                    y2 = (int)(bg::get<1,1>(element[e1]) + 0.5);
                    
                    if(ckt->is_vertical(l1))
                    {
                        if(y2 < lly)
                        {
                            iterPty[e1] = lly;
                            iterPtx[e1] = x1;
                        }
                        else //else if(y1 > ury)
                        {
                            iterPty[e1] = ury;
                            iterPtx[e1] = x1;
                        }
                    }
                    else
                    {
                        if(x2 < llx)
                        {
                            iterPty[e1] = y1;
                            iterPtx[e1] = llx;
                        }
                        else // else if(x1 > urx)
                        {
                            iterPty[e1] = y1;
                            iterPtx[e1] = urx;
                        }
                    }

                    c1 = VIA_COST* abs(pinl-l1);
                    c2 = (int)(bg::distance(wireseg, pt(iterPtx[e1],iterPty[e1])));
                    //cost = (int)bg::distance(wireseg, pt(iterPtx[e1], iterPty[e1])); //element[e1]);
                    if(abs(tarl-l1) < 2 && bg::intersects(element[e1], wireseg))
                    {
                        //element[e1])); // + VIA_COST * abs(tarl - l1));
                        if(hasMinElem)
                        {
                            if(minCost > c1 + c2)
                            {
                                minCost = c1 + c2;
                                minElem = e1;
                            }
                        }
                        else
                        {
                            hasMinElem = true;
                            minElem = e1;
                            minCost = c1 + c2;
                        }
                    }
                    //cost = (int)bg::distance(wireseg, b);
                    PQ.push(make_tuple(e1,c1,c2));   
                    //PQ.push(make_tuple(e1,0,0)); //cost));   
                }
            }



            bool arrival = false;
            while(PQ.size() > 0)
            {
                //pair<int,int> e = PQ.top();
                int cost1, cost2;
                ituple e = PQ.top();
                PQ.pop();
                e1 = get<0>(e);
                cost1 = get<1>(e);
                cost2 = get<2>(e);
                t1 = rtree.trackID[e1];
                l1 = rtree.trackNuml[e1];

                curtrack = &ckt->tracks[t1];
                x1 = curtrack->llx;
                y1 = curtrack->lly;
                x2 = curtrack->urx;
                y2 = curtrack->ury;
                l1 = curtrack->l;
                //if(bg::intersects(element[e1], wireseg) && (tarl-1 <= l1 && l1 <= tarl+1) )
                if(hasMinElem && (e1 == minElem))
                {
                    break;
                    /*
                    vector<pt> intersection;
                    bg::intersection(element[e1], wireseg, intersection);
                    pt p = intersection[0];
                    iterPtx[e1] =  (int)(bg::get<0>(p) + 0.5);
                    iterPty[e1] =  (int)(bg::get<1>(p) + 0.5);
                    cout << "Arrival" << endl;
                    cout << "x " << iterPtx[e1] << " y " << iterPty[e1] << endl;
                    cout << bg::dsv(element[e1]) << endl;
                    cout << bg::dsv(wireseg) << endl;
                    */
                    //break;
                }

                queries.clear();
                trackrtree->query(bgi::intersects(element[e1]), back_inserter(queries));
                for(auto& it : queries)
                {
                    e2 = it.second;
                    t2 = rtree.trackID[e2];
                    l2 = rtree.trackNuml[e2];

                    if(abs(l1 - l2) > 1) continue;
                    if(backtrace[e2] == -1)
                    {
                        element[e2] = it.first;
                        backtrace[e2] = e1;
                        depth[e2] = depth[e1] + 1;
                        
                        vector<pt> intersection;
                        intersection.clear();
                        bg::intersection(element[e1], element[e2], intersection);
                        if(intersection.size() == 0){
                            cout << bg::dsv(element[e1]) << endl;
                            cout << bg::dsv(element[e2]) << endl;
                            printf("???\n"); 

                        }
                        pt p = intersection[0];
                        int x = (int)(bg::get<0>(p) + 0.5);
                        int y = (int)(bg::get<1>(p) + 0.5);
                        iterPtx[e2] = x;
                        iterPty[e2] = y;
                        //int traceDist = abs(iterPtx[e1] - iterPtx[e2]) + abs(iterPty[e1] - iterPty[e2]);
                        //c1 = (traceDist == 0)? cost1 + traceDist : cost1 + traceDist + abs(l1-l2)*VIA_COST;
                        c1 = cost1 + abs(iterPtx[e1] - iterPtx[e2]) + abs(iterPty[e1] - iterPty[e2]) + abs(l1-l2)*VIA_COST;
                        c2 = (int)(bg::distance(p, wireseg));    
            //cout << "target wire : " << bg::dsv(wireseg) << endl;
            //cout << "current seg : " << bg::dsv(element[e2]) << endl;
                        if(bg::intersects(element[e2], wireseg) && abs(l2 - tarl) < 2)
                        {
                            //printf("Mincost %d Curcost %d\n", minCost, c1+c2);
                            if(hasMinElem)
                            {
                                if(minCost > c1+c2)
                                {
                                    minCost = c1 + c2;
                                    minElem = e2;
                                }
                                else
                                {
                                    continue;
                                }
                            }
                            else
                            {
                                hasMinElem = true;
                                minCost = c1 + c2;
                                minElem = e2;
                            }
                        }

                        PQ.push(make_tuple(e2, c1, c2));
                    }
                }
            }

            // backtrace start
            if(hasMinElem)
            {
                if(j==0)
                {
                    maxDepth = depth[e1];
                }
                
                
                cout << "arrival..." << endl;    
                e1 = minElem;
                l1 = tarl;
                l2 = rtree.trackNuml[e1];
                if(ckt->is_vertical(l1) == ckt->is_vertical(l2))
                {
                    cout << "both same direction..." << endl;
                    cout << bg::dsv(wireseg) << endl;
                    cout << bg::dsv(element[e1]) << endl;
                
                }
               
                vector<pt> intersection;
                bg::intersection(element[e1], wireseg, intersection);
                pt p = intersection[0];
                x1 =  (int)(bg::get<0>(p) + 0.5);
                y1 =  (int)(bg::get<1>(p) + 0.5);
                x2 = iterPtx[e1];
                y2 = iterPty[e1];
                l = rtree.trackNuml[e1];
                
                if(x1 > x2) swap(x1, x2);
                if(y1 > y2) swap(y1, y2);
                trackid = rtree.trackID[e1];
                seq = j;
                width = curbus->width[l];

                Wire w;
                w.id = wires.size();
                w.x1 = x1;
                w.y1 = y1;
                w.x2 = x2;
                w.y2 = y2;
                w.l = l;
                w.width = width;
                w.busid = busid;
                w.bitid = bitid;
                w.trackid = trackid;
                w.vertical = (rtree.trackDir[e1] == VERTICAL)? true : false;

                printf("Create wire (%d %d) (%d %d) M%d\n", x1, y1, x2, y2, l);
                wires.push_back(w);
                curbit->wires.push_back(w.id);

                interval.empty[trackid] -=
                    w.vertical ? IntervalT::closed(w.y1, w.y2) : IntervalT::closed(w.x1, w.x2);

                curtree->wires.push_back(w.id);
                ////////////////////////////////////////
                int lower, upper;
                lower = (w.vertical) ? (int)(bg::get<0,1>(element[e1]) + 0.5) : (int)(bg::get<0,0>(element[e1]) + 0.5);
                upper = (w.vertical) ? (int)(bg::get<1,1>(element[e1]) + 0.5) : (int)(bg::get<1,0>(element[e1]) + 0.5);

                IntervalSetT tmp = interval.empty[trackid] & IntervalT::open(lower, upper);
                trackrtree->remove({element[e1],e1});

                IntervalSetT::iterator it1 = tmp.begin();
                IntervalSetT::iterator it2 = tmp.end();
                while(it1 != it2)
                {
                    seg s1 = w.vertical ? 
                        seg(pt(x1, it1->lower()), pt(x1, it1->upper())) : seg(pt(it1->lower(), y1), pt(it1->upper(),y1));
                    trackrtree->insert({s1, rtree.elemindex});
                    rtree.trackNuml[rtree.elemindex] = l;
                    rtree.trackID[rtree.elemindex] = trackid;
                    rtree.trackDir[rtree.elemindex] = w.vertical ? VERTICAL : HORIZONTAL;
                    rtree.elemindex++;
                    it1++;
                }
                ////////////////////////////////////////               

                while(backtrace[e1] != e1)
                {
                    e2 = backtrace[e1];
                    x1 = iterPtx[e1];
                    y1 = iterPty[e1];
                    x2 = iterPtx[e2];
                    y2 = iterPty[e2];
                    l = rtree.trackNuml[e2];
                    if(x1 > x2) swap(x1, x2);
                    if(y1 > y2) swap(y1, y2);

                    if(x1 != x2 && y1 != y2)
                    {
                        cout << x1 << " " << x2 << endl;
                        cout << y1 << " " << y2 << endl;
                        //exit(0);
                    }
                    trackid = rtree.trackID[e1];
                    seq = j;
                    width = curbus->width[l];

                    Wire w;
                    w.id = wires.size();
                    w.x1 = x1;
                    w.y1 = y1;
                    w.x2 = x2;
                    w.y2 = y2;
                    w.l = l;
                    w.width = width;
                    w.busid = busid;
                    w.bitid = bitid;
                    w.trackid = trackid;
                    w.vertical = (rtree.trackDir[e1] == VERTICAL)? true : false;

                    printf("Create wire (%d %d) (%d %d) M%d\n", x1, y1, x2, y2, l);
                    wires.push_back(w);
                    curbit->wires.push_back(w.id);
                    curtree->wires.push_back(w.id);

                    interval.empty[trackid] -=
                        w.vertical ? IntervalT::closed(w.y1, w.y2) : IntervalT::closed(w.x1, w.x2);



                    ///////////////////////////////////////
                    lower = (w.vertical) ? (int)(bg::get<0,1>(element[e1]) + 0.5) : (int)(bg::get<0,0>(element[e1]) + 0.5);
                    upper = (w.vertical) ? (int)(bg::get<1,1>(element[e1]) + 0.5) : (int)(bg::get<1,0>(element[e1]) + 0.5);

                    tmp = interval.empty[trackid] & IntervalT::open(lower, upper);
                    trackrtree->remove({element[e1],e1});

                    it1 = tmp.begin();
                    it2 = tmp.end();
                    while(it1 != it2)
                    {
                        seg s1 = w.vertical ? 
                            seg(pt(x1, it1->lower()), pt(x1, it1->upper())) : seg(pt(it1->lower(), y1), pt(it1->upper(),y1));
                        trackrtree->insert({s1, rtree.elemindex});
                        rtree.trackNuml[rtree.elemindex] = l;
                        rtree.trackID[rtree.elemindex] = trackid;
                        rtree.trackDir[rtree.elemindex] = w.vertical ? VERTICAL : HORIZONTAL;
                        rtree.elemindex++;
                        it1++;
                    }                   
                    ////////////////////////////////////////

                    e1 = e2;

                }
                
            }else
            {
                cout << "No solution..." << endl;
            }
            // backtrace end
        }

    }

}








void Circuit::pin_access() {
    wire_track_mapping();
    pin_track_mapping();
    ContactMapping();
    for(int i=0; i < buses.size(); i++)
        pin_access(i);
    return;
}

void Circuit::pin_access(int busid) {
    Bus* bus = &buses[busid];
    for(int i=0; i < bus->multipins.size(); i++) {
        MultiPin* mp = &multipins[bus->multipins[i]];
        Segment* sg = &rou->segs[rou->multipin2seg[mp->id]];
        assert ( sg->wires.size() == mp->pins.size() );

        int dist_l = mp->l - sg->l;
        if( mp->needVia == true )
            if( mp->l == 0 )
                dist_l++;
            else
                dist_l--;
        cout << " dist l : " << dist_l << endl;

    }
    return;
}

bool Circuit::is_cross(Track* tr1, Track* tr2) {
    pair<int,int> a(tr1->llx,tr1->lly);
    pair<int,int> b(tr1->urx,tr1->ury);
    pair<int,int> c(tr2->llx,tr2->lly);
    pair<int,int> d(tr2->urx,tr2->ury);

    if( ccw(a,b,c)*ccw(a,b,d) <= 0 && ccw(c,d,a)*ccw(c,d,b) <= 0 )
        return true;
    else
        return false;
}

bool Circuit::is_intersect(Point _a, Point _b, Point _c, Point _d) {
    pair<int,int> a(_a.x,_a.y);
    pair<int,int> b(_b.x,_b.y);
    pair<int,int> c(_c.x,_c.y);
    pair<int,int> d(_d.x,_d.y);
    return is_intersect(make_pair(a,b),make_pair(c,d));
}

bool Circuit::is_intersect(Track* tr1, Track* tr2) {
    pair<int,int> a(tr1->llx,tr1->lly);
    pair<int,int> b(tr1->urx,tr1->ury);
    pair<int,int> c(tr2->llx,tr2->lly);
    pair<int,int> d(tr2->urx,tr2->ury);
    return is_intersect(make_pair(a,b),make_pair(c,d));
}

bool Circuit::is_intersect(pair<pair<int,int>,pair<int,int> > line1, pair<pair<int,int>,pair<int,int> > line2) {
    pair<int,int> a = line1.first;
    pair<int,int> b = line1.second;
    pair<int,int> c = line2.first;
    pair<int,int> d = line2.second;
    int ab = ccw(a,b,c)*ccw(a,b,d);
    int cd = ccw(c,d,a)*ccw(c,d,b);
    if( ab == 0 && cd == 0 ) {
        if( a > b ) swap(a,b);
        if( c > d ) swap(c,d);
        return c <= b && a <= d;
    }
    return ab <= 0 && cd <= 0;
}

void Circuit::wire_track_mapping() {
    for(int i=0; i < rou->wires.size(); i++) {
        Wire* theWire = &rou->wires[i];
        Track* theTrack = &tracks[theWire->trackid];
        theTrack->wires.push_back(theWire->trackid);
    }
    return;
}

void Circuit::pin_track_mapping() {
    int count = 0;
    for(int i=0; i < multipins.size(); i++) {
        MultiPin* MP = &multipins[i];
        Segment* Seg = &rou->segs[rou->multipin2seg[i]];
        for(int j = 0; j < MP->pins.size(); j++) {
            Pin* P = &this->pins[MP->pins[j]];
            int layer_num = P->l;
            if( MP->needVia == true ) {
                if( layer_num == 0 )
                    layer_num++;
                else
                    layer_num--; 
            }
            Layer* theLayer = &layers[layer_num];
            for(int k=0; k < theLayer->tracks.size(); k++) {
                Track* TR = &tracks[theLayer->tracks[k]];
                Point _a(P->llx,P->lly);
                Point _b(P->urx,P->ury);
                Point _c, _d;
                if( theLayer->is_vertical() ){
                    _c.x = TR->llx - TR->width/2;
                    _c.y = TR->lly;
                    _d.x = TR->urx + TR->width/2;
                    _d.y = TR->ury;
                } else {
                    _c.x = TR->llx;
                    _c.y = TR->lly - TR->width/2;
                    _d.x = TR->urx;
                    _d.y = TR->ury + TR->width/2;
                }
                if( rect_intersect(_a,_b,_c,_d) == true ) {
                    P->trackid = TR->id;
                    count++;
                    break; 
                }
            }
        }
    }
    cout << " num_pin : " << pins.size();
    cout << " count : " << count << endl;
    return;
}

void Circuit::ContactMapping() {

    for(int i=1; i < layers.size(); i++) {
        Layer* DownLayer = &layers[i-1];
        Layer* UpLayer = &layers[i];
        for(int j=0; j < UpLayer->tracks.size(); j++) {
            Track* UpTrack = &tracks[UpLayer->tracks[j]];
            Contact start;
            start.id = this->contacts.size();
            start.trackid = UpTrack->id;
            start.l = UpTrack->l;
            start.p.x = UpLayer->llx;
            start.p.y = UpLayer->lly;
            this->contacts.push_back(start);
            UpTrack->contacts.push_back(start.id);
            for(int k=0; k < DownLayer->tracks.size(); k++) {
                Track* DownTrack = &tracks[DownLayer->tracks[k]];
                if( is_cross(DownTrack,UpTrack) == true ) {
                    Contact con;
                    con.trackid = UpTrack->id;
                    con.l = UpTrack->l;
                    if( UpLayer->is_vertical() == true ) {
                        con.p.x = UpLayer->llx;
                        con.p.y = DownLayer->lly;
                    } else {
                        con.p.x = DownLayer->llx;
                        con.p.y = UpLayer->lly;
                    }
                }
            }
            Contact end;
            end.id = this->contacts.size();
            end.trackid = UpTrack->id;
            end.l = UpTrack->l;
            end.p.x = UpLayer->urx;
            end.p.y = UpLayer->ury;
            if( this->contacts[UpTrack->contacts.back()].p != end.p ) {
                this->contacts.push_back(end);
                UpTrack->contacts.push_back(end.id);
            }
        }
    }

    return;
}

void Circuit::debug() {
    Pin* P = &pins[90];
    Layer* L = &layers[P->l];
    for(int i=0; i < L->tracks.size(); i++) {
        Track* T = &tracks[L->tracks[i]];

        pair<int,int> a(P->llx,P->lly);
        pair<int,int> b(P->urx,P->ury);
        pair<int,int> c(T->llx,T->lly);
        pair<int,int> d(T->urx,T->ury);

        if( is_intersect(make_pair(a,b),make_pair(c,d)) == true ) {
            P->trackid = T->id;
            cout << "ab : " << ccw(a,b,c) * ccw(a,b,d) << endl;
            cout << "cd : " << ccw(c,d,a) * ccw(c,d,b) << endl;

            cout << " Found !!" << endl;
            cout << " a : " << a.first << " " << a.second << endl;
            cout << " b : " << b.first << " " << b.second << endl;
            cout << " c : " << c.first << " " << c.second << endl;
            cout << " d : " << d.first << " " << d.second << endl;
        }

    }


    exit(0);
    return;
}

bool Circuit::rect_intersect(Point A_ll, Point A_ur, Point B_ll, Point B_ur) {

    bool a = A_ll.x > B_ur.x;
    bool b = A_ur.x < B_ll.x;
    bool c = A_ur.y < B_ll.y;
    bool d = A_ll.y > B_ur.y;

    if( ( a || b || c || d ) == true )
        return false;
    else
        return true;
}


int Circuit::ccw(pair<int,int> a, pair<int,int> b, pair<int,int> c)
{
    double op = a.first/100.0*b.second/100.0 + b.first/100.0*c.second/100.0 + c.first/100.0*a.second/100.0;
    op -= (a.second/100.0*b.first/100.0 + b.second/100.0*c.first/100.0 + c.second/100.0*a.first/100.0);
    //cout << op << endl;
    
    if( op > 0 ) return 1;
    else if ( op == 0 ) return 0;
    else return -1;
}


