#include "circuit.h"
#include "route.h"
#include <algorithm>
#include <vector>
#include <iostream>

//#define GCELL_WIDTH rou->grid.GCELL_WIDTH
//#define GCELL_HEIGHT rou->grid.GCELL_HEIGHT
#define DEBUG_TRACK
//#define DEBUG_VIA
#define DEBUG_MAP

void OABusRouter::Router::RouteAll()
{
    for(int i=0; i < ckt->buses.size(); i++)
    {
        cout << "current busid : " << i << endl;
        PinAccess(i);
    }



}

void OABusRouter::Router::MappingMultipin2Seg()
{
    int i, j, x, y;
    
    int x1, y1, x2, y2, l1, l2;
    int treeid, busid, mpid, segid;
    int numSegs, numTrees, deg;
    int mplx, mply, mpux, mpuy;
    StTree* sttree;
    Segment* curseg;
    Pin* curpin;
    typedef BoxBG box;
    typedef PointBG pt;
    typedef SegmentBG seg;

    numTrees = rsmt.trees.size();
    


    for(i=0;  i < numTrees; i++)
    {
        sttree = rsmt[i];
        treeid = sttree->id;
        busid = rsmt.busID[treeid];
        vector<int> sorted;
        sorted = sttree->segs;


        /*
        for(j=0; j < numSegs; j++)
        {
            segid = sttree->segs[j];
            seg = &segs[segid];
            x1 = seg->x1;
            y1 = seg->y1;
            x2 = seg->x2;
            y2 = seg->y2;
            segrtree.insert(SegmentValT(SegmentBG(PointBG(x1,y1), PointBG(x2,y2)), segid));
            if(seg2bus[segid] != busid)
            {
                cout << "Invalid id mapping..." << endl;
                exit(0);
            }
        }
        */

        ///////
#ifdef DEBUG_MAP
        printf("%s -> {", ckt->buses[busid].name.c_str());
        for(j=0; j < deg; j++)
        {
            printf(" %d", sttree->node2multipin[j]);
        }
        printf(" }\n");
#endif

        for(auto& mpid : ckt->buses[busid].multipins)
        {
            mplx = INT_MAX;
            mply = INT_MAX;
            mpux = INT_MIN;
            mpuy = INT_MIN;
            for(auto& pinid : ckt->multipins[mpid].pins)
            {
                curpin = &ckt->pins[pinid];
                mplx = min(curpin->llx, mplx);
                mply = min(curpin->lly, mply);
                mpux = max(curpin->urx, mpux);
                mpuy = max(curpin->ury, mpuy);
            }

            typedef BoxBG box;
            typedef PointBG pt;
            int curl;
            box b1;
            b1 = box(pt(mplx, mply), pt(mpux, mpuy));
            curl = ckt->multipins[mpid].l;
            sort(sorted.begin(), sorted.end(), [&, this, b1, curl](int left, int right){
                    int x1, x2, y1, y2, l, gcellid1, gcellid2;
                    int dist1, dist2;
                    int VIA_COST = 1000;
                    box b2;
                    gcellid1 = this->grid.GetIndex(this->segs[left].x1, this->segs[left].y1, this->segs[left].l);
                    gcellid2 = this->grid.GetIndex(this->segs[left].x2, this->segs[left].y2, this->segs[left].l); 
                    x1 = this->grid.llx(gcellid1);
                    x2 = this->grid.urx(gcellid2);
                    y1 = this->grid.lly(gcellid1);
                    y2 = this->grid.ury(gcellid2);
                    l = this->segs[left].l;
                    b2 = box(pt(x1, y1), pt(x2, y2));
                    dist1 = (int)(bg::distance(b1,b2) +0.5) + VIA_COST*abs(curl-l);
#ifdef DEBUG_MAP
                    cout << "b1     " << bg::dsv(b1) << endl;
                    cout << "b2     " << bg::dsv(b2) << endl;
                    cout << "dist : " << dist1 << endl;
#endif
                    gcellid1 = this->grid.GetIndex(this->segs[right].x1, this->segs[right].y1, this->segs[right].l);
                    gcellid2 = this->grid.GetIndex(this->segs[right].x2, this->segs[right].y2, this->segs[right].l); 
                    x1 = this->grid.llx(gcellid1);
                    x2 = this->grid.urx(gcellid2);
                    y1 = this->grid.lly(gcellid1);
                    y2 = this->grid.ury(gcellid2);
                    l = this->segs[right].l;
                    b2 = box(pt(x1, y1), pt(x2, y2));
                    dist2 = (int)(bg::distance(b1,b2) + 0.5) + VIA_COST*abs(curl-l);
#ifdef DEBUG_MAP
                    cout << "b1     " << bg::dsv(b1) << endl;
                    cout << "b2     " << bg::dsv(b2) << endl;
                    cout << "dist : " << dist2 << endl;
                    cout << endl << endl;
#endif
                    return dist1 < dist2;
                    });

            segid = sorted[0];
            multipin2seg[mpid] = segid;
#ifdef DEBUG_MAP

            int gcellid1 = this->grid.GetIndex(this->segs[segid].x1, this->segs[segid].y1, this->segs[segid].l);
            int gcellid2 = this->grid.GetIndex(this->segs[segid].x2, this->segs[segid].y2, this->segs[segid].l); 
            x1 = this->grid.llx(gcellid1);
            x2 = this->grid.urx(gcellid2);
            y1 = this->grid.lly(gcellid1);
            y2 = this->grid.ury(gcellid2);
            l1 = this->segs[segid].l;
            printf("Current Multipin (%d %d) (%d %d) M%d\n", mplx, mply, mpux, mpuy, curl);
            printf("Mapped Segment (%d %d) (%d %d) M%d\n", x1, y1, x2, y2, l1);
            printf("\n\n");
#endif
        }
    }
}

void OABusRouter::Router::MappingPin2Wire()
{
    int i, j, mpid, segid, wireid, pinid;
    int numMultipins, numPins, numWires;
    numMultipins = ckt->multipins.size();
    MultiPin* curMP;
    Segment* curS;

    for(i=0; i < numMultipins; i++)
    {
        curMP = &ckt->multipins[i];
        mpid = curMP->id;
        segid = multipin2seg[mpid];

        curS = &segs[segid];
        numPins = curMP->pins.size();
        numWires = curS->wires.size();
     
        if(!curS->assign) continue;

        if(numWires != numPins)
        {
            cout << "different #wires #pins..." << endl;
            cout << "seg2bus: " << seg2bus[curS->id] << endl;
            cout << "mp2bus : " << curMP->busid << endl;
            cout << "numBits: " << ckt->buses[curMP->busid].numBits << endl;
            cout << "#bits  : " << ckt->buses[curMP->busid].bits.size() << endl;
            cout << "#Bw    : " << curS->bw << endl;
            cout << "#wires : " << numWires << endl;
            cout << "#pins  : " << numPins << endl;
            exit(0);
        }

        for(j=0; j < numWires; j++)
        {
            wireid = curS->wires[j];
            pinid = curMP->pins[j];
            pin2wire[pinid] = wireid;
        }
    }

}



void OABusRouter::Router::CreateVia()
{

    int numJuncs, numBW, numtrees, busid, bitid;
    int i, c, s1, s2, l1, l2, x, y;
    bool assign1, assign2, ver1, ver2, isLL1, isUR1, isLL2, isUR2;
    Junction *curJ;
    Segment *seg1, *seg2;
    Wire *wire1, *wire2;

    // for intersection function
    //vector<PointBG> intersection;
    SegmentBG tmp1, tmp2;
    //PointBG pt;

    
    numJuncs = junctions.size();
#ifdef DEBUG_VIA
    cout << "# Junctions : " << numJuncs << endl;
#endif
    
    numtrees = rsmt.trees.size();
    StTree* curtree;
    for(i=0; i < numtrees; i++)
    {
        curtree = rsmt[i];

        for(auto& juncid : curtree->junctions)
        {
            curJ = &junctions[juncid];
            busid = junc2bus[curJ->id];
            s1 = curJ->s1;
            s2 = curJ->s2;
            l1 = curJ->l1;
            l2 = curJ->l2;
            seg1 = &segs[s1];
            seg2 = &segs[s2];
            assign1 = seg1->assign; //assign[s1];
            assign2 = seg2->assign; // assign[s2];
            numBW = curJ->bw;
            ver1 = seg1->vertical;
            ver2 = seg2->vertical;



#ifdef DEBUG_VIA
            if(seg1->assign) printf("s%d is assigned\n", s1);
            if(seg2->assign) printf("s%d is assigned\n", s2);

            printf("Segment(1) id %d (%d %d) (%d %d) M%d\n",s1,  seg1->x1, seg1->y1, seg1->x2, seg1->y2, seg1->l);
            printf("Segment(2) id %d (%d %d) (%d %d) M%d\n",s2,  seg2->x1, seg2->y1, seg2->x2, seg2->y2, seg2->l);

#endif

            isLL1 = ((seg1->x1 == curJ->x) && (seg1->y1 == curJ->y))? true : false;
            isUR1 = ((seg1->x2 == curJ->x) && (seg1->y2 == curJ->y))? true : false;
            isLL2 = ((seg2->x1 == curJ->x) && (seg2->y1 == curJ->y))? true : false;
            isUR2 = ((seg2->x2 == curJ->x) && (seg2->y2 == curJ->y))? true : false;




            // Valid if both segments are assigned
            if(assign1 && assign2)
            {
                for(c=0; c<numBW; c++)
                {
                    wire1 = &wires[seg1->wires[c]];
                    wire2 = &wires[seg2->wires[c]];
                    bitid = wire1->bitid;
                    if(wire1->bitid != wire2->bitid)
                    {
                        cout << "different bitid..." << endl;
                        exit(0); 
                    }


                    // via coordinate
                    if(ver1 && !ver2)
                    {
                        x = wire1->x1;
                        y = wire2->y1;
                    }
                    else if(!ver1 && ver2)
                    {
                        x = wire2->x1;
                        y = wire1->y1;
                    }
                    else if(ver1 && ver2)
                    {

                        DiscreteIntervalT i1 = DiscreteIntervalT::closed(wire1->y1, wire1->y2);
                        DiscreteIntervalT i2 = DiscreteIntervalT::closed(wire2->y1, wire2->y2);
                        DiscreteIntervalT overlap = i1 & i2;
                        cout << i1 << endl;
                        cout << i2 << endl;
                        cout << overlap << endl;
                    }
                    else if(!ver1 && !ver2)
                    {
                        DiscreteIntervalT i1 = DiscreteIntervalT::closed(wire1->x1, wire1->x2);
                        DiscreteIntervalT i2 = DiscreteIntervalT::closed(wire2->x1, wire2->x2);
                        DiscreteIntervalT overlap = i1 & i2;
                        cout << i1 << endl;
                        cout << i2 << endl;
                        cout << overlap << endl;
                    }

                    tmp1 = SegmentBG(PointBG(wire1->x1, wire1->y1), PointBG(wire1->x2, wire1->y2));
                    tmp2 = SegmentBG(PointBG(wire2->x1, wire2->y1), PointBG(wire2->x2, wire2->y2));

                    if(!bg::intersects(tmp1, tmp2))
                    {
#ifdef DEBUG_VIA
                        cout << "No Intersection point..." << endl;
#endif
                        //exit(0);
                    }



                    //pt = *intersection.begin();
                    //x = (int)(bg::get<0>(intersection[0]) + 0.5);
                    //x = (int)(bg::get<0>(pt) + 0.5);
                    //y = (int)(bg::get<1>(intersection[0]) + 0.5);
                    //y = (int)(bg::get<1>(pt) + 0.5);
#ifdef DEBUG_VIA
                    cout << "(1) " << bg::dsv(tmp1) << endl;
                    cout << "(2) " << bg::dsv(tmp2) << endl;
                    printf("x = %4d\ny = %4d\n", x, y);
#endif



                    for(int l=l1; l < l2; l++)
                    {
                        Via via;
                        via.id = vias.size();
                        via.bitid = bitid;
                        via.busid = busid;
                        via.x = x;
                        via.y = y;
                        via.l = l;
                        vias.push_back(via);
                        via2bus[via.id] = busid;
                    
                   
                        int _x[2] = { x, x };
                        int _y[2] = { y, y };
                        rtree.insert_element(wire1->trackid, _x, _y, wire1->l, true);
                        curtree->vias.push_back(via.id);
                    
                    }

                    // cut
                    if(isLL1)
                    {
                        int _x[2] = { wire1->x1, x };
                        int _y[2] = { wire1->y1, y };
                        rtree.insert_element(wire1->trackid, _x, _y, wire1->l, false);
                        wire1->x1 = x;
                        wire1->y1 = y;
                    }

                    if(isUR1)
                    {
                        
                        int _x[2] = { x, wire1->x2 };
                        int _y[2] = { y, wire1->y2 };
                        rtree.insert_element(wire1->trackid, _x, _y, wire1->l, false);
                        wire1->x2 = x;
                        wire1->y2 = y;
                    }

                    if(isLL2)
                    {
                        int _x[2] = { wire2->x1, x };
                        int _y[2] = { wire2->y1, y };
                        rtree.insert_element(wire2->trackid, _x, _y, wire2->l, false);
                        wire2->x1 = x;
                        wire2->y1 = y;
                    }

                    if(isUR2)
                    {
                        int _x[2] = { x, wire2->x2 };
                        int _y[2] = { y, wire2->y2 };
                        rtree.insert_element(wire2->trackid, _x, _y, wire2->l, false);
                        wire2->x2 = x;
                        wire2->y2 = y;
                    }



                }
            }else{
                printf("[Invalid] s%d and s%d not assigned....\n", s1, s2);



            }
        }
        // 
    }



}


/*
void OABusRouter::Router::SpacingAwareTrackAssign()
{
    
    int bw;
    StTree* curtree;
    SegRtree* trackrtree;




}
*/


void OABusRouter::Router::TrackAssign()
{
    //this->grid.print();
    //exit(0);
    
    int numLayers, numRows, numCols, numSegs, numtrees;
    int i, j, k, row, col, curl, dir, GCidx, bw;
    int x1,y1, x2, y2, segid, busid, bitid, trackid, width, seq;
    int lx, ly, ux, uy, l;
    int g1, g2;
    int GCx1, GCy1, GCx2, GCy2;
    int curBW, tarBW, start, end;
    int xs[2], ys[2];
    //int MAXSIZE=100;
    //int GCELL_WIDTH, GCELL_HEIGHT; 
    int absx1, absx2, absy1, absy2;
    bool isVertical, isValid;
    bool ver1, ver2;
    
    typedef SegmentBG seg;
    typedef PointBG pt;
    typedef BoxBG box;
    Wire* curwire;
    BoxBG qbox;
    Segment* curS;
    Bus* curBus;
    Bit* curBit;
    Gcell* curGC;
    StTree* curtree;
    numSegs = segs.size();


    //GCELL_WIDTH = grid.GCELL_WIDTH;
    //GCELL_HEIGHT = grid.GCELL_HEIGHT;
    numtrees = rsmt.trees.size();
    //numSegs = this->segs.size();
 
    SegRtree* trackrtree;
    trackrtree = &rtree.track;
    for(i=0; i < numtrees; i++)
    {
        curtree = rsmt[i];
        for(auto& segid : curtree->segs)
        {
            curS = &segs[segid];
            busid = seg2bus[segid];
            curl = curS->l;
            x1 = min(curS->x1, curS->x2);
            y1 = min(curS->y1, curS->y2);
            x2 = max(curS->x1, curS->x2);
            y2 = max(curS->y1, curS->y2);
            tarBW = curS->bw;
            isVertical = curS->vertical;
            //this->bitwidth[curS->id];

            // Expand Segment window
            if(isVertical)
            {
                if(x2+1 < grid.numCols)
                {
                    x2 += 1;
                }
                else if(x1-1 > 0)
                {
                    x1 -= 1;
                }
            }
            else
            {
                if(y2+1 < grid.numRows)
                {
                    y2 += 1;
                }
                else if(y1-1 > 0)
                {
                    y1 -= 1;
                }
            }
            
            g1 = grid.GetIndex(x1, y1, curl);
            g2 = grid.GetIndex(x2, y2, curl);

            lx = grid.llx(g1);
            ly = grid.lly(g1);
            ux = grid.urx(g2);
            uy = grid.ury(g2);
            
            vector<pair<seg, int>> queries;
            box b(pt(lx, ly), pt(ux, uy));
            trackrtree->query(bgi::intersects(b), back_inserter(queries));
            /*
            trackrtree->query(bgi::intersects(b) && bgi::satisfies(
                        [&,this,curl,b](const pair<seg, int>& val){
                        bool falling, target;
                        pt ll(bg::get<0,0>(val.first), bg::get<0,1>(val.first));
                        pt ur(bg::get<1,0>(val.first), bg::get<1,1>(val.first));
                        target = (this->rtree.layer(val.second) == curl) ? true : false;
                        falling = (bg::within(ll, b) || bg::within(ur, b)) ? true : false; 
                        return target && !falling;
                        }), back_inserter(queries));
            */
            
            vector<int> availables;
            for(auto& it : queries)
            {
                
                bool falling, target;
                pt ll(bg::get<0,0>(it.first), bg::get<0,1>(it.first));
                pt ur(bg::get<1,0>(it.first), bg::get<1,1>(it.first));
                falling = (bg::within(ll, b) || bg::within(ur,b)) ? true : false;
                target = (rtree.layer(it.second) == curl) ? true : false;
                cout << "Box " << bg::dsv(b) << endl;
                cout << "Pt1 " << bg::dsv(ll) << endl;
                cout << "pt2 " << bg::dsv(ur) << endl;
                if(!falling && target)
                {
                    cout << "Not Falling...!!" << endl;
                    
                    availables.push_back(rtree.trackid(it.second));
                }else{
                    cout << "Falling...!!" << endl;

                }
            }

            sort(availables.begin(), availables.end());
            int it1, it2;
            vector<int> tmp;
            it1 = 0;
            it2 = 1;
            tmp.push_back(availables[0]);
           

            while(it2 < availables.size())
            {
                int offset1 = rtree.track_offset(tmp[it1]);
                int offset2 = rtree.track_offset(availables[it2]);
                int width = ckt->buses[i].width[curl];           
                if(abs(offset1 - offset2) > spacing[curl])
                //if(abs(offset1 - offset2) - width > spacing[curl])
                {
                    tmp.push_back(availables[it2]);
                    it1++;
                }
                it2++;
            }

            availables = tmp;


 
            isValid = (availables.size() >= tarBW) ? true :false;

            // Create wire
            if(isValid)
            {
                //sort(availables.begin(), availables.end());

                // Assign track for all bits
                //assign[segid] = true;
                curBus = &ckt->buses[busid];
                vector<int>::iterator it = availables.begin();
                for(int s=0; s < tarBW; s++)
                {
                    seq = s;
                    width = curBus->width[curl];
                    bitid = curBus->bits[s];
                    trackid = *it++;
                    xs[0] = isVertical? rtree.track_offset(trackid) : grid.llx(g1);
                    xs[1] = isVertical? rtree.track_offset(trackid) : grid.urx(g2);
                    ys[0] = isVertical? grid.lly(g1) : rtree.track_offset(trackid);
                    ys[1] = isVertical? grid.ury(g2) : rtree.track_offset(trackid);
                    curwire = CreateWire(bitid, trackid, xs, ys, curl, seq, false);
                    //
                    curS->wires.push_back(curwire->id);

                   
                }


            }else{
                cout << "Invalid..." << endl;
                //exit(0);


            }
            ///
        }

        ///////////
        for(auto& juncid : curtree->junctions)
        {
            Segment *s1, *s2;
            Wire *w1, *w2, *w_it1, *w_it2;
            int l1, l2;

            s1 = &segs[junctions[juncid].s1];
            s2 = &segs[junctions[juncid].s2];
            l1 = s1->l;
            l2 = s2->l;

            for(j=0; j < s1->bw; j++)
            {
                w1 = &wires[s1->wires[j]];
                w2 = &wires[s2->wires[j]];
                int x, y;
                if(!Intersection(w1, w2, x, y))
                {
                    cout << "No intersection...!" << endl;
                    
                    printf("wire1 (%d %d) (%d %d)\n", w1->x1, w1->y1, w1->x2, w1->y2);
                    printf("wire2 (%d %d) (%d %d)\n", w2->x1, w2->y1, w2->x2, w2->y2);
                    

                    if(w1->vertical && !w2->vertical)
                    {
                        x = rtree.track_offset(w1->trackid);
                        y = rtree.track_offset(w2->trackid);
                    }
                    else if(!w1->vertical && w2->vertical)
                    {
                        x = rtree.track_offset(w2->trackid);
                        y = rtree.track_offset(w1->trackid);
                    }
                    else
                    {
                        //printf("Invalid pair of tracks...\n");
                        //exit(0);
                    }

                    int xs1[2], xs2[2], ys1[2], ys2[2];
                    xs1[0] = min(w1->x1, x);
                    xs1[1] = max(w1->x2, x);
                    ys1[0] = min(w1->y1, y);
                    ys1[1] = max(w1->y2, y);
                    xs2[0] = min(w2->x1, x);
                    xs2[1] = max(w2->x2, x);
                    ys2[0] = min(w2->y1, y);
                    ys2[1] = max(w2->y2, y);
                    if(ValidUpdate(w1->id, xs1, ys1) && ValidUpdate(w2->id, xs2, ys2))
                    {
                        UpdateWire(w1->id, xs1, ys1);
                        UpdateWire(w2->id, xs2, ys2);
                    }
                    else
                    {
                        cout << "Invalid updated..." << endl;
                        exit(0);
                    }
                }


                if(abs(w1->l - w2->l) > 1)
                {
                    w_it1 = w1;       
                    for(l=l1+1; l < l2; l++)
                    {

                        // find trackid
                        vector<pair<seg,int>> queries;
                        trackrtree->query(bgi::intersects(pt(x,y)) && bgi::satisfies([&,l,this](const pair<seg,int>& val){
                                return (l == this->rtree.layer(val.second)) ? true : false;
                                    }), back_inserter(queries));
                        if(queries.size() ==0)
                        {
                            cout << "Invalid.. no remained tracks...." << endl;
                            exit(0);
                        }
                        
                        trackid = rtree.trackid(queries[0].second);
                        bitid = w_it1->bitid;
                        xs[0] = x;
                        xs[1] = x;
                        ys[0] = y;
                        ys[1] = y;
                        w_it2 = CreateWire(bitid, trackid, xs, ys, l, j, false);

                        SetNeighbor(w_it1, w_it2, x, y);

                        
                        w_it1 = w_it2;
                    }

                    SetNeighbor(w_it1, w2, x, y);

                }
                else
                {
                    SetNeighbor(w1, w2, x, y);
                }
            }
        }

    }
    
#ifdef DEBUG_TRACK
    Wire* curW;
    
    printf("\n\n======================================================\n\n");
    for(i=0; i < wires.size(); i++)
    {
        curW = &wires[i];
        printf("Create wire(%-4d) (%4d %-4d) (%4d %-4d) M%d --> %s\n",i,curW->x1, curW->y1, curW->x2, curW->y2, curW->l, ckt->buses[curW->busid].name.c_str());  
    }

    printf("\n\n======================================================\n\n\n\n");
#endif

}



void OABusRouter::Grid3D::print()
{
    int row, col, layer, cap;
    int idx, numGCs;
    Gcell* curGC;

    numGCs = numRows* numCols* numLayers;

    for(layer = 0; layer < numLayers; layer++)
    {
        printf("layer M%2d\n", layer);
        for(row =0; row < numRows; row++)
        {
            for(col = 0; col < numCols; col++)
            {
                printf("___");
            }
            printf("\n");
            
            for(col = 0; col < numCols; col++)
            {

                
                idx = GetIndex(col, row, layer);
                curGC = &gcells[idx];
                cap = curGC->cap;
                printf("|%2d", cap);

                
                
                //printf("Current index(%3d) (%3d %3d %3d) Cap %d\n", idx, col, row, layer, cap);
                //cout << "   -> ";
                //for(auto& it : curGC->resources)
                //{
                //    cout << it << " ";
                //}
                //cout << endl << endl;
            }
            printf("|\n");
            
            


        }
        
        for(col = 0; col < numCols; col++)
        {
            printf("___");
        }
        printf("\n");

        
        printf("\n\n");
        //printf("\n\n");
    }

    
}


    /*
     *
            dir = this->grid.direction[curl];
            isVertical = (dir == VERTICAL)?true:false;

            // Initial resources
            col = x1;
            row = y1;
            GCidx = this->grid.GetIndex(col,row,curl);
            set1 = this->grid[GCidx]->resources;

            start = x1;
            end = x2;

            if(isVertical)
            {
                start = y1;
                end = y2;
            }

            isValid = true;

#ifdef DEBUG_TRACK
            if(isVertical)
                printf("Segment (%d %d) (%d %d) M%d VERTICAL\n", x1, y1, x2, y2, curl);
            else
                printf("Segment (%d %d) (%d %d) M%d HORIZONTAL\n", x1, y1, x2, y2, curl);
            printf("Start %d End %d\n", start, end);
#endif

            while(start <= end)
            {
                GCidx = (isVertical)? this->grid.GetIndex(col,start,curl) : this->grid.GetIndex(start,row,curl);
                set2 = this->grid[GCidx]->resources;
                //it = set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), v3.begin());
                set_intersection(set1.begin(), set1.end(), set2.begin(), set2.end(), inserter(set3, set3.begin()));
                curBW = set3.size();

#ifdef DEBUG_TRACK
                if(isVertical)
                    printf("Current Gcell (%d %d) M%d Cap %d\n", col, start, curl, set2.size());
                else
                    printf("Current Gcell (%d %d) M%d Cap %d\n", start, row, curl, set2.size());
                printf("\n\nSet1 -> ");
                for(auto& iter : set1) cout << iter << " ";
                printf("\n\nSet2 -> ");
                for(auto& iter : set2) cout << iter << " ";
                printf("\n\nSet3 -> ");
                for(auto& iter : set3) cout << iter << " ";
                printf("\n\n\nset1 %d set2 %d set3 %d\n\n", set1.size(), set2.size(), set3.size());
#endif


                if(curBW < tarBW)
                {
                    printf("Bitwidth (%d %d)\n", curBW, tarBW);
                    cout << "Fail to assign track..." << endl;
                    isValid = false;
                    //
                    set1 = set3;
                    break;
                }
                else
                {
                    set1 = set3;
                    set3.clear(); // = vector<int>(MAXSIZE);
                }



                start++;
            }

    for(i=0; i < numSegs; i++)
    {
        queries.clear();

        curS = &segs[i];
        curl = curS->l;
        GCx1 = min(curS->x1, curS->x2);
        GCy1 = min(curS->y1, curS->y2);
        GCx2 = max(curS->x1, curS->x2);
        GCy2 = max(curS->y1, curS->y2);
        
        x1 = grid.GetOffset_x(GCx1);
        y1 = grid.GetOffset_y(GCy1);
        x2 = grid.GetOffset_x(GCx2) + grid.GCELL_WIDTH;
        y2 = grid.GetOffset_y(GCy2) + grid.GCELL_HEIGHT;


        qbox = BoxBG(PointBG(x1,y1), PointBG(x2, y2));
        curRtree->query(bgi::intersects(qbox) && 
                bgi::satisfies([&, curl, qbox, this](const SegmentValT& val){
                        int llx, lly, urx, ury;
                        int tarl;
                        int idx;
                        bool fallLL, fallUR, targetL;
                        idx = val.second;
                        SegmentBG curSeg = val.first;
                        tarl = this->rtree.trackNuml[idx];
                        llx = bg::get<0,0>(curSeg);
                        lly = bg::get<0,1>(curSeg);
                        urx = bg::get<1,0>(curSeg);
                        ury = bg::get<1,1>(curSeg);
                        fallLL = (bg::within(PointBG(llx,lly),qbox))?true:false;
                        fallUR = (bg::within(PointBG(urx,ury),qbox))?true:false;
                        targetL = (curl == tarl)?true:false;
                        return (!fallLL && !fallUR && targetL);
                    }), back_inserter(queries)); 

        tarBW = bitwidth[curS->id];
        curBW = queries.size();

        if(curBW < tarBW)
        {
            cout << "Cur Bitwidth : " << curBW << endl;
            cout << "Tar Bitwidth : " << tarBW << endl;
        }else{
            
            Layer* curL = &ckt->layers[curl];
            isVertical = (curL->direction == VERTICAL)?true:false;
            if(isVertical)
            {
                printf("Segment (%d %d) (%d %d) M%d VERTICAL\n", x1, y1, x2, y2, curl);
            }
            else
            {
                printf("Segment (%d %d) (%d %d) M%d HORIZONTAL\n", x1, y1, x2, y2, curl);
            }

            
            for(auto& iter : queries)
            {
                int tmpx1, tmpx2, tmpy1, tmpy2, tmpl;
                Track* curT = &ckt->tracks[iter.second];
                tmpx1 = curT->llx;
                tmpx2 = curT->urx;
                tmpy1 = curT->lly;
                tmpy2 = curT->ury;
                tmpl = ckt->layerHashMap[curT->layer];
                
                printf("Track %d (%d %d) (%d %d) M%d\n", curT->id, tmpx1, tmpy1, tmpx2, tmpy2, tmpl);
            }
        }
    }
    */


    /*

    cout << "4" << endl;
*/





