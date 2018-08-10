#include "circuit.h"
#include "route.h"

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
    int xs[2], ys[2];
    bool pin;
    bool vertical_arrange;
    bool vertical_segment;
    typedef SegmentBG seg;
    typedef PointBG pt;
    typedef BoxBG box;
    typedef pair<int,int> ipair;
    typedef tuple<int,int,int> ituple;
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

    printf("\n\n%s pin access\n", curbus->name.c_str());

    for(i=0; i < nummultipins; i++)
    {
        curmultipin = &ckt->multipins[curbus->multipins[i]];
        vertical_arrange = curmultipin->vertical_arrange();
        numpins = curmultipin->pins.size();
        int maxDepth = INT_MAX;
        vector<int> tracel;
        set<int> used;
        vertical_arrange = (curmultipin->align == VERTICAL)? true : false;

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
                t1 = rtree.trackid(e1); //[e1];
                l1 = rtree.layer(e1); //];
                //it.second];
                if(vertical_arrange == ckt->is_vertical(l1)) continue;
                
                // condition 2
                if(used.find(t1) != used.end()) continue;
                
                if(abs(pinl - l1) < 2) //pinl+1 >= l1 && l1 >= pinl-1)
                {
                    backtrace[e1] = e1;
                    element[e1] = it.first;
                    depth[e1] = 0;
                    
                    x1 = (int)(bg::get<0,0>(element[e1]) + 0.5);
                    y1 = (int)(bg::get<0,1>(element[e1]) + 0.5);
                    x2 = (int)(bg::get<1,0>(element[e1]) + 0.5);
                    y2 = (int)(bg::get<1,1>(element[e1]) + 0.5);
                   
                    if(pinl != l1)
                    {
                        for(auto& it2 : queries)
                        {
                            e2 = it2.second;
                            if(rtree.layer(e2) == pinl)
                            {
                                if(rtree.direction(e1) == VERTICAL)
                                {
                                    iterPtx[e1] = rtree.offset(e1);
                                    iterPty[e1] = rtree.offset(e2);
                                }
                                else
                                {
                                    iterPtx[e1] = rtree.offset(e2);
                                    iterPty[e1] = rtree.offset(e1);
                                }
                                break;
                            }
                        }
                    }else{
                        if(bg::intersects(pt(x1,y1), b))
                        {
                            iterPtx[e1] = x1;
                            iterPty[e1] = y1;
                        }
                        else if(bg::intersects(pt(x2,y2), b))
                        {
                            iterPtx[e1] = x2;
                            iterPty[e1] = y2;
                        }
                        else
                        {   
                            printf("Invalid segment...\n");
                            exit(0);
                        }
                    }
                    /*
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
                    */
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
                t1 = rtree.trackid(e1); //[e1];
                l1 = rtree.layer(e1); //l[e1];

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
                //
                if(maxDepth <= depth[e1])
                {
                    continue;      
                }


                queries.clear();
                trackrtree->query(bgi::intersects(element[e1]), back_inserter(queries));
                
                
                for(auto& it : queries)
                {
                    e2 = it.second;
                    t2 = rtree.trackid(e2); //ID[e2];
                    l2 = rtree.layer(e2);

    
                    // condition 1
                    if(abs(l1 - l2) > 1) continue;
                    // condition 2
                    if(used.find(t2) != used.end()) continue;
                    
                    // condition 3
                    if(backtrace[e2] == -1)
                    {
                        element[e2] = it.first;
                        backtrace[e2] = e1;
                        depth[e2] = depth[e1] + 1;

                        if(j!=0 && tracel[depth[e2]] != l2) continue;
                        
                        vector<pt> intersection;
                        intersection.clear();
                        bg::intersection(element[e1], element[e2], intersection);
                        ///////////////////////////////////////////////////////
                        if(intersection.size() == 0){
                            cout << bg::dsv(element[e1]) << endl;
                            cout << bg::dsv(element[e2]) << endl;
                            printf("???\n"); 

                        }
                        ///////////////////////////////////////////////////////
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
                int w1, w2;
                
                if(j==0)
                {
                    maxDepth = depth[e1] + 1;
                    tracel.insert(tracel.begin(), targetwire->l);
                }
                
                
                cout << "arrival..." << endl;    
                e1 = minElem;
                l1 = tarl;
                l2 = rtree.layer(e1); //trackNuml[e1];
                
                
                if(l1 == l2)
                {
                    targetwire->x1 = min(iterPtx[e1], targetwire->x1);
                    targetwire->x2 = max(iterPtx[e1], targetwire->x2);
                    targetwire->y1 = min(iterPty[e1], targetwire->y1);
                    targetwire->y2 = max(iterPty[e1], targetwire->y2);
                    l = targetwire->l;
                    
                    if(j==0)
                        tracel.insert(tracel.begin(), l);
                    
                    
                    pin = (e1 == backtrace[e1]) ? true : false;
                    targetwire->pin = (e1 == backtrace[e1]) ? true : false;
                    xs[0] = targetwire->x1;
                    xs[1] = targetwire->x2;
                    ys[0] = targetwire->y1;
                    ys[1] = targetwire->y2;
                    rtree.insert_element(targetwire->trackid, xs, ys, targetwire->l, true);

                    w1 = targetwire->id;
                    //
                    used.insert(targetwire->trackid);

                    if(targetwire->pin)
                    {
                        pin2wire[curpin->id] = w1;
                        wires[w1].intersection[PINTYPE] = {iterPtx[e1], iterPty[e1]};
                    }
                    
                    //if(j==0)
                    //    maxDepth++;
                }
                else
                {
                    vector<pt> intersection;
                    bg::intersection(element[e1], wireseg, intersection);
                    pt p = intersection[0];
                    x1 =  (int)(bg::get<0>(p) + 0.5);
                    y1 =  (int)(bg::get<1>(p) + 0.5);
                    x2 = iterPtx[e1];
                    y2 = iterPty[e1];
                    l = rtree.layer(e1); //trackNuml[e1];
                    //
                    if(j==0) 
                        tracel.insert(tracel.begin(), l);

                    // Data setting
                    xs[0] = min(x1,x2);
                    xs[1] = max(x1,x2);
                    ys[0] = min(y1,y2);
                    ys[1] = max(y1,y2);
                    trackid = rtree.trackid(e1); //]
                    seq = j;
                    pin = (e1 == backtrace[e1]) ? true : false;

                    w1 = targetwire->id;
                    w2 = CreateWire(bitid, trackid, xs, ys, l, seq, pin)->id;

                    //
                    used.insert(trackid);
                    
                    if(pin)
                    {
                        pin2wire[curpin->id] = w2;
                        wires[w2].intersection[PINTYPE] = {x2, y2};
                    }

                    //w2 = w.id;
                    SetNeighbor(&wires[w1], &wires[w2], x1, y1);

                    w1 = w2;                
                }
                

                // Backtrace
                while(backtrace[e1] != e1)
                {
                    // Iterating Points
                    e2 = backtrace[e1];
                    x1 = iterPtx[e1];
                    y1 = iterPty[e1];
                    x2 = iterPtx[e2];
                    y2 = iterPty[e2];
                    l = rtree.layer(e2); 
                    // if first bit trace all layer sequences.
                    if(j==0) 
                        tracel.insert(tracel.begin(), l);
                    //

#ifdef DEBUG_PIN
                    if(x1 != x2 && y1 != y2)
                    {
                        cout << x1 << " " << x2 << endl;
                        cout << y1 << " " << y2 << endl;
                        //exit(0);
                    }
#endif    

                    // Data setting                    
                    xs[0] = min(x1,x2);
                    xs[1] = max(x1,x2);
                    ys[0] = min(y1,y2);
                    ys[1] = max(y1,y2);
                    pin = (e2 == backtrace[e2])? true : false;
                    trackid = rtree.trackid(e2); //[e1];
                    seq = j;
                    
                    w2 = CreateWire(bitid, trackid, xs, ys, l, seq, pin)->id;
                    if(pin)
                    {
                        pin2wire[curpin->id] = w2;
                        wires[w2].intersection[PINTYPE] = {x2, y2};
                        
                    }
                    
                    //
                    used.insert(trackid);
                    
                   
                    // Setting neighbor wire
                    SetNeighbor(&wires[w1], &wires[w2], x1, y1);

                    // Swap element for iterating
                    w1 = w2;
                    e1 = e2;
                }
 
#ifdef DEBUG
                if(j==0)
                {
                    printf("trace layer num -> {");
                    for(auto& it : tracel)
                    {
                        printf(" %d", it);
                    }
                    printf(" }\n\n");
                }
#endif


            }
            else
            {
                cout << "No solution..." << endl;
            
                printf("maxDepth %dn", maxDepth);
                printf("trace layer num -> {");
                for(auto& it : tracel)
                {
                    printf(" %d", it);
                }
                printf(" }\n\n");

            }
            // backtrace end
        }

    }

}








void Circuit::pin_access() {
    for(int i=0; i < buses.size(); i++)
        pin_access(buses[i].name);
    return;
}

void Circuit::pin_access(string busName) {



    return;
}

void Circuit::debug() {

    for(int i=0; i < multipins.size(); i++){
        MultiPin* mp = &multipins[i];
        if( mp->busid == 1 ) {
            Segment* Seg = &rou->segs[rou->multipin2seg[mp->id]];

            cout << " seg layer : " << layers[Seg->l].name << endl;
            cout << " mp layer : " << layers[mp->l].name << endl;
            cout << " seg direction : " << layers[Seg->l].direction << endl;
            cout << " mp align : " << mp->align << endl;
            cout << " needVia : " << mp->needVia << endl;
            cout << " - - - - - - - - - - " << endl;
        }
    }

    return;
}
