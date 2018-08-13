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
    
    // Cost Metrics
    int DEPTH_COST = 1000;
    int VIA_COST = 1000;
    int SPACING_VIOLATION = 10000;
    int WIRE_EXPAND = 10000;

    // Variables
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

    // Routing direction
    enum Direction
    {
        Left,
        Right,
        Up,
        Down,
        T_Junction,
        Point
    };

    
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
        vector<int> sorted = curmultipin->pins;
        vector<int> tracel;
        vector<int> tracedir; //Dir;
        vector<int> tracepts;
        set<int> used;
        Circuit* circuit = ckt;
        vertical_arrange = (curmultipin->align == VERTICAL)? true : false;

        curseg = &segs[multipin2seg[curmultipin->id]];
        llx = grid.llx(grid.GetIndex(curseg->x1, curseg->y1, curseg->l));
        lly = grid.lly(grid.GetIndex(curseg->x1, curseg->y1, curseg->l));
        urx = grid.urx(grid.GetIndex(curseg->x2, curseg->y2, curseg->l));
        ury = grid.ury(grid.GetIndex(curseg->x2, curseg->y2, curseg->l));

        // Ordering
        if(curmultipin->align == VERTICAL)
        {
            if(ury < multipin2lly[curmultipin->id])
            {
                sort(sorted.begin(), sorted.end(), [&,circuit](int left, int right){
                        return circuit->pins[left].lly > circuit->pins[right].lly;
                        });
            }
            else if(lly > multipin2ury[curmultipin->id])
            {
                sort(sorted.begin(), sorted.end(), [&,circuit](int left, int right){
                        return circuit->pins[left].lly < circuit->pins[right].lly;
                        });
            }
        }
        else
        {
            if(urx < multipin2llx[curmultipin->id])
            {
                sort(sorted.begin(), sorted.end(), [&,circuit](int left, int right){
                        return circuit->pins[left].llx > circuit->pins[right].llx;
                        });
            }
            else if(llx > multipin2urx[curmultipin->id])
            {
                sort(sorted.begin(), sorted.end(), [&,circuit](int left, int right){
                        return circuit->pins[left].llx < circuit->pins[right].llx;
                        });
            }
        }


        // Maze routing for each pin
        for(j=0; j < numpins; j++)
        {
            curpin = &ckt->pins[sorted[j]]; //curmultipin->pins[j]];
            targetwire = &wires[pin2wire[curpin->id]]; //curmultipin->pins[j]]]; //curpin->id]];       
            tarl = targetwire->l;
            bitid = ckt->bitHashMap[curpin->bitName];
            curbit = &ckt->bits[bitid];
            
            llx = curpin->llx;
            lly = curpin->lly;
            urx = curpin->urx;
            ury = curpin->ury;
            pinl = curpin->l;
            
            int e1, e2, t1, t2;
            int c1, c2; 
            int targetShape;
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
            
            auto cmp = [](const ituple &left, const ituple &right){
                return (get<1>(left) + get<2>(left) > get<1>(right) + get<2>(right));
            };
            priority_queue<ituple , vector<ituple>, decltype(cmp)> PQ(cmp);
           
            // Pin box
            box b(PointBG(llx,lly), PointBG(urx,ury));
            
            queries.clear();    
            trackrtree->query(bgi::intersects(b), back_inserter(queries));
            
            // Candidates intersected with pin
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

                    // c1 : iterating distance
                    // c2 : distance between iterating point and target wire
                    c1 = VIA_COST* abs(pinl-l1);
                    c2 = (int)(bg::distance(wireseg, pt(iterPtx[e1],iterPty[e1])));
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
                    PQ.push(make_tuple(e1,c1,c2));   
                }
            }



            // Search until getting destination
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
                
                // if current element has minimum cost, and equals to minimum element
                // break loop
                if(hasMinElem && (e1 == minElem))
                {
                    break;

                }
                //
                // if maxDepth is assigned,
                // current depth cannot over maxDepth
                if(maxDepth <= depth[e1])
                {
                    continue;      
                }


                queries.clear();
                trackrtree->query(bgi::intersects(element[e1]), back_inserter(queries));
                
                // Intersected tracks available
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
                        // condition5
                        if(j!=0)
                        {
                            // Get direction
                            int curDir;
                            if(rtree.direction(e1) == VERTICAL)
                            {
                                if(iterPty[e1] == iterPty[e2])
                                    curDir = Direction::Point;
                                else if(iterPty[e1] < iterPty[e2])
                                    curDir = Direction::Up;
                                else if(iterPty[e1] > iterPty[e2])
                                    curDir = Direction::Down;
                            }
                            else
                            {
                                if(iterPtx[e1] == iterPtx[e2])
                                    curDir = Direction::Point;
                                else if(iterPtx[e1] < iterPtx[e2])
                                    curDir = Direction::Right;
                                else if(iterPtx[e1] > iterPtx[e2])
                                    curDir = Direction::Left;
                            }
                            
                            // if current routing direction is different,
                            // don't push into the priority queue
                            if(tracedir[depth[e1]] != curDir)
                                continue;
                        }
                        
                        
                        // condition4
                        if(depth[e1] == 0)
                        {
                            if(iterPtx[e1] == iterPtx[e2] && iterPty[e1] == iterPty[e2])
                            {
                                continue;
                            }
                        }

                        // Cost
                        c1 = cost1 + abs(iterPtx[e1] - iterPtx[e2]) + abs(iterPty[e1] - iterPty[e2]) + abs(l1-l2)*VIA_COST + DEPTH_COST;
                        c2 = (int)(bg::distance(p, wireseg));    

                       
                       
                        
                        // condition7
                        int xs[2], ys[2];
                        xs[0] = min(iterPtx[e1], iterPtx[e2]);
                        xs[1] = max(iterPtx[e1], iterPtx[e2]);
                        ys[0] = min(iterPty[e1], iterPty[e2]);
                        ys[1] = max(iterPty[e1], iterPty[e2]);
                        if(rtree.direction(e1) == VERTICAL)
                        {
                            xs[0] -= 
                                ((int)(1.0*curbus->width[rtree.layer(e1)]/2) + spacing[rtree.layer(e1)]);
                            xs[1] += 
                                ((int)(1.0*curbus->width[rtree.layer(e1)]/2) + spacing[rtree.layer(e1)]);
                            ys[0] -= spacing[rtree.layer(e1)];
                            ys[1] += spacing[rtree.layer(e1)];
                        }
                        else
                        {
                            ys[0] -= 
                                ((int)(1.0*curbus->width[rtree.layer(e1)]/2) + spacing[rtree.layer(e1)]);
                            ys[1] += 
                                ((int)(1.0*curbus->width[rtree.layer(e1)]/2) + spacing[rtree.layer(e1)]);
                            xs[0] -= spacing[rtree.layer(e1)];
                            xs[1] += spacing[rtree.layer(e1)];

                        }

                        if(rtree.spacing_violations(bitid, xs, ys, rtree.layer(e1)))
                        {
                            c1 += SPACING_VIOLATION;
                        }


                     
                        // If target wire is found, 
                        if(bg::intersects(element[e2], wireseg) && abs(l2 - tarl) < 2)
                        {
                            // condition6 
                            if(!targetwire->leaf() && (l2 == tarl))
                                continue;

                            // Intersection
                            vector<pt> intersection;
                            bg::intersection(element[e2], wireseg, intersection);
                            pt p = intersection[0];
                            x1 =  (int)(bg::get<0>(p) + 0.5);
                            y1 =  (int)(bg::get<1>(p) + 0.5);

                            if(j!=0)
                            {
                                int curDir;
                                if(rtree.direction(e2) == VERTICAL)
                                {
                                    if(y1 == iterPty[e2])
                                        curDir = Direction::Point;
                                    else if(y1 < iterPty[e2])
                                        curDir = Direction::Down;
                                    else if(y1 > iterPty[e2])
                                        curDir = Direction::Up;
                                }
                                else
                                {
                                    if(x1 == iterPtx[e2])
                                        curDir = Direction::Point;
                                    else if(x1 < iterPtx[e2])
                                        curDir = Direction::Left;
                                    else if(x1 > iterPtx[e2])
                                        curDir = Direction::Right;
                                }

                                // if current routing direction is different,
                                // don't push into the priority queue
                                if(tracedir[depth[e2]] != curDir)
                                {
#ifdef DEBUG_PIN
                                    printf("current depth %d trace dir ", depth[e2]);
                                    if(tracedir[depth[e2]] == Direction::Up)
                                        printf("Up ");
                                    else if(tracedir[depth[e2]] == Direction::Down)
                                        printf("Down ");
                                    else if(tracedir[depth[e2]] == Direction::Left)
                                        printf("Left ");
                                    else if(tracedir[depth[e2]] == Direction::Right)
                                        printf("Right ");
                                    else if(tracedir[depth[e2]] == Direction::Point)
                                        printf("Point ");
                                    else
                                        printf("T-Junction ");

                                    printf("current dir ");
                                    if(curDir == Direction::Up)
                                        printf("Up ");
                                    else if(curDir == Direction::Down)
                                        printf("Down ");
                                    else if(curDir == Direction::Left)
                                        printf("Left ");
                                    else if(curDir == Direction::Right)
                                        printf("Right ");
                                    else if(curDir == Direction::Point)
                                        printf("Point ");
                                    else
                                        printf("T-Junction ");

                                    printf("(%d %d) -> (%d %d)\n", iterPtx[e2], iterPty[e2], x1, y1);
#endif
                                    continue;
                                }
                            
                                // condition8 (wire ordering)
                                int current;
                                int lower = tracepts[0];
                                int upper = tracepts[tracepts.size()-1];

                                if(tarl == l2)
                                    current = (targetwire->vertical) ? iterPty[e1] : iterPtx[e1];
                                else
                                    current = (targetwire->vertical) ? iterPty[e2] : iterPtx[e2];
                            
                                if(lower < current && current < upper)
                                    continue;
                            }


                            // Check spacing violation
                            xs[0] = min(x1, iterPtx[e2]);
                            xs[1] = max(x1, iterPtx[e2]);
                            ys[0] = min(y1, iterPty[e2]);
                            ys[1] = max(y1, iterPty[e2]);
                            if(rtree.direction(e2) == VERTICAL)
                            {
                                xs[0] -= 
                                    ((int)(1.0*curbus->width[rtree.layer(e2)]/2) + spacing[rtree.layer(e2)]);
                                xs[1] += 
                                    ((int)(1.0*curbus->width[rtree.layer(e2)]/2) + spacing[rtree.layer(e2)]);
                                ys[0] -= spacing[rtree.layer(e2)];
                                ys[1] += spacing[rtree.layer(e2)];
                            }
                            else
                            {
                                ys[0] -= 
                                    ((int)(1.0*curbus->width[rtree.layer(e2)]/2) + spacing[rtree.layer(e2)]);
                                ys[1] += 
                                    ((int)(1.0*curbus->width[rtree.layer(e2)]/2) + spacing[rtree.layer(e2)]);
                                xs[0] -= spacing[rtree.layer(e2)];
                                xs[1] += spacing[rtree.layer(e2)];
                            }

                            if(rtree.spacing_violations(bitid, xs, ys, rtree.layer(e2)))
                                c1 += SPACING_VIOLATION;

                            ////////////////////////////////////////////////////////////

                            //    c1 += WIRE_EXPAND;

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
                cout << "arrival..." << endl;    
                e1 = minElem;
                l1 = tarl;
                l2 = rtree.layer(e1); //trackNuml[e1];
               
                if(l1 == l2)
                {
                    
                    x1 = targetwire->x1;
                    y1 = targetwire->y1;
                    x2 = targetwire->x2;
                    y2 = targetwire->y2;
                    l = targetwire->l;
                   
                    // Store trace
                    if(j==0)
                    {
                        maxDepth = depth[e1];
                        tracel.insert(tracel.begin(), l);
                        // Direction
                        if(targetwire->vertical)
                        {
                            if(iterPty[e1] >= y2){
                                tracedir.insert(tracedir.begin(), Direction::Down);
                            }else if(iterPty[e1] <= y1){
                                tracedir.insert(tracedir.begin(), Direction::Up);
                            }else{
                                tracedir.insert(tracedir.begin(), Direction::T_Junction);
                            }
                        }
                        else
                        {   
                            if(iterPtx[e1] >= x2){
                                tracedir.insert(tracedir.begin(), Direction::Left);
                            }else if(iterPtx[e1] <= x1){
                                tracedir.insert(tracedir.begin(), Direction::Right);
                            }else{
                                tracedir.insert(tracedir.begin(), Direction::T_Junction);
                            }
                        }
                    }
                    
                    //
                    pin = (e1 == backtrace[e1]) ? true : false;
                    targetwire->pin = (e1 == backtrace[e1]) ? true : false;

                    /////////////////////////////////////////////
                    if(targetwire->vertical)
                        tracepts.push_back(iterPty[e1]);
                    else 
                        tracepts.push_back(iterPtx[e1]);
                    sort(tracepts.begin(), tracepts.end());
                    ////////////////////////////////////////////
                    
                    xs[0] = min(iterPtx[e1], x1); //targetwire->x1;
                    xs[1] = max(iterPtx[e1], x2); //targetwire->x2;
                    ys[0] = min(iterPty[e1], y1); //targetwire->y1;
                    ys[1] = max(iterPty[e1], y2); //targetwire->y2;
                    targetwire->x1 = xs[0];
                    targetwire->x2 = xs[1];
                    targetwire->y1 = ys[0];
                    targetwire->y2 = ys[1];

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
                    
                    /////////////////////////////////////////////
                    if(targetwire->vertical)
                        tracepts.push_back(y1);
                    else 
                        tracepts.push_back(x1);
                    
                    sort(tracepts.begin(), tracepts.end());
                    ////////////////////////////////////////////
                    
                    
                    //
                    if(j==0) 
                    {
                        bool alignVertical = (curmultipin->align == VERTICAL)? true : false;
                        bool targetVertical = targetwire->vertical;
                        if(alignVertical != targetVertical)
                        {
                            maxDepth = depth[e1];
                        }
                        else
                        {
                            if(depth[e1] < 1)
                            {
                                maxDepth = depth[e1] + 1;
                                tracel.insert(tracel.begin(), targetwire->l);
                                // Direction
                                if(targetwire->vertical)
                                {
                                    if(y1 >= targetwire->y2){
                                        tracedir.insert(tracedir.begin(), Direction::Down);
                                    }else if(y1 <= targetwire->y1){
                                        tracedir.insert(tracedir.begin(), Direction::Up);
                                    }else{
                                        tracedir.insert(tracedir.begin(), Direction::T_Junction);
                                    }
                                }
                                else
                                {
                                    if(x1 >= targetwire->x2){
                                        tracedir.insert(tracedir.begin(), Direction::Left);
                                    }else if(x1 <= targetwire->x1){
                                        tracedir.insert(tracedir.begin(), Direction::Right);
                                    }else{
                                        tracedir.insert(tracedir.begin(), Direction::T_Junction);
                                    }
                                }
                            }
                            else
                            {
                                maxDepth = depth[e1];
                            }
                        }
                        
                        tracel.insert(tracel.begin(), l);
                        // Direction
                        if(rtree.direction(e1) == VERTICAL)
                        {
                            if(y1 == y2)
                                tracedir.insert(tracedir.begin(), Direction::Point);
                            else if(y2 > y1)
                                tracedir.insert(tracedir.begin(), Direction::Down);
                            else if(y2 < y1)
                                tracedir.insert(tracedir.begin(), Direction::Up);
                        }
                        else
                        {
                            if(x1 == x2)
                                tracedir.insert(tracedir.begin(), Direction::Point);
                            else if(x2 > x1)
                                tracedir.insert(tracedir.begin(), Direction::Left);
                            else if(x2 < x1)
                                tracedir.insert(tracedir.begin(), Direction::Right);
                        }
                    }
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
                        wire2pin[w2] = curpin->id;
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
                    { 
                        tracel.insert(tracel.begin(), l);
                        // Direction
                        if(rtree.direction(e2) == VERTICAL)
                        {
                            if(y1 == y2)
                                tracedir.insert(tracedir.begin(), Direction::Point);
                            else if(y2 > y1)
                                tracedir.insert(tracedir.begin(), Direction::Down);
                            else if(y2 < y1)
                                tracedir.insert(tracedir.begin(), Direction::Up);
                        }
                        else
                        {
                            if(x1 == x2)
                                tracedir.insert(tracedir.begin(), Direction::Point);
                            else if(x2 > x1)
                                tracedir.insert(tracedir.begin(), Direction::Left);
                            else if(x2 < x1)
                                tracedir.insert(tracedir.begin(), Direction::Right);
                        }
                        

                    }
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
                        wire2pin[w2] = curpin->id;
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
            
                printf("maxDepth %d\n", maxDepth);
                printf("trace layer num -> {");
                for(auto& it : tracel)
                {
                    printf(" %d", it);
                }
                printf(" }\n");
                printf("trace direction -> {");
                for(auto& it : tracedir)
                {
                    if(it == Direction::Point)
                        printf(" Point");
                    if(it == Direction::Left)
                        printf(" Left");
                    if(it == Direction::Right)
                        printf(" Right");
                    if(it == Direction::Down)
                        printf(" Down");
                    if(it == Direction::Up)
                        printf(" Up");
                    if(it == Direction::T_Junction)
                        printf(" T-junction");
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
