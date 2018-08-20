
#include "circuit.h"
#include "route.h"
#include "func.h"

#include <stdlib.h>
#include <time.h>
#include <unordered_map>
#include <tuple>

//#define DEBUG_MAZE
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



static int randseed = 777;

struct Elem
{   
    int index;
    int cost;
    int count;
    

    Elem():
        index(INT_MAX),
        cost(INT_MAX),
        count(INT_MAX) {}

    Elem(int index, int cost, int count):
        index(index),
        cost(cost),
        count(count) {}

    Elem(const Elem& e):
        index(e.index),
        cost(e.cost),
        count(e.count) {}

    bool operator < (const Elem& e){ return cost < e.cost; }
};


bool operator < (const Elem& left, const Elem& right){
    return (left.cost < right.cost) || ((left.cost == right.cost) && (left.count < right.count));
}
bool operator > (const Elem& left, const Elem& right){
    return (left.cost > right.cost) || ((left.cost == right.cost) && (left.count > right.count));
}



int random(int moduler)
{
    //srand(time(timeseed));
    srand(randseed);
    return rand() % moduler;
}





void OABusRouter::Router::PostGlobalRouting()
{
    int numtrees, treeid, i;
    StTree* curtree;
    numtrees = rsmt.trees.size();
    for(i=0; i < numtrees; i++)
    {
        curtree = &rsmt.trees[i];
        treeid = curtree->id;
        if(!curtree->assign)
        {
            ObstacleAwareRouting(treeid);
        }

    }


}



int routing_direction(int x1, int y1, int x2, int y2, bool vertical)
{
    if(vertical)
    {
        if(y1 < y2)
            return Direction::Up;
        else if(y1 > y2)
            return Direction::Down;
        else
            return Direction::Point;
    }
    else
    {
        if(x1 < x2)
            return Direction::Right;
        else if(x1 > x2)
            return Direction::Left;
        else
            return Direction::Point;
    }
}

void OABusRouter::Router::RouteAll()
{
    int thr = 10;
    bool gcell_model;
    
    gcell_model = (grid.numCols > thr && grid.numRows > thr) ? true : false;

    if(!gcell_model)
    {
        GenBackbone();

        vector<int> sorted;
        for(auto& b : ckt->buses)
            sorted.push_back(b.id);
        
        Circuit* circuit = ckt;
        sort(sorted.begin(), sorted.end(), [&,circuit](int left, int right){
                int x1 = circuit->buses[left].llx; //(int)( 1.0 * ( circuit->buses[left].llx + circuit->buses[left].urx ) / 2 );
                int y1 = circuit->buses[left].lly; //(int)( 1.0 * ( circuit->buses[left].lly + circuit->buses[left].ury ) / 2 );
                int x2 = circuit->buses[right].llx; //(int)( 1.0 * ( circuit->buses[right].llx + circuit->buses[right].urx ) / 2 );
                int y2 = circuit->buses[right].lly; //(int)( 1.0 * ( circuit->buses[right].lly + circuit->buses[right].ury ) / 2 );
                return (y1 > y2); // || ((y1 == y2) && (x1 < x2));
                });

        for(auto& busid : sorted)
        {
            
            Bus* curbus = &ckt->buses[busid];
            printf("%s (%d %d) (%d %d)\n", curbus->name.c_str(), curbus->llx, curbus->lly, curbus->urx, curbus->ury);
        }
        for(auto& busid : sorted){
            ObstacleAwareBusRouting(busid);

        }

    }
    else
    {
        GenBackbone();

        TopologyMapping3D();

        CreateClips();
        SolveILP_v2();
        PostGlobalRouting();
        TrackAssign();

        MappingMultipin2Seg();
        MappingPin2Wire();
        Cut();
        for(int i =0; i < ckt->buses.size(); i++)
            PinAccess(i);
    
        Cut();
    }
    






}



bool OABusRouter::Router::ObstacleAwareBusRouting(int busid)
{
    // Cost Metrics
    int DEPTH_COST = 1000;
    int VIA_COST = 1000;
    int SPACING_VIOLATION = 100000;
    int WIRE_EXPAND = 10000;

    // Variables
    int i, j, wireid;
    int nummultipins, numwires, numpins;
    int numlayers, numbits, cost;
    int x1, y1, x2, y2, x, y;
    int llx, lly, urx, ury;
    int l1, l2, curDir, dist;
    int bitid, trackid, l, seq, width;
    int maxWidth;
    int xs[2], ys[2];
    bool pin;
    bool vertical_arrange1;
    bool vertical_arrange2;
    bool vertical;
    bool solution = true;
    typedef PointBG pt;
    typedef SegmentBG seg;
    typedef BoxBG box;
    typedef tuple<int,int,int> ituple;


    Bus* curbus;
    Bit* curbit;
    Pin* curpin;
    Wire* curwire, *targetwire;
    MultiPin* curmultipin;
    Track* curtrack;
    Segment* curseg;
    SegRtree* trackrtree;


    curbus = &ckt->buses[busid];
    numbits = curbus->numBits;
    // only two pin net
    if(curbus->numPinShapes > 2) 
        return false;


    printf("start routing %s\n", curbus->name.c_str());

    MultiPin *mp1, *mp2;
    mp1 = &ckt->multipins[curbus->multipins[0]];
    mp2 = &ckt->multipins[curbus->multipins[1]];

    if(multipin2llx[mp1->id] < multipin2llx[mp2->id])
    {
        mp1 = &ckt->multipins[curbus->multipins[0]];
        mp2 = &ckt->multipins[curbus->multipins[1]];
    }
    else
    {
        mp1 = &ckt->multipins[curbus->multipins[1]];
        mp2 = &ckt->multipins[curbus->multipins[0]];
    }

    vector<int> sorted1 = mp1->pins;
    vector<int> sorted2 = mp2->pins;
    Circuit* circuit = ckt;

    if(multipin2lly[mp1->id] < multipin2lly[mp2->id])
    {
        if(mp1->align == VERTICAL)
            sort(sorted1.begin(), sorted1.end(), [&,circuit](int left, int right){
                    return circuit->pins[left].lly > circuit->pins[right].lly; });
        else
            sort(sorted1.begin(), sorted1.end(), [&,circuit](int left, int right){
                    return circuit->pins[left].llx > circuit->pins[right].llx; });
    
        if(mp2->align == VERTICAL)
            sort(sorted2.begin(), sorted2.end(), [&,circuit](int left, int right){
                    return circuit->pins[left].lly > circuit->pins[right].lly; });
        else
            sort(sorted2.begin(), sorted2.end(), [&,circuit](int left, int right){
                    return circuit->pins[left].llx > circuit->pins[right].llx; });
    }
    else
    {
        if(mp1->align == VERTICAL)
            sort(sorted1.begin(), sorted1.end(), [&,circuit](int left, int right){
                    return circuit->pins[left].lly < circuit->pins[right].lly; });
        else
            sort(sorted1.begin(), sorted1.end(), [&,circuit](int left, int right){
                    return circuit->pins[left].llx < circuit->pins[right].llx; });
    
        if(mp2->align == VERTICAL)
            sort(sorted2.begin(), sorted2.end(), [&,circuit](int left, int right){
                    return circuit->pins[left].lly < circuit->pins[right].lly; });
        else
            sort(sorted2.begin(), sorted2.end(), [&,circuit](int left, int right){
                    return circuit->pins[left].llx < circuit->pins[right].llx; });
    }




    bool isRef = true;
    int maxDepth = INT_MAX;
    int bound;
    vector<int> setP;   // Pins
    vector<int> setT;   // Routing topologies
    vector<int> tracelNum;  // trace layer number
    vector<int> traceDir;   // trace direction
    //vector<int> tracePts;
    Pin *pin1, *pin2;
    trackrtree = &rtree.track;

    for(i=0; i < numbits; i++)
    {
    

        pin1 = &ckt->pins[mp1->pins[i]];
        pin2 = &ckt->pins[mp2->pins[i]];
        bitid = curbus->bits[i];

        if(pin1->llx > pin2->lly)
            swap(pin1, pin2);

        //pin1 = &ckt->pins[sorted1[i]];//&ckt->pins[mp1->pins[i]];
        //pin2 = &ckt->pins[sorted2[i]];//&ckt->pins[mp2->pins[i]];
        //bitid = ckt->bitHashMap[pin1->bitName];

        printf("start (%d %d) (%d %d) M%d \n", pin1->llx, pin1->lly, pin1->urx, pin1->ury, pin1->l);
        printf("end   (%d %d) (%d %d) M%d \n", pin2->llx, pin2->lly, pin2->urx, pin2->ury, pin2->l);


        vertical_arrange1 = (mp1->align == VERTICAL) ? true : false;
        vertical_arrange2 = (mp2->align == VERTICAL) ? true : false;

        // routing p1 and p2
        //
        

        box pinbox1, pinbox2;

        if(mp1->align == VERTICAL)
        {
            int pinlx = pin1->llx;
            int pinux = pin1->urx;
            int pinly = pin1->lly - (int)( 1.0*curbus->width[pin1->l] / 2 );
            int pinuy = pin1->ury + (int)( 1.0*curbus->width[pin1->l] / 2 );
            pinbox1 = box(pt(pinlx, pinly), pt(pinux, pinuy));    
        }
        else
        {
            int pinlx = pin1->llx - (int)( 1.0*curbus->width[pin1->l] / 2 );
            int pinux = pin1->urx + (int)( 1.0*curbus->width[pin1->l] / 2 );
            int pinly = pin1->lly;
            int pinuy = pin1->ury;
            pinbox1 = box(pt(pinlx, pinly), pt(pinux, pinuy));    
        }

        if(mp2->align == VERTICAL)
        {
            int pinlx = pin2->llx;
            int pinux = pin2->urx;
            int pinly = pin2->lly - (int)( 1.0*curbus->width[pin2->l] / 2 );
            int pinuy = pin2->ury + (int)( 1.0*curbus->width[pin2->l] / 2 );
            pinbox2 = box(pt(pinlx, pinly), pt(pinux, pinuy));    
        }
        else
        {
            int pinlx = pin2->llx - (int)( 1.0*curbus->width[pin2->l] / 2 );
            int pinux = pin2->urx + (int)( 1.0*curbus->width[pin2->l] / 2 );
            int pinly = pin2->lly;
            int pinuy = pin2->ury;
            pinbox2 = box(pt(pinlx, pinly), pt(pinux, pinuy));    
        }
        



        //
        seg elem;
        int dep;
        int e1, e2, t1, t2;
        int c1, c2;
        int xDest, yDest;
        int minElem = INT_MAX;
        int minCost = INT_MAX;
        bool hasMinElem = false;
        bool destination;
        vector<int> backtrace(rtree.elemindex, -1);
        vector<int> depth(rtree.elemindex, -1);
        vector<int> elemCost(rtree.elemindex, INT_MAX);
        vector<int> iterPtx(rtree.elemindex, -1);
        vector<int> iterPty(rtree.elemindex, -1);
        vector<int> lastPtx(rtree.elemindex, -1);
        vector<int> lastPty(rtree.elemindex, -1);
        vector<seg> element(rtree.elemindex);
        vector<pair<seg, int>> queries;

        // Priority Queue
        auto cmp = [](const ituple &left, const ituple &right){
            return (get<1>(left) + get<2>(left) > get<1>(right) + get<2>(right));
        };
        priority_queue<ituple , vector<ituple>, decltype(cmp)> PQ(cmp);

        queries.clear();
        trackrtree->query(bgi::intersects(pinbox1), back_inserter(queries));


        // Initial candidate
        for(auto& it : queries)
        {
            e1 = it.second;
            t1 = rtree.trackid(e1);
            l1 = rtree.layer(e1);
            maxWidth = rtree.width(e1);

            // width constraint
            if(maxWidth < curbus->width[l1])
                continue;


            // condition
            if(vertical_arrange1 == ckt->is_vertical(l1))
                continue;
            
            //if(used.find(t1) != used.end()) 
            //    continue;

            if(abs(pin1->l - l1) > 1) 
                continue;

            // visit
            //printf("element %d\n", e1);
            backtrace[e1] = e1;
            element[e1] = it.first;
            depth[e1] = 0;

            // get point
            lpt(element[e1], x1, y1);
            upt(element[e1], x2, y2);

            // Find iterating point of e1
            if(pin1->l != l1)
            {
                for(auto& it2 : queries)
                {
                    e2 = it2.second;
                    if(rtree.layer(e2) == pin1->l)
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
            }
            else
            {
                if(bg::intersects(pt(x1,y1), pinbox1))
                {
                    iterPtx[e1] = x1;
                    iterPty[e1] = y1;
                }
                else if(bg::intersects(pt(x2,y2), pinbox1))
                {
                    iterPtx[e1] = x2;
                    iterPty[e1] = y2;
                }
                else
                {
                    iterPtx[e1] = rtree.vertical(e1) ? x1 : (int)((bg::get<0,0>(pinbox1) + bg::get<1,0>(pinbox1))/2);
                    iterPty[e1] = rtree.vertical(e1) ? (int)((bg::get<0,1>(pinbox1) + bg::get<1,1>(pinbox1))/2) : y1;
                }
            }


            c1 = VIA_COST * abs(pin1->l - l1);
            c2 = 0; //2* (int)(bg::distance(pinbox2, pt(iterPtx[e1], iterPty[e1])));
            
            // if arrives at the destination
            if(abs(pin2->l-l1) < 2 && bg::intersects(element[e1], pinbox2))
            {
                lpt(element[e1], x1, y1);
                upt(element[e1], x2, y2);


                // Via insertion or intersects point
                int xoffset, yoffset;
                if(pin2->l != l1)
                {
                    int minOffset;
                    int minDist = INT_MAX;
                    vector<int> &offsets = ckt->layers[pin2->l].trackOffsets;
                    vector<int>::iterator lower, upper;
                    vertical = rtree.vertical(e1);

                    if(vertical){
                        lower = lower_bound(offsets.begin(), offsets.end(), pin2->lly);
                        upper = upper_bound(offsets.begin(), offsets.end(), pin2->ury);
                        xoffset = x1;
                    }else{
                        lower = lower_bound(offsets.begin(), offsets.end(), pin2->llx);
                        upper = upper_bound(offsets.begin(), offsets.end(), pin2->urx);
                        yoffset = y1;
                    }


                    while(lower != upper)
                    {
                        yoffset = (vertical)? *lower : yoffset;
                        xoffset = (vertical)? xoffset : *lower;
                        lower++;
                        dist = abs(iterPtx[e1] - xoffset) + abs(iterPty[e1] - yoffset);

                        if(dist < minDist)
                        {
                            minOffset = (vertical) ? yoffset : xoffset;
                            minDist = dist;
                        }
                    }

                    yoffset = (vertical)? minOffset : yoffset;
                    xoffset = (vertical)? xoffset : minOffset;

                }
                else
                {
                    if(bg::intersects(pt(x1, y1), pinbox2))
                    {
                        xoffset = x1;
                        yoffset = y1;
                    }
                    else if(bg::intersects(pt(x2, y2), pinbox2))
                    {
                        xoffset = x2;
                        yoffset = y2;
                    }
                    else
                    {
                        xoffset = rtree.vertical(e1) ? x1 : (int)((bg::get<0,0>(pinbox2) + bg::get<1,0>(pinbox2))/2);
                        yoffset = rtree.vertical(e1) ? (int)((bg::get<0,1>(pinbox2) + bg::get<1,1>(pinbox2))/2) : y1;
                        //xoffset = rtree.vertical(e1) ? x1 : (int)(bg::get<0,0>(pinbox2) + 0.5);
                        //yoffset = rtree.vertical(e1) ? (int)(bg::get<0,1>(pinbox2) + 0.5) : y1;
                    }
                }

                // end point of path
                lastPtx[e1] = xoffset;
                lastPty[e1] = yoffset;
                xs[0] = min(iterPtx[e1], lastPtx[e1]);
                xs[1] = max(iterPtx[e1], lastPtx[e1]);
                ys[0] = min(iterPty[e1], lastPty[e1]);
                ys[1] = max(iterPty[e1], lastPty[e1]);
                width = curbus->width[l1];
                vertical = rtree.vertical(e1);

                if(rtree.spacing_violations(bitid, xs, ys, l1, width, spacing[l1], vertical))
                    c2 += SPACING_VIOLATION;


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

            // push into the priority queue
            PQ.push(make_tuple(e1, c1, c2));
            elemCost[e1] = c1 + c2;
        
        }
        //


        while(PQ.size() > 0)
        {

            //printf("\n\ncurrent Queue size %d\n", PQ.size());
            int cost1, cost2;
            ituple e = PQ.top();
            PQ.pop();

            e1 = get<0>(e);
            cost1 = get<1>(e);
            cost2 = get<2>(e);
            
            //printf("MinCost %d CurCost %d\n", minCost, cost1 + cost2);
            // 
            if(hasMinElem && (e1 == minElem))
                break;
            

            // routing condition
            if(maxDepth <= depth[e1])
            {
                //printf("current depth %d maxDepth %d\n", depth[e1], maxDepth);
                continue;
            }

            t1 = rtree.trackid(e1);
            l1 = rtree.layer(e1);


            // query intersected tracks
            queries.clear();
            trackrtree->query(bgi::intersects(element[e1]), back_inserter(queries));


            // Intersected available tracks
            for(auto& it : queries)
            {
                e2 = it.second;
                t2 = rtree.trackid(e2);
                l2 = rtree.layer(e2);
                maxWidth = rtree.width(e2);
                elem = it.first;
                dep = depth[e1] + 1;
                destination = false;                
                // intersection
                intersection(element[e1], elem, x, y);

                // condition
                if(abs(l1 - l2) > 1)
                    continue;

                if(e1 == e2)
                    continue;

                // width constraint
                if(maxWidth < curbus->width[l2])
                    continue;


                if(!isRef && tracelNum[dep] != l2)
                {
                    // layer condition
                    if(tracelNum[dep] != l2)
                        continue;
                    curDir = routing_direction(iterPtx[e1], iterPty[e1], x, y, rtree.vertical(e1));
                    // if current routing direction is different,
                    // don't push into the priority queue
                    if(traceDir[depth[e1]] != curDir)
                    {
                        continue; 
                    }
                }

                // condition
                if((depth[e1] == 0) && 
                        (x == iterPtx[e2] && y == iterPty[e2]))
                {
                    continue;
                }


                // cost
                c1 = cost1 + abs(iterPtx[e1] - x) + abs(iterPty[e1] - y) + abs(l1-l2) * VIA_COST + DEPTH_COST;
                c2 = cost2; //2 * (int)(bg::distance(pt(iterPtx[e2], iterPty[e2]), pinbox2));
                
                // check spacing violation
                xs[0] = min(iterPtx[e1], x); //iterPtx[e2]);
                xs[1] = max(iterPtx[e1], x); //iterPtx[e2]);
                ys[0] = min(iterPty[e1], y); //iterPty[e2]);
                ys[1] = max(iterPty[e1], y); //iterPty[e2]);
                width = curbus->width[l1];
                vertical = rtree.vertical(e1);
                
                if(rtree.spacing_violations(bitid, xs, ys, l1, width, spacing[l1], vertical))
                    c2 += SPACING_VIOLATION;

                // if destination
                if(abs(pin2->l-l2) < 2 && bg::intersects(elem, pinbox2))
                {
                    destination = true;

                    lpt(elem, x1, y1);
                    upt(elem, x2, y2);
                    
                    // Via insertion or intersects point
                    int xoffset, yoffset;
                    if(pin2->l != l2)
                    {
                        int minOffset;
                        int minDist = INT_MAX;
                        vector<int> &offsets = ckt->layers[pin2->l].trackOffsets;
                        vector<int>::iterator lower, upper;
                        vertical = rtree.vertical(e2);

                        if(vertical){
                            lower = lower_bound(offsets.begin(), offsets.end(), pin2->lly);
                            upper = upper_bound(offsets.begin(), offsets.end(), pin2->ury);
                            xoffset = x1;
                        }else{
                            lower = lower_bound(offsets.begin(), offsets.end(), pin2->llx);
                            upper = upper_bound(offsets.begin(), offsets.end(), pin2->urx);
                            yoffset = y1;
                        }
                        
                        
                        while(lower != upper)
                        {
                            yoffset = (vertical) ? *lower : yoffset;
                            xoffset = (vertical) ? xoffset : *lower;
                            lower++;
                            dist = abs(x - xoffset) + abs(y - yoffset);
                            
                            if(dist < minDist)
                            {
                                minOffset = (vertical) ? yoffset : xoffset;
                                minDist = dist;
                            }
                        }

                        yoffset = (vertical)? minOffset : yoffset;
                        xoffset = (vertical)? xoffset : minOffset;

                    }
                    else
                    {
                        if(bg::intersects(pt(x1, y1), pinbox2))
                        {
                            xoffset = x1;
                            yoffset = y1;
                        }
                        else if(bg::intersects(pt(x2, y2), pinbox2))
                        {
                            xoffset = x2;
                            yoffset = y2;
                        }
                        else
                        {
                            xoffset = rtree.vertical(e2) ? x1 : (int)((bg::get<0,0>(pinbox2) + bg::get<1,0>(pinbox2))/2);
                            yoffset = rtree.vertical(e2) ? (int)((bg::get<0,1>(pinbox2) + bg::get<1,1>(pinbox2))/2) : y1;
                            //xoffset = rtree.vertical(e2) ? x1 : (int)(bg::get<0,0>(pinbox2) + 0.5);
                            //yoffset = rtree.vertical(e2) ? (int)(bg::get<0,1>(pinbox2) + 0.5) : y1;
                        }
                    }
                    
                    // end point of path
                    lastPtx[e2] = xoffset;
                    lastPty[e2] = yoffset;
                   
                    // condition(routing direction)
                    //if(i!=0)
                    if(!isRef)
                    {
                        curDir = routing_direction(x, y, xoffset, yoffset, rtree.vertical(e2));
                        // if current routing direction is different,
                        // don't push into the priority queue
                        if(traceDir[dep] != curDir)
                            continue;
                    }
                    
                    
                    // check spacing violation
                    xs[0] = min(x, xoffset);
                    ys[0] = min(y, yoffset);
                    xs[1] = max(x, xoffset);
                    ys[1] = max(y, yoffset);
                    width = curbus->width[l2];
                    vertical = rtree.vertical(e2);

                    if(rtree.spacing_violations(bitid, xs, ys, l2, width, spacing[l2], vertical))
                        c2 += SPACING_VIOLATION;
               

                }
                // if destination
 

                if(elemCost[e2] < c1 + c2)
                    continue;

                element[e2] = elem;
                depth[e2] = dep;
                backtrace[e2] = e1;
                iterPtx[e2] = x;
                iterPty[e2] = y;
                elemCost[e2] = c1 + c2;

                if(destination)
                {
                    if(hasMinElem)
                    {
                        if(minCost > c1 + c2)
                        {
                            minCost = c1 + c2;
                            minElem = e2;
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
            //
        }


        if(hasMinElem)
        {
            cout << "arrival..." << endl;
            e2 = minElem;
            l2 = rtree.layer(e2);
            
            //if(i==0)
            if(isRef)
            {
                maxDepth = depth[e2];
                curDir = routing_direction(iterPtx[e2], iterPty[e2], lastPtx[e2], lastPty[e2], rtree.vertical(e2));
                tracelNum.insert(tracelNum.begin(), l2);
                traceDir.insert(traceDir.begin(), curDir);
            }

            int w1, w2;
            xs[0] = min(lastPtx[e2], iterPtx[e2]);
            ys[0] = min(lastPty[e2], iterPty[e2]);
            xs[1] = max(lastPtx[e2], iterPtx[e2]);
            ys[1] = max(lastPty[e2], iterPty[e2]);
            seq = i;
            trackid = rtree.trackid(e2);
            pin = true;

            w2 = CreateWire(bitid, trackid, xs, ys, l2, seq, pin)->id;
            wires[w2].intersection[PINTYPE] = { lastPtx[e2], lastPty[e2] };
            pin2wire[pin2->id] = w2;
            wire2pin[w2] = pin2->id;
            
            while(backtrace[e2] != e2)
            {
                // Iterating
                e1 = backtrace[e2];
                x1 = iterPtx[e1];
                y1 = iterPty[e1];
                x2 = iterPtx[e2];
                y2 = iterPty[e2];
                l1 = rtree.layer(e1);
                vertical = rtree.vertical(e1);

                //if(i==0)
                if(isRef)
                {
                    tracelNum.insert(tracelNum.begin(), l1);
                    traceDir.insert(traceDir.begin(), routing_direction(x1, y1, x2, y2, vertical)); 
                }

                xs[0] = min(x1, x2);
                ys[0] = min(y1, y2);
                xs[1] = max(x1, x2);
                ys[1] = max(y1, y2);
                pin = (e1 == backtrace[e1]) ? true : false;
                trackid = rtree.trackid(e1);
                seq = i;

                w1 = CreateWire(bitid, trackid, xs, ys, l1, seq, pin)->id;
                
                if(pin)
                {
                    pin2wire[pin1->id] = w1;
                    wire2pin[w1] = pin1->id;
                    wires[w1].intersection[PINTYPE] = {x1, y1};
                }


                SetNeighbor(&wires[w1], &wires[w2], x2, y2);

                w2 = w1;
                e2 = e1;
            }

            //if(i==0)
            if(isRef)
            {
                printf("trace layer num -> {");
                for(auto& it : tracelNum)
                {
                    printf(" %d", it);
                }
                printf(" }\n");
                printf("trace direction -> {");
                for(auto& it : traceDir)
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
        }
        else
        {
            cout << "No solution..." << endl;
            if(isRef)
            {
                solution = false;
                break;
            }
            /*
            printf("maxDepth %d\n", maxDepth);
            printf("trace layer num -> {");
            for(auto& it : tracelNum)
            {
                printf(" %d", it);
            }
            printf(" }\n");
            printf("trace direction -> {");
            for(auto& it : traceDir)
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
            */
        }
        //
        isRef = false;
    }

    printf("\n\n\n");
    return solution; 



}




void OABusRouter::Router::ObstacleAwareRouting(int treeid)
{

    int numlayers, numcols, numrows, numgcells;
    int cost, count, i, j, k, deg, setNsize, bw, cap;
    int col1, row1, l1, cap1, col2, row2, l2, cap2;
    int g_selected, g1, g2, g_iter, g_closest, tmp;
    bool arrival, valid, visit, solution, isVertical;
    TreeNode *n1, *n2, *curnode;
    StTree* curtree;
    Gcell *curGC, *tarGC, *gc_n, *gc_iter, *gc_closest;
    
    vector<int> setN;   // stored gcell id which must be connected
    vector<int> setG;   // stored gcell id which the topology has
    vector<int> combG_PATH;
    Elem e;

    curtree = &rsmt.trees[treeid];
    deg = curtree->deg;
    //
    bw = ckt->buses[rsmt.busID[treeid]].numBits; //4;   
    curtree->segs.clear();
    curtree->junctions.clear();

    // store all gcell id correspond to each node
    for(i=0; i < deg; i++)
    {
        curnode = &curtree->nodes[i];
        col1 = curnode->x;
        row1 = curnode->y;
        l1 = curnode->l;
        setN.push_back(grid.GetIndex(col1,row1,l1));
    }
    
    setNsize = setN.size(); //deg;
    // Pick random node and push into the setG
    tmp = random(setNsize);
    g_selected = setN[tmp];
    setN.erase(setN.begin() + tmp);
    setNsize = setN.size();
    setG.push_back(g_selected);

    numlayers = grid.numLayers;
    numcols = grid.numCols;
    numrows = grid.numRows;
    numgcells = numlayers * numcols * numrows;
    printf("#layer %d #numcol %d #numrow %d #numgcell %d\n", numlayers, numcols, numrows, numgcells);
    arrival = false;
    solution = true;

    int backtrace[numgcells];
    int move[] = {-1, 1};

    // loop until setN empty
    while(setN.size() != 0)
    {
        memset(backtrace, -1, sizeof(int)*numgcells);
        priority_queue<Elem, vector<Elem>, greater<Elem>> PQ;
        count = 0;
        cost = 0;

        // Select Start node random
        tmp = random(setNsize);
        g_selected = setN[tmp];
        setN.erase(setN.begin() + tmp);
        setNsize = setN.size();
        backtrace[g_selected] = g_selected; // starting point

        PQ.push(Elem(g_selected, cost, count++));
        /*
        int minDist[setNsize];
        int totalDist = 0;
        for(j=0; j<setNsize; j++)
        {
            g_iter = setN[j];
            sort(setG.begin(), setG.end(), 
                    [&,g_iter,this](int left, int right){
                    int distl, distr;
                    Gcell* gc_iter = this->grid[g_iter];
                    Gcell* gc_left = this->grid[left];
                    Gcell* gc_right = this->grid[right];
                    distl = abs(gc_iter->x - gc_left->x) + abs(gc_iter->y - gc_left->y) + abs(gc_iter->l - gc_left->l);
                    distr = abs(gc_iter->x - gc_right->x) + abs(gc_iter->y - gc_right->y) + abs(gc_iter->l - gc_right->l);
                    return distl < distr; 
                    });
            g_closest = setG[0];   
            gc_iter = grid[g_iter];
            gc_closest = grid[g_closest];
            minDist[j] = 
                abs(gc_iter->x - gc_closest->x) + abs(gc_iter->y - gc_closest->y) + 3*abs(gc_iter->l - gc_closest->l);
            totalDist += minDist[j];
        }
        */

        //
        while(PQ.size() > 0)
        {
            e = PQ.top();
            PQ.pop();

            g1 = e.index;
            arrival = (find(setG.begin(), setG.end(), g1) != setG.end()) ? true : false;
            if(arrival) break;

            curGC = grid[g1];
            col1 = curGC->x;
            row1 = curGC->y;
            l1 = curGC->l;
            isVertical = (curGC->direction == VERTICAL) ? true : false;
            //cost = e.cost;
            // SetG U PATH(g_current)
            /*
            vector<int> path;
            int minDist2[setNsize];
            cost = 0;
            for(g_iter = g1; g_iter != backtrace[g_iter]; g_iter = backtrace[g_iter])
            {
                if(grid[g_iter]->l == grid[backtrace[g_iter]]->l)
                    cost += 1;
                else
                    cost += 3;
            
                gc_iter = grid[g_iter];
                for(j=0; j < setNsize; j++)
                {
                    gc_n = grid[setN[j]];
                    int dist = abs(gc_iter->x - gc_n->x) + abs(gc_iter->y - gc_n->y) + 3*abs(gc_iter->l - gc_n->l);
                    if(minDist[j] > dist)
                    {
                        cost -= abs(minDist[j] - dist);
                        minDist2[j] = dist;
                    }else{
                        minDist2[j] = minDist[j];
                    }
                }
                path.push_back(backtrace[g_iter]);
            }
            */
            // BFS adjacent
            for(i=0; i < 4; i++)
            {
                j = i % 2;
                k = (int)( 1.0 * i / 2 );
               

                if(k==0)
                {
                    col2 = (isVertical)? col1 : col1 + move[j];
                    row2 = (isVertical)? row1 + move[j] : row1;
                    l2 = l1;
                    //cost += 1;
                    cost = e.cost + 1;
                }

                if(k==1)
                {
                    col2 = col1;
                    row2 = row1;
                    l2 = l1 + move[j];
                    //cost += 3;
                    cost = e.cost + 1;
                }

                if(col2 < 0 || col2 >= numcols) continue;
                if(row2 < 0 || row2 >= numrows) continue;
                if(l2 < 0 || l2 >= numlayers) continue;

                // adj gcell index 
                g2 = grid.GetIndex(col2, row2, l2);
                visit = (backtrace[g2] != -1)? true: false;
                if(visit) continue;
                backtrace[g2] = g1;

                // Cap , cost calculation
                tarGC = grid[g2];
                cap = tarGC->cap;
                //cout << "Cap : " << cap << endl;
                if(cap > bw)
                {
                    /*
                    // distance between current Gcell and SetG
                    sort(setG.begin(), setG.end(), [&,tarGC, this] (int left, int right){
                        int distl, distr;
                        Gcell* gc_left = this->grid[left];
                        Gcell* gc_right = this->grid[right];
                        distl = abs(tarGC->x - gc_left->x) + abs(tarGC->y - gc_left->y) + abs(tarGC->l - gc_left->l);
                        distr = abs(tarGC->x - gc_right->x) + abs(tarGC->y - gc_right->y) + abs(tarGC->l - gc_right->l);
                        return distl < distr;
                        });
                    g_closest = setG[0];
                    gc_closest = grid[g_closest];
                    cost +=  abs(tarGC->x - gc_closest->x) + abs(tarGC->y - gc_closest->y) + abs(tarGC->l - gc_closest->l);
                               
                    combG_PATH.clear();
                    combG_PATH = setG;
                    */


                    // Summation of manhatan distance between all target node and SetG U PATH
                    //double tmp2 = 0;

                    
                    /*
                    for(j=0; j<setNsize; j++)
                    {
                        gc_n = grid[setN[j]];
                        gc_iter = grid[g2];
                        int dist = abs(gc_iter->x - gc_n->x) + abs(gc_iter->y - gc_n->y) + 3*abs(gc_iter->l - gc_n->l);
                        if(minDist2[j] > dist)
                        {
                            cost -= abs(minDist2[j] - dist);
                        }
                        sort(combG_PATH.begin(), combG_PATH.end(), 
                                [&,g_iter,this](int left, int right){
                                int distl, distr;
                                Gcell* gc_iter = this->grid[g_iter];
                                Gcell* gc_left = this->grid[left];
                                Gcell* gc_right = this->grid[right];
                                distl = abs(gc_iter->x - gc_left->x) + abs(gc_iter->y - gc_left->y) + abs(gc_iter->l - gc_left->l);
                                distr = abs(gc_iter->x - gc_right->x) + abs(gc_iter->y - gc_right->y) + abs(gc_iter->l - gc_right->l);
                                return distl < distr; 
                                });
                        g_closest = combG_PATH[0];   
                        gc_iter = grid[g_iter];
                        gc_closest = grid[g_closest];
                        tmp2 += abs(gc_iter->x - gc_closest->x) + abs(gc_iter->y - gc_closest->y) + abs(gc_iter->l - gc_closest->l);
                        
                    }
                    */
               
                    //cost += (int)(tmp2 / setNsize);
                    // Push into Priority Queue
                    PQ.push(Elem(g2, cost, count++));
                    backtrace[g2] = g1;
                }
            }
        }
        // end loop while(PQ.empty())

        if(PQ.empty())
        {
            cout << "Priority queue empty" << endl;
        }

        // Backtrace
        if(arrival)
        {
            g_selected = e.index;
            while(g_selected != backtrace[g_selected])
            {
                setG.push_back(backtrace[g_selected]);
                g_selected = backtrace[g_selected];
            }
        }else{
            //solution = false;
            //break;
        }
    }
    // end loop while(setN.empty())

    if(solution)
    {

#ifdef DEBUG_MAZE
        printf("tree%d -> \n{\n", treeid);
        for(i=0; i < curtree->nodes.size(); i++)
        {
            curnode = &curtree->nodes[i];
            col1 = curnode->x;
            row1 = curnode->y;
            l1 = curnode->l;

            printf("    n%d:(%d, %d, %d)\n", i, col1, row1, l1);
        }
        printf("}\n\n");
        printf("tree%d -> \n{\n", treeid);
        for(i=0; i < setG.size(); i++)
        {
            curGC = grid[setG[i]];
            col1 = curGC->x;
            row1 = curGC->y;
            l1 = curGC->l;

            printf("    g%d:(%d, %d, %d)\n", setG[i], col1, row1, l1);

        }
        printf("}\n\n\n");  
#endif
        curtree->gcells = setG;   

        

        int numver, numhor;
        int x1, y1, x2, y2;
        int segid, numsegs;
        int col, row, l;
        vector<int> verls;
        vector<int> horls;
        vector<SegRtree> srtree(numlayers);
        dense_hash_map<int,int> vermap;
        dense_hash_map<int,int> hormap;
        dense_hash_map<int,bool> vertical;
        vermap.set_empty_key(INT_MAX);
        hormap.set_empty_key(INT_MAX);
        vertical.set_empty_key(INT_MAX);

        for(i=0; i<numlayers;i++)
        {
           if(grid.direction[i] == VERTICAL)
           {
                vermap[i] = verls.size();
                verls.push_back(i);
                vertical[i] = true;
           }
           else
           {
                hormap[i] = horls.size();
                horls.push_back(i);
                vertical[i] = false;
           }
        }

        numver = verls.size();
        numhor = horls.size();


        vector<IntervalSetT> isetV(numver*numcols);
        vector<IntervalSetT> isetH(numhor*numrows);
        vector<tuple<int,int,int>> juncs;
        for(i=0; i < setG.size(); i++)
        {
            curGC = grid[setG[i]];
            col = curGC->x;
            row = curGC->y;
            l = curGC->l;

            if(vertical[l])
            {
                isetV[vermap[l]*numcols + col] +=
                    IntervalT::open(row-1, row+1);
            }
            else
            {
                isetH[hormap[l]*numrows + row] +=
                    IntervalT::open(col-1, col+1);
            }
            curGC->cap -= bw;
        }


        for(auto l : verls)
        {
            for(col=0; col < numcols; col++)
            {
                IntervalSetT& curset = isetV[vermap[l]*numcols + col];
                IntervalSetT::iterator it = curset.begin();
                DiscreteIntervalT intv;

                while(it != curset.end())
                {
                    intv = (*it++);
                    x1 = col;
                    x2 = col;
                    y1 = intv.lower() + 1;
                    y2 = intv.upper() - 1;

                    if(y1 == y2){
                        juncs.push_back(make_tuple(x1,y1,l));                
                        continue;
                    }
                    
                    segid = this->segs.size();
                    
                    Segment seg(segid, x1, y1, x2, y2, l, bw, true, true);
                    segs.push_back(seg);
                    seg2bus[segid] = rsmt.busID[treeid]; 
                    curtree->segs.push_back(segid);
                    
                    srtree[l].insert({SegmentBG(PointBG(x1,y1), PointBG(x2,y2)), segid});       
                }

            }
        }

        for(auto l : horls)
        {
            for(row=0; row < numrows; row++)
            {
                IntervalSetT& curset = isetH[hormap[l]*numrows + row];
                IntervalSetT::iterator it = curset.begin();
                DiscreteIntervalT intv;

                while(it != curset.end())
                {
                    intv = (*it++);
                    x1 = intv.lower() + 1; //col;
                    x2 = intv.upper() - 1;
                    y1 = row;
                    y2 = row; //intv.upper() - 1;
                    
                    
                    if(x1 == x2)
                    {
                        juncs.push_back(make_tuple(x1,y1,l));                
                        continue;
                    }
                    
                    segid = this->segs.size();
                    
                    Segment seg(segid, x1, y1, x2, y2, l, bw, true, false);
                    segs.push_back(seg);
                    seg2bus[segid] = rsmt.busID[treeid]; 
                    curtree->segs.push_back(segid);
                    srtree[l].insert({SegmentBG(PointBG(x1,y1), PointBG(x2,y2)), segid});       
                
                
                }

            }
        }


        for(auto& pt : juncs)
        {
            x1 = get<0>(pt);
            y1 = get<1>(pt);
            l1 = get<2>(pt);
            vector<pair<SegmentBG,int>> queries;
        
            int s1, s2;

            if(l1-1 < 0 || l1+1 > numlayers-1) continue;
            srtree[l1-1].query(bgi::intersects(PointBG(x1,y1)), back_inserter(queries));
            if(queries.size() == 0) continue;
            s1 = queries[0].second;

            queries.clear();
            srtree[l1+1].query(bgi::intersects(PointBG(x1,y1)), back_inserter(queries));
            if(queries.size() == 0) continue;
            s2 = queries[0].second;



            Junction jc;
            jc.id = junctions.size();
            jc.x = x1;
            jc.y = y1;
            jc.l1 = l1-1;
            jc.l2 = l1+1;
            jc.s1 = s1;
            jc.s2 = s2;
            jc.bw = bw;

            junctions.push_back(jc);
            curtree->junctions.push_back(jc.id);
            junc2bus[jc.id] = rsmt.busID[treeid];

            segs[s1].neighbor.push_back(s2);
            segs[s2].neighbor.push_back(s1);
            segs[s1].junctions.push_back(jc.id);
            segs[s2].junctions.push_back(jc.id);
        }


        for(auto& segid : curtree->segs)
        {
            Segment* curS = &segs[segid];
            vector<pair<SegmentBG,int>> queries;
            l1 = curS->l;
            SegmentBG seg(PointBG(curS->x1, curS->y1), PointBG(curS->x2, curS->y2));
            srtree[l1].remove(SegmentValT(seg, segid));

            if(l1 > 0)
            {
                queries.clear();
                srtree[l1-1].query(bgi::intersects(seg), back_inserter(queries));

                for(auto& val : queries)
                {
                    vector<PointBG> intersection;
                    bg::intersection(val.first, seg, intersection);

                    if(intersection.size() > 0)
                    {
                        x1 = (int)(bg::get<0>(intersection[0]) + 0.5);
                        y1 = (int)(bg::get<1>(intersection[0]) + 0.5);
                        Junction jc;
                        jc.id = junctions.size();
                        jc.x = x1;
                        jc.y = y1;
                        jc.l1 = l1-1;
                        jc.l2 = l1;
                        jc.s1 = val.second;
                        jc.s2 = segid;
                        jc.bw = curS->bw;

                        junctions.push_back(jc);
                        curtree->junctions.push_back(jc.id);
                        junc2bus[jc.id] = rsmt.busID[treeid];

                        segs[jc.s1].neighbor.push_back(jc.s2);
                        segs[jc.s2].neighbor.push_back(jc.s1);
                        segs[jc.s1].junctions.push_back(jc.id);
                        segs[jc.s2].junctions.push_back(jc.id);
                    }


                }

            }

            if(l1 < numlayers-1)
            {
                queries.clear();
                srtree[l1+1].query(bgi::intersects(seg), back_inserter(queries));

                for(auto& val : queries)
                {
                    vector<PointBG> intersection;
                    bg::intersection(val.first, seg, intersection);

                    if(intersection.size() > 0)
                    {
                        x1 = (int)(bg::get<0>(intersection[0]) + 0.5);
                        y1 = (int)(bg::get<1>(intersection[0]) + 0.5);
                        Junction jc;
                        jc.id = junctions.size();
                        jc.x = x1;
                        jc.y = y1;
                        jc.l1 = l1;
                        jc.l2 = l1+1;
                        jc.s1 = segid; //val.second;
                        jc.s2 = val.second;
                        jc.bw = curS->bw;

                        junctions.push_back(jc);
                        curtree->junctions.push_back(jc.id);
                        junc2bus[jc.id] = rsmt.busID[treeid];

                        segs[jc.s1].neighbor.push_back(jc.s2);
                        segs[jc.s2].neighbor.push_back(jc.s1);
                        segs[jc.s1].junctions.push_back(jc.id);
                        segs[jc.s2].junctions.push_back(jc.id);
                    }
                }
            }

        }

    }
    else
    {
        //printf("No solution tree%d\n", treeid);

    }
}


//void OABusRouter::Router::MazeRouting(int p1, int p2)
//{



//}








