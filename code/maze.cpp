
#include "circuit.h"
#include "route.h"
#include "func.h"

#include <stdlib.h>
#include <time.h>
#include <unordered_map>
#include <tuple>


#define VIA_COST 1000
#define DEPTH_COST 1000
#define SPACING_VIOLATION 100000


#define DEBUG_MAZE2
//#define DEBUG_MAZE
// Routing direction
typedef PointBG pt;
typedef SegmentBG seg;
typedef BoxBG box;


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

template <typename A>
A pop_random(vector<A>& target)
{
    int index = random(target.size());
    A value = target[index];
    target.erase(target.begin() + index);
    return value;
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
void into_array(int v1, int v2, int v[])
{
    v[0] = v1;
    v[1] = v2;
}

void into_array(int x1, int x2, int y1, int y2, int x[], int y[])
{
    x[0] = x1;
    x[1] = x2;
    y[0] = y1;
    y[1] = y2;
}



void OABusRouter::Wire::get_info(int b, int t, int x[], int y[], int curl, int s, bool accessPin)
{
    id = NOT_ASSIGN;
    x1 = x[0];
    y1 = y[0];
    x2 = x[1];
    y2 = y[1];
    l = curl;
    seq = s;
    busid = ckt->busHashMap[ckt->bits[b].busName];
    bitid = b;
    width = ckt->buses[busid].width[curl];
    trackid = t;
    vertical = ckt->is_vertical(curl);
    pin = accessPin;
    via = (x[0]==x[1] && y[0]==y[1]) ? true : false;
}

void OABusRouter::Wire::add_intersection(int key, pair<int,int> p)
{
    intersection[key] = p;
}

void pin_area(int x[], int y[], int align, int width, box& pb)
{
    if(align == VERTICAL)
    {
        y[0] -= (int)( 1.0* width / 2 );
        y[1] += (int)( 1.0* width / 2 );
    }
    else
    {
        x[0] -= (int)( 1.0* width / 2 );
        x[1] += (int)( 1.0* width / 2 );
    }
    
    pb = box(pt(x[0], y[0]), pt(x[1], y[1]));
}

void expand_width(int x[], int y[], int width, bool vertical)
{
    if(vertical)
    {
        x[0] -= (int)(1.0 * width / 2);
        x[1] += (int)(1.0 * width / 2);
    }
    else
    {
        y[0] -= (int)(1.0 * width / 2);
        y[1] += (int)(1.0 * width / 2);
    }
}


void OABusRouter::Router::route_all()
{
    int thr = 10;
    bool gcell_model;
    
    //gcell_model = (grid.numCols > thr && grid.numRows > thr) ? true : false;
    gcell_model = false;
    if(!gcell_model)
    {
        gen_backbone();

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
            Bus* curbus = &ckt->buses[busid];
            if(route_bus(busid))
            {
                printf("%s -> routing success\n", curbus->name.c_str());   
            }
            else
            {
                printf("%s -> routing failed\n", curbus->name.c_str());
            }
        }

    }
    else
    {
        gen_backbone();
        topology_mapping3d();

        CreateClips();
        SolveILP_v2();
        PostGlobalRouting();
        track_assign();
        mapping_multipin2seg();
        mapping_pin2wire();
        cut();
        for(int i =0; i < ckt->buses.size(); i++)
            pin_access(i);
    
        cut();
    }
    
}

bool OABusRouter::Router::route_bus(int busid)
{
    int congested;
    int mp1, mp2;
    int numMultipins;
    pair<int,int> candi;
    vector<int> mps;                    // multipins of target bus
    vector<int> reroutes;          // ripupped buses
    vector<pair<int,int>> comb;         // combination

    mps = ckt->buses[busid].multipins;
    numMultipins = ckt->buses[busid].multipins.size();

    // combination of picking the two pins
    for(int i=0; i < numMultipins; i++)
        for(int j=i+1; j < numMultipins; j++)
            comb.push_back({mps[i],mps[j]});

    // loop for route_two_pin_net
    while(comb.size() > 0)
    {
        candi = pop_random(comb);
        mp1 = candi.first;
        mp2 = candi.second;
        // pick two multipins for routing
        // routing topologies
        vector<Segment> tp;

        if(route_twopin_net(busid, mp1, mp2, tp))
        {
            // route until no remained multipins
            while(mps.size() > 0)
            {
                mp1 = pop_random(mps);
                // route target multipin to net topologies (tp)
                /*
                if(!route_multipin_to_tp(busid, mp1, tp))
                {

                    // rip-up
                    congested = get_congested_bus(busid);
                    rip_up(congested);
                    reroutes.push_back(congested);
                    // re-try
                    mps.push_back(mp1);
                }
                */
            }

            // reroute rip-upped buses
            for(int i=0; i < reroutes.size() ; i++)
            {
                // if failed again
                if(!reroute(reroutes[i]))
                {
                    // do something
                    return false;
                }
            }

            // update to global structure
            update_net_tp(tp);

            // routing success
            return true;
        }
    }
    // routing fail
    return false;
}



bool OABusRouter::Router::route_twopin_net(int busid, int m1, int m2, vector<Segment> &tp)
{
    printf("route twopin net\n");
    // Variables
    int i, j, wireid;
    int numwires, numpins;
    int numbits, cost;
    int x1, y1, x2, y2, x, y;
    int llx, lly, urx, ury;
    int l1, l2, curDir, dist;
    int bitid, trackid, l, seq;
    int maxWidth, align1, align2;
    int xs[2], ys[2];
    int wirex[2], wirey[2];
    int pin1x[2], pin1y[2];
    int pin2x[2], pin2y[2];
    int maxDepth = INT_MAX;
    int count, index;

    bool pin;
    bool vertical_arrange1;
    bool vertical_arrange2;
    bool vertical;
    bool solution = true;
    bool isRef = true;
    
    
    typedef PointBG pt;
    typedef SegmentBG seg;
    typedef BoxBG box;
    typedef tuple<int,int,int> ituple;

    vector<int> tracelNum;  // trace layer number
    vector<int> traceDir;   // trace direction
    vector<int> p1;
    vector<int> p2;
    vector<int> sequence;
    vector<Wire> created;
    vector<pair<int, int>> edges;
    vector<pair<int, int>> pts;
    dense_hash_map<int,int> width;

    //vector<int> tracePts;
    Pin *pin1, *pin2;
    Bus* curbus;
    SegRtree* trackrtree;
    MultiPin *mp1, *mp2;

    // get copied rtree
    Rtree localrtree(rtree);
    trackrtree = &localrtree.track;
    
    mp1 = &ckt->multipins[m1];
    mp2 = &ckt->multipins[m2];
    if(multipin2llx[mp1->id] > multipin2llx[mp2->id])
        swap(mp1, mp2);

    curbus = &ckt->buses[busid];
    numbits = curbus->numBits;
    width = curbus->width;


    printf("start routing %s\n", curbus->name.c_str());

    for(i=0; i < numbits; i++)
    {
        // always pin1 located leftside of pin2
        pin1 = &ckt->pins[mp1->pins[i]];
        pin2 = &ckt->pins[mp2->pins[i]];
        align1 = mp1->align;
        align2 = mp2->align;
        bitid = curbus->bits[i];
        seq = i; 
        vertical_arrange1 = (mp1->align == VERTICAL) ? true : false;
        vertical_arrange2 = (mp2->align == VERTICAL) ? true : false;
        
        if(pin1->llx > pin2->llx)
        {
            swap(pin1, pin2);
            swap(vertical_arrange1, vertical_arrange2);
            swap(align1, align2);
        }
        //
        p1.push_back(pin1->id);
        p2.push_back(pin2->id);
        sequence.push_back(bitid);

        //

        // Get Pin Area valid
        box pinbox1, pinbox2;
        into_array(pin1->llx, pin1->urx, pin1->lly, pin1->ury, pin1x, pin1y);
        into_array(pin2->llx, pin2->urx, pin2->lly, pin2->ury, pin2x, pin2y);
        pin_area(pin1x, pin1y, align1, width[pin1->l], pinbox1);
        pin_area(pin2x, pin2y, align2, width[pin2->l], pinbox2);

        /*
        printf("p1 (%d %d) (%d %d) -> ", pin1->llx, pin1->lly, pin1->urx, pin1->ury);
        cout << bg::dsv(pinbox1) << endl;
        printf("p2 (%d %d) (%d %d) -> ", pin2->llx, pin2->lly, pin2->urx, pin2->ury);
        cout << bg::dsv(pinbox2) << endl;
        */
        // variables
        seg elem;
        int dep;
        int w1, w2;
        int e1, e2, t1, t2;
        int c1, c2;
        int xDest, yDest;
        int minElem = INT_MAX;
        int minCost = INT_MAX;
        bool hasMinElem = false;
        bool destination;
        vector<int> backtrace(localrtree.elemindex, -1);
        vector<int> depth(localrtree.elemindex, -1);
        vector<int> elemCost(localrtree.elemindex, INT_MAX);
        vector<int> iterPtx(localrtree.elemindex, -1);
        vector<int> iterPty(localrtree.elemindex, -1);
        vector<int> lastPtx(localrtree.elemindex, -1);
        vector<int> lastPty(localrtree.elemindex, -1);
        vector<seg> element(localrtree.elemindex);
        vector<pair<seg, int>> queries;

        // Priority Queue
        auto cmp = [](const ituple &left, const ituple &right){
            return (get<1>(left) + get<2>(left) > get<1>(right) + get<2>(right));
        };
        priority_queue<ituple , vector<ituple>, decltype(cmp)> PQ(cmp);


        //
        queries.clear();
        trackrtree->query(bgi::intersects(pinbox1), back_inserter(queries));

        // Initial candidate
        for(auto& it : queries)
        {
            e1 = it.second;
            t1 = localrtree.trackid(e1);
            l1 = localrtree.layer(e1);
            maxWidth = localrtree.width(e1);

            // width constraint
            if(maxWidth < width[l1])
                continue;

            // condition
            if(vertical_arrange1 == ckt->is_vertical(l1))
                continue;

            if(abs(pin1->l - l1) > 1) 
                continue;


            // visit
            backtrace[e1] = e1;         // e1 is root
            element[e1] = it.first;
            depth[e1] = 0;

            // get point
            lpt(element[e1], x1, y1);
            upt(element[e1], x2, y2);


            // Find iterating point of e1
            into_array(min(x1, x2), max(x1, x2),  min(y1, y2), max(y1, y2), wirex, wirey);
            intersection_pin(pin1x, pin1y, pin1->l, wirex, wirey, l1, -1, -1, x, y);
            iterPtx[e1] = x;
            iterPty[e1] = y;

            c1 = VIA_COST * abs(pin1->l - l1);
            c2 = 0; //2* (int)(bg::distance(pinbox2, pt(iterPtx[e1], iterPty[e1])));
            
            // if arrives at the destination
            if(abs(pin2->l-l1) < 2 && bg::intersects(element[e1], pinbox2))
            {
                intersection_pin(pin2x, pin2y, pin2->l, wirex, wirey, l1, iterPtx[e1], iterPty[e1], x, y);
                lastPtx[e1] = x;
                lastPty[e1] = y;


                into_array(min(iterPtx[e1], lastPtx[e1]), max(iterPtx[e1], lastPtx[e1]), 
                          min(iterPty[e1], lastPty[e1]), max(iterPty[e1], lastPty[e1]), xs, ys);
                vertical = localrtree.vertical(e1);

                if(localrtree.spacing_violations(bitid, xs, ys, l1, width[l1], spacing[l1], vertical))
                    c2 += SPACING_VIOLATION;


                if(minCost > c1 + c2)
                {
                    hasMinElem = true;
                    minCost = c1 + c2;
                    minElem = e1;
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
                continue;

            t1 = localrtree.trackid(e1);
            l1 = localrtree.layer(e1);

            // query intersected tracks
            queries.clear();
            trackrtree->query(bgi::intersects(element[e1]), back_inserter(queries));

            // Intersected available tracks
            for(auto& it : queries)
            {
                e2 = it.second;
                t2 = localrtree.trackid(e2);
                l2 = localrtree.layer(e2);
                maxWidth = localrtree.width(e2);
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
                if(maxWidth < width[l2])
                    continue;


                if(!isRef)
                {
                    // layer condition
                    if(tracelNum[dep] != l2)
                        continue;
                    
                    curDir = routing_direction(iterPtx[e1], iterPty[e1], x, y, localrtree.vertical(e1));
                    // if current routing direction is different,
                    // don't push into the priority queue
                    if(traceDir[depth[e1]] != curDir)
                        continue; 
                }

                // condition
                if((depth[e1] == 0) && 
                        (x == iterPtx[e1] && y == iterPty[e1]))
                    continue;


                // cost
                c1 = cost1 + manhatan_distance(iterPtx[e1], iterPty[e1], x, y) + abs(l1-l2) * VIA_COST + DEPTH_COST;
                c2 = cost2; //2 * (int)(bg::distance(pt(iterPtx[e2], iterPty[e2]), pinbox2));
                
                // check spacing violation
                into_array(min(iterPtx[e1], x), max(iterPtx[e1], x), min(iterPty[e1], y), max(iterPty[e1], y), xs, ys); 
                vertical = localrtree.vertical(e1);
                
                if(localrtree.spacing_violations(bitid, xs, ys, l1, width[l1], spacing[l1], vertical))
                    c2 += SPACING_VIOLATION;

                // if destination
                if(abs(pin2->l-l2) < 2 && bg::intersects(elem, pinbox2))
                {
                    destination = true;

                    lpt(elem, x1, y1);
                    upt(elem, x2, y2);
                    into_array(x1, x2, y1, y2, wirex, wirey);
                    intersection_pin(pin2x, pin2y, pin2->l, wirex, wirey, l2, x, y, lastPtx[e2], lastPty[e2]); 
                   
                    // condition(routing direction)
                    if(!isRef)
                    {
                        curDir = routing_direction(x, y, lastPtx[e2], lastPty[e2], localrtree.vertical(e2));
                        // if current routing direction is different,
                        // don't push into the priority queue
                        if(traceDir[dep] != curDir)
                            continue;
                    }
                    
                    
                    // check spacing violation
                    into_array(min(x, lastPtx[e2]), max(x, lastPtx[e2]), min(y, lastPty[e2]), max(y, lastPty[e2]), xs, ys);
                    vertical = localrtree.vertical(e2);

                    if(localrtree.spacing_violations(bitid, xs, ys, l2, width[l2], spacing[l2], vertical))
                        c2 += SPACING_VIOLATION;
                }
 

                // if previous cost is smaller, continue
                if(elemCost[e2] < c1 + c2)
                    continue;

                element[e2] = elem;
                depth[e2] = dep;
                backtrace[e2] = e1;
                iterPtx[e2] = x;
                iterPty[e2] = y;
                elemCost[e2] = c1 + c2;

                // if current element arrivals destination
                // compare minimum cost
                if(destination)
                {
                    if(minCost > c1 + c2)
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
            // initial element
            e2 = minElem;
            l2 = localrtree.layer(e2);
            
            
            // if reference routing, store
            if(isRef)
            {
                
                maxDepth = depth[e2];
                curDir = routing_direction(iterPtx[e2], iterPty[e2], lastPtx[e2], lastPty[e2], localrtree.vertical(e2));
                tracelNum.insert(tracelNum.begin(), l2);
                traceDir.insert(traceDir.begin(), curDir);

                // Create default wires
                created = vector<Wire>((maxDepth+1)*numbits);
            }
            
            // reverse count because of backtrace
            count = maxDepth;
            
            w2 = (maxDepth+1)*i + count--;
            trackid = localrtree.trackid(e2);
            pin = true;
            // data
            into_array(min(lastPtx[e2], iterPtx[e2]), max(lastPtx[e2], iterPtx[e2]), 
                       min(lastPty[e2], iterPty[e2]), max(lastPty[e2], iterPty[e2]), xs, ys);
            
            wirex[0] = xs[0];
            wirex[1] = xs[1];
            wirey[0] = ys[0];
            wirey[1] = ys[1];
            expand_width(wirex, wirey, width[l2], localrtree.vertical(e2));
            

            created[w2].get_info(bitid, trackid, xs, ys, l2, seq, pin);
            created[w2].add_intersection(PINTYPE, {lastPtx[e2], lastPty[e2]});
            // rtree update
            localrtree.insert_element(trackid, xs, ys, l2, false);
            localrtree.update_wire(bitid, wirex, wirey, l2, false);

           
            while(backtrace[e2] != e2)
            {
                // Iterating
                e1 = backtrace[e2];
                x1 = iterPtx[e1];
                y1 = iterPty[e1];
                x2 = iterPtx[e2];
                y2 = iterPty[e2];
                l1 = localrtree.layer(e1);
                vertical = localrtree.vertical(e1);

                if(isRef)
                {
                    tracelNum.insert(tracelNum.begin(), l1);
                    traceDir.insert(traceDir.begin(), routing_direction(x1, y1, x2, y2, vertical)); 
                }

                // data
                w1 = (maxDepth+1)*i + count--;
                pin = (e1 == backtrace[e1]) ? true : false;
                trackid = localrtree.trackid(e1);
                into_array(min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2), xs, ys);

                wirex[0] = xs[0];
                wirex[1] = xs[1];
                wirey[0] = ys[0];
                wirey[1] = ys[1];
                
                // rtree update
                expand_width(wirex, wirey, width[l1], localrtree.vertical(e1));
                created[w1].get_info(bitid, trackid, xs, ys, l1, seq, pin);
                localrtree.insert_element(trackid, xs, ys, l1, false);
                localrtree.update_wire(bitid, wirex, wirey, l1, false);

                if(pin)
                    created[w1].add_intersection(PINTYPE, {x1, y1});
                
                
                edges.push_back({w1, w2});
                pts.push_back({x2, y2});
                w2 = w1;
                e2 = e1;
            }


        }
        else
        {
            cout << "No solution..." << endl;
            solution = false;
            break;
        }
        //
        isRef = false;
    }
    
   
    //
    if(solution)
    {
        dense_hash_map<int,int> local2global;
        local2global.set_empty_key(INT_MAX);
       

        tp = vector<Segment>(maxDepth+1);
        for(i=0; i < numbits; i++)
        {  


            for(count = 0; count < maxDepth+1 ; count++)
            {
                index = i*(maxDepth+1) + count;
                wireid = wires.size();
                local2global[index] = wireid;
                
                Wire* curw = &created[index];
                into_array(curw->x1, curw->x2, curw->y1, curw->y2, xs, ys);
                l = curw->l;
                seq = curw->seq;
                pin = curw->pin;
                trackid = curw->trackid;
                bitid = curw->bitid;
                curw = CreateWire(bitid, trackid, xs, ys, l, seq, pin);


#ifdef DEBUG_MAZE2
                if(curw->id != wireid)
                {
                    printf("illegal...\n");
                    exit(0);

                }
                if(count==0)
                {
                    pin1 = &ckt->pins[p1[i]];
                    pin2 = &ckt->pins[p2[i]];
                    printf("%s p1 (%d %d) (%d %d) -> p2 (%d %d) (%d %d)\n", 
                            ckt->bits[curbus->bits[i]].name.c_str(),
                            pin1->llx, pin1->lly, pin1->urx, pin1->ury,
                            pin2->llx, pin2->lly, pin2->urx, pin2->ury);
                }
                printf("    %s (%d %d) (%d %d) M%d ",
                        ckt->bits[bitid].name.c_str(), xs[0], ys[0], xs[1], ys[1], l);
                
                if(count == 0)
                    printf("p1 %d -> wire %d", pin1->id, wireid);
                else if(count == maxDepth)
                    printf("p2 %d -> wire %d", pin2->id, wireid);
                
                printf("\n");


                if(pin1->id != p1[i] || pin2->id != p2[i])
                {
                    printf("no matching pins id...\n");
                    exit(0);
                }

#endif
                if(count == 0)
                {
                    pin2wire[p1[i]] = wireid;
                    wire2pin[wireid] = p1[i];
                    curw->add_intersection(PINTYPE, created[index].intersection[PINTYPE]);
                }
                
                if(count == maxDepth)
                {
                    pin2wire[p2[i]] = wireid;
                    wire2pin[wireid] = p2[i];
                    curw->add_intersection(PINTYPE, created[index].intersection[PINTYPE]);
                }
                
                tp[count].wires.push_back(wireid);
                tp[count].x1 = min(tp[count].x1, curw->x1);
                tp[count].y1 = min(tp[count].y1, curw->y1);
                tp[count].x2 = max(tp[count].x2, curw->x2);
                tp[count].y2 = max(tp[count].y2, curw->y2);
                tp[count].l = curw->l;
                tp[count].vertical = curw->vertical;
            }
        }
    
        for(count = 0; count < maxDepth+1; count++)
            sort(tp[count].wires.begin(), tp[count].wires.end(), [&,this](int left, int right){
                    return this->wires[left].seq < this->wires[right].seq;
                    });

        for(i=0; i < edges.size(); i++)
        {
            int w1 = local2global[edges[i].first];
            int w2 = local2global[edges[i].second];
            SetNeighbor(&wires[w1], &wires[w2], pts[i].first, pts[i].second);
        }
    }


    return solution; 
}



bool OABusRouter::Router::route_multipin_to_tp(int busid, int m, vector<Segment> &tp)
{

    printf("route multipin to tp\n");

    // Variables
    int i, j, wireid;
    int numwires, numpins;
    int numbits, cost;
    int x1, y1, x2, y2, x, y;
   
    int llx, lly, urx, ury;
    int l1, l2, curDir, dist;
    int bitid, trackid, l, seq;
    int maxWidth, align;
    int xs[2], ys[2];
    int wirex[2], wirey[2];
    int pinx[2], piny[2];
    int maxDepth = INT_MAX;
    int count, index;

    bool pin;
    bool vertical_align;
    bool vertical;
    bool solution = false;
    bool isRef;
    
    
    typedef PointBG pt;
    typedef SegmentBG seg;
    typedef BoxBG box;
    typedef tuple<int,int,int> ituple;
    typedef pair<int,int> ipair;

   
    dense_hash_map<int,int> width;
    dense_hash_map<int,int> sequence;
    sequence.set_empty_key(INT_MAX);

    Pin* curpin;
    Bus* curbus;
    Wire* tarwire;
    MultiPin* curmp;
    SegRtree* trackrtree;
    Circuit* circuit;

    Rtree localrtree(rtree);
    trackrtree = &localrtree.track;
    curbus = &ckt->buses[busid];
    curmp = &ckt->multipins[m];
    numbits = curbus->numBits;
    width = curbus->width;
    circuit = ckt;
    align = curmp->align;
    numpins = curmp->pins.size();

    llx = multipin2llx[m];
    lly = multipin2lly[m];
    urx = multipin2urx[m];
    ury = multipin2ury[m];
    l = curmp->l;
    vertical_align = (curmp->align == VERTICAL)? true : false;

    for(i = 0; i < numpins; i++)
    {
        sequence[curmp->pins[i]] = i;
    }
    
    
    auto cmp1 = [&,llx,lly,urx,ury,l](const Segment& left, const Segment& right){
        int dist1 = 
            (int)(bg::distance(box(pt(llx,lly), pt(urx,ury)), box(pt(left.x1, left.y1), pt(left.x2, left.y2)))) 
            + abs(l-left.l)*VIA_COST;

        int dist2 = 
            (int)(bg::distance(box(pt(llx,lly), pt(urx,ury)), box(pt(right.x1, right.y1), pt(right.x2, right.y2)))) 
            + abs(l-right.l)*VIA_COST;
        return dist1 > dist2;
    };

    priority_queue<Segment, vector<Segment>, decltype(cmp1)> PQ1(cmp1);
    
    // segment list
    for(auto seg : tp)
        PQ1.push(seg);
    

    while(PQ1.size() > 0)
    {
        Segment target = PQ1.top();
        PQ1.pop();
        
        vector<Segment> topology;
        vector<Wire> created;
        // element { local id, local id } 
        vector<pair<int, int>> local_edges;
        vector<pair<int, int>> local_pts;
        // element { local id, global id }
        vector<pair<int, int>> global_edges;
        vector<pair<int, int>> global_pts;


        int lastRoutingShape;        
        vector<int> sorted;
        vector<int> tracelNum;  // trace layer number
        vector<int> traceDir;   // trace direction
        sorted = curmp->pins;

        maxDepth = INT_MAX;
        solution = true;
        isRef = true;

        llx = target.x1;
        lly = target.y1;
        urx = target.x2;
        ury = target.y2;

        // Ordering
        if(vertical_align)
        {
            if(ury < multipin2lly[curmp->id])
            {
                sort(sorted.begin(), sorted.end(), [&,circuit](int left, int right){
                        return circuit->pins[left].lly > circuit->pins[right].lly;
                        });
            }
            else if(lly > multipin2ury[curmp->id])
            {
                sort(sorted.begin(), sorted.end(), [&,circuit](int left, int right){
                        return circuit->pins[left].lly < circuit->pins[right].lly;
                        });
            }
        }
        else
        {
            if(urx < multipin2llx[curmp->id])
            {
                sort(sorted.begin(), sorted.end(), [&,circuit](int left, int right){
                        return circuit->pins[left].llx > circuit->pins[right].llx;
                        });
            }
            else if(llx > multipin2urx[curmp->id])
            {
                sort(sorted.begin(), sorted.end(), [&,circuit](int left, int right){
                        return circuit->pins[left].llx < circuit->pins[right].llx;
                        });
            }
        }


        //
        for(i=0; i < numpins; i++)
        {
            seq = sequence[sorted[i]];
            curpin = &ckt->pins[sorted[i]];
            tarwire = &wires[target.wires[seq]];
            bitid = curbus->bits[seq];

            box pinbox;
            seg wireseg(pt(tarwire->x1, tarwire->y1), pt(tarwire->x2, tarwire->y2));
            // get 
            into_array(curpin->llx, curpin->urx, curpin->lly, curpin->ury, pinx, piny);
            pin_area(pinx, piny, align, width[curpin->l], pinbox);
            
            // variables
            seg elem;
            int dep;
            int w1, w2;
            int e1, e2, t1, t2;
            int c1, c2;
            int xDest, yDest;
            int minElem = INT_MAX;
            int minCost = INT_MAX;
            bool hasMinElem = false;
            bool destination;
            vector<int> backtrace(localrtree.elemindex, -1);
            vector<int> depth(localrtree.elemindex, -1);
            vector<int> elemCost(localrtree.elemindex, INT_MAX);
            vector<int> iterPtx(localrtree.elemindex, -1);
            vector<int> iterPty(localrtree.elemindex, -1);
            vector<int> lastPtx(localrtree.elemindex, -1);
            vector<int> lastPty(localrtree.elemindex, -1);
            vector<seg> element(localrtree.elemindex);
            vector<pair<seg, int>> queries;

            auto cmp2 = [](const ituple &left, const ituple &right){
                return (get<1>(left) + get<2>(left) > get<1>(right) + get<2>(right));
            };
            priority_queue<ituple , vector<ituple>, decltype(cmp2)> PQ2(cmp2);

            queries.clear();
            trackrtree->query(bgi::intersects(pinbox), back_inserter(queries));

            // Initial candidates
            for(auto& it : queries)
            {
                e1 = it.second;
                t1 = localrtree.trackid(e1);
                l1 = localrtree.layer(e1);
                maxWidth = localrtree.width(e1);

                // width constraint
                if(maxWidth < width[l1])
                    continue;

                //
                if(vertical_align == localrtree.vertical(e1))
                    continue;

                if(abs(curpin->l - l1) > 1)
                    continue;

                // visit
                backtrace[e1] = e1; // root
                element[e1] = it.first;
                depth[e1] = 0;

                // get point
                lpt(element[e1], x1, y1);
                upt(element[e1], x2, y2);

                // Find interating point of e1
                into_array(min(x1,x2), max(x1,x2), min(y1,y2), max(y1,y2), xs, ys);
                intersection_pin(pinx, piny, curpin->l, xs, ys, l1, -1, -1, x, y);

                iterPtx[e1] = x;
                iterPty[e1] = y;

                c1 = VIA_COST * abs(curpin->l - l1);
                c2 = 0;

                if(abs(tarwire->l - l1) < 2 && bg::intersects(element[e1],wireseg))
                {
                    intersection(wireseg, element[e1], x, y);
                    lastPtx[e1] = x;
                    lastPty[e1] = y;

                    into_array(min(iterPtx[e1], lastPtx[e1]), max(iterPtx[e1], lastPtx[e1]),
                            min(iterPty[e1], lastPty[e1]), max(iterPty[e1], lastPty[e1]), xs, ys);
                    vertical = localrtree.vertical(e1);

                    if(localrtree.spacing_violations(bitid, xs, ys, l1, width[l1], spacing[l1], vertical))
                        c2 += SPACING_VIOLATION;

                    if(minCost > c1 + c2)
                    {
                        hasMinElem = true;
                        minCost = c1 + c2;
                        minElem = e1;
                    }

                }

                //
                PQ2.push(make_tuple(e1, c1, c2));
                elemCost[e1] = c1 + c2;
            }



            //
            while(PQ2.size() > 0)
            {
                int cost1, cost2;
                ituple e = PQ2.top();
                PQ2.pop();

                e1 = get<0>(e);
                cost1 = get<1>(e);
                cost2 = get<2>(e);

                if(hasMinElem && (e1 == minElem))
                    break;

                // depth condition
                if(maxDepth <= depth[e1])
                    continue;

                t1 = localrtree.trackid(e1);
                l1 = localrtree.layer(e1);

                // query
                queries.clear();
                trackrtree->query(bgi::intersects(element[e1]), back_inserter(queries));

                //
                for(auto& it : queries)
                {
                    e2 = it.second;
                    t2 = localrtree.trackid(e2);
                    l2 = localrtree.layer(e2);
                    maxWidth = localrtree.width(e2);
                    elem = it.first;
                    dep = depth[e1] + 1;
                    destination = false;

                    if(e1 == e2)
                        continue;
                    
                    if(abs(l1-l2) > 1)
                        continue;
         
                    if(maxWidth < width[l2])
                        continue;

                    // intersection
                    intersection(element[e1], elem, x, y);

                    if(!isRef)
                    {
                        // layer sequence
                        if(tracelNum[dep] != l2)
                            continue;

                        curDir = routing_direction(iterPtx[e1], iterPty[e1], x, y, localrtree.vertical(e1));
                        if(traceDir[depth[e1]] != curDir)
                            continue;
                    }
                

                    if((depth[e1] == 0) && (x == iterPtx[e1] && y == iterPty[e1]))
                        continue;

                    // cost
                    c1 = cost1 + manhatan_distance(iterPtx[e1], iterPty[e1], x, y) + abs(l1-l2) * VIA_COST + DEPTH_COST;
                    c2 = cost2;

                    // check spacing violation
                    into_array(min(iterPtx[e1], x), max(iterPtx[e1], x), min(iterPty[e1], y), max(iterPty[e1], y), xs, ys);
                    vertical = localrtree.vertical(e1);

                    if(localrtree.spacing_violations(bitid, xs, ys, l1, width[l1], spacing[l1], vertical))
                        c2 += SPACING_VIOLATION;

                    // if destination
                    if(abs(tarwire->l - l2) < 2 && bg::intersects(elem, wireseg))
                    {
                        destination = true;

                        intersection(elem, wireseg, lastPtx[e2], lastPty[e2]);
                        into_array(min(x, lastPtx[e2]), max(x, lastPtx[e2]), min(y, lastPty[e2]), max(y, lastPty[e2]), xs, ys);
                        vertical = localrtree.vertical(e2);
                        if(!isRef)
                        {
                            curDir = routing_direction(x, y, lastPtx[e2], lastPty[e2], vertical);

                            if(traceDir[dep] != curDir)
                                continue;
                        }
    
                        if(localrtree.spacing_violations(bitid, xs, ys, l2, width[l2], spacing[l2], vertical))
                            c2 += SPACING_VIOLATION;

                    }

                    if(elemCost[e2] < c1 + c2)
                        continue;

                    element[e2] = elem;
                    depth[e2] = dep;
                    backtrace[e2] = e1;
                    iterPtx[e2] = x;
                    iterPty[e2] = y;
                    elemCost[e2] = c1 + c2;

                    //
                    if(destination)
                    {
                        if(minCost > c1 + c2)
                        {
                            hasMinElem = true;
                            minCost = c1 + c2;
                            minElem = e2;
                        }
                    }

                    PQ2.push(make_tuple(e2, c1, c2));
                }
                //
            }


            if(hasMinElem)
            {
                e2 = minElem;
                l2 = localrtree.layer(e2);

                // if reference routing, store
                if(isRef)
                {
                    maxDepth = depth[e2];

                    if(lastPtx[e2] == tarwire->x1 && lastPty[e2] == tarwire->y1)
                        lastRoutingShape = EndPointLL;        
                    else if(lastPtx[e2] == tarwire->x2 && lastPty[e2] == tarwire->y2)
                        lastRoutingShape = EndPointUR;
                    else
                        lastRoutingShape = T_Junction;

                    curDir = routing_direction(iterPtx[e2], iterPty[e2], lastPtx[e2], lastPty[e2], localrtree.vertical(e2));
                    tracelNum.insert(tracelNum.begin(), l2);
                    traceDir.insert(traceDir.begin(), curDir);
                
                    created = vector<Wire>((maxDepth+1)*numbits);
                }

                count = maxDepth;

                w2 = (maxDepth+1)*seq + count--;
                trackid = localrtree.trackid(e2);
                pin = (backtrace[e2] == e2)? true : false;

                //
                into_array(min(lastPtx[e2], iterPtx[e2]), max(lastPtx[e2], iterPtx[e2]),
                           min(lastPty[e2], iterPty[e2]), max(lastPty[e2], iterPty[e2]), xs, ys);

                wirex[0] = xs[0];
                wirex[1] = xs[1];
                wirey[0] = ys[0];
                wirey[1] = ys[1];
                expand_width(wirex, wirey, width[l2], localrtree.vertical(e2));

                created[w2].get_info(bitid, trackid, xs, ys, l2, seq, pin);
                if(pin)
                    created[w2].add_intersection(PINTYPE, {iterPtx[e2], iterPty[e2]});

                //
                localrtree.insert_element(trackid, xs, ys, l2, false);
                localrtree.update_wire(bitid, wirex, wirey, l2, false);
                //
                global_edges.push_back({w2, tarwire->id});
                global_pts.push_back({lastPtx[e2], lastPty[e2]});

                while(backtrace[e2] != e2)
                {
                    e1 = backtrace[e2];
                    x1 = iterPtx[e1];
                    y1 = iterPty[e1];
                    x2 = iterPtx[e2];
                    y2 = iterPty[e2];
                    l1 = localrtree.layer(e1);
                    vertical = localrtree.vertical(e1);

                    if(isRef)
                    {
                        tracelNum.insert(tracelNum.begin(), l1);
                        traceDir.insert(traceDir.begin(), routing_direction(x1, y1, x2, y2, vertical));
                    }

                    // data
                    w1 = (maxDepth+1)*seq + count--;
                    pin = (e1 == backtrace[e1]) ? true : false;
                    trackid = localrtree.trackid(e1);
                    into_array(min(x1,x2), max(x1,x2), min(y1,y2), max(y1,y2), xs, ys);

                    wirex[0] = xs[0];
                    wirex[1] = xs[1];
                    wirey[0] = ys[0];
                    wirey[1] = ys[1];
                    expand_width(wirex, wirey, width[l1], localrtree.vertical(e1));

                    created[w1].get_info(bitid, trackid, xs, ys, l1, seq, pin);
                    localrtree.insert_element(trackid, xs, ys, l1, false);
                    localrtree.update_wire(bitid, wirex, wirey, l1, false);

                    if(pin)
                        created[w1].add_intersection(PINTYPE, {x1, y1});

                    local_edges.push_back({w1,w2});
                    local_pts.push_back({x2,y2});
                    w2 = w1;
                    e2 = e1;
                }
            }
            else
            {
                solution = false;
                break;
            }
            //
            isRef = false;
        }
        
        
        if(solution)
        {
            dense_hash_map<int,int> local2global;
            local2global.set_empty_key(INT_MAX);

            topology = vector<Segment>(maxDepth+1);

            for(i=0; i < numbits; i++)
            {   
                for(count = 0; count < maxDepth+1 ; count++)
                {
                    index = i*(maxDepth+1) + count;
                    wireid = wires.size();
                    local2global[index] = wireid;

                    Wire* curw = &created[index];
                    into_array(curw->x1, curw->x2, curw->y1, curw->y2, xs, ys);
                    l = curw->l;
                    seq = curw->seq;
                    pin = curw->pin;
                    trackid = curw->trackid;
                    bitid = curw->bitid;

                    curw = CreateWire(bitid, trackid, xs, ys, l, seq, pin);

                    if(count == 0)
                    {
                        pin2wire[curpin->id] = wireid;
                        wire2pin[wireid] = curpin->id;
                        curw->add_intersection(PINTYPE, created[index].intersection[PINTYPE]);
                    }

                    topology[count].wires.push_back(wireid);
                    topology[count].x1 = min(topology[count].x1, curw->x1);
                    topology[count].y1 = min(topology[count].y1, curw->y1);
                    topology[count].x2 = max(topology[count].x2, curw->x2);
                    topology[count].y2 = max(topology[count].y2, curw->y2);
                    topology[count].l = curw->l;
                    topology[count].vertical = curw->vertical;
                }
            }

            for(count = 0; count < maxDepth+1; count++)
                sort(topology[count].wires.begin(), topology[count].wires.end(), [&,this](int left, int right){
                        return this->wires[left].seq < this->wires[right].seq;
                        });

            for(i=0; i < local_edges.size(); i++)
            {
                int w1 = local2global[local_edges[i].first];
                int w2 = local2global[local_edges[i].second];
                SetNeighbor(&wires[w1], &wires[w2], local_pts[i].first, local_pts[i].second);
            }

            for(i=0; i < global_edges.size(); i++)
            {
                int w1 = local2global[global_edges[i].first];
                int w2 = global_edges[i].second;
                SetNeighbor(&wires[w1], &wires[w2], global_pts[i].first, global_pts[i].second);
            }


            tp.insert(tp.end(), topology.begin(), topology.end());
            
            return true;
        }
        //
    }




    return false;
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








