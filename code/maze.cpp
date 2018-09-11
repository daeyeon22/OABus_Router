
#include "func.h"
#include "circuit.h"
#include "route.h"
#include "rtree.h"

#include <stdlib.h>
#include <time.h>
#include <unordered_map>
#include <tuple>

#define MAX_ITERATION_COUNT 10
#define VIA_COST 1000 
#define DEPTH_COST 5000
#define SPACING_VIOLATION max(ckt->width + ckt->height,100000)
#define NOTCOMPACT 10000
#define DESTINATION -12311
//#define DESTINATION SIZE_MAX

#define DELTA ckt->delta
#define EPSILON ckt->epsilon
//#define DEBUG_ROUTE_MP_TO_TP
//#define DEBUG_ROUTE_TWOPIN_NET
//#define DEBUG_MAZE
// Routing direction
typedef PointBG pt;
typedef SegmentBG seg;
typedef BoxBG box;

static int randseed = 777;
static int failed = 0;
static int failed_tw = 0;;
static int failed_tp = 0;

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

int routing_direction(int x1, int y1, int x2, int y2, bool vertical)
{
    if(vertical && (x1 != x2))
    {
        cout << "invalid routing direction..." << endl;
        cout << x1 << " " << y1 << " " << x2 << " " << y2 << endl;
        exit(0);
    }

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
    //if(x1 != x2 && y1 != y2)
    //{
    //    printf("invalid into array ... (%d %d) (%d %d)\n", x1, y1, x2, y2);
    //    exit(0);
    //}


    x[0] = x1;
    x[1] = x2;
    y[0] = y1;
    y[1] = y2;
}


void OABusRouter::Wire::get_info(int b, int t, int x[], int y[], int curl, int s, bool accessPin)
{
    if(x[0] != x[1] && y[0] != y[1])
    {
        printf("invalid get info ... (%d %d) (%d %d)\n", x[0], y[0], x[1], y[1]);
        exit(0);

    }
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
        y[0] -= width/2;
        y[1] += width/2;
    }
    else
    {
        x[0] -= width/2;
        x[1] += width/2;
    }
    
    pb = box(pt(x[0], y[0]), pt(x[1], y[1]));
}
void expand_width(int x[], int y[], int width, bool vertical)
{
    if(vertical)
    {
        x[0] -= width/2;
        x[1] += width/2;
    }
    else
    {
        y[0] -= width/2;
        y[1] += width/2;
    }
}


void OABusRouter::Router::route_all()
{
    int total_buses = ckt->buses.size();
    int total_success = 0;
    //gen_backbone();
    vector<int> sorted;
    for(auto& b : ckt->buses)
        sorted.push_back(b.id);


    Circuit* circuit = ckt;
    sort(sorted.begin(), sorted.end(), [&,circuit](int left, int right){
            int x1 = circuit->buses[left].llx; //(int)( 1.0 * ( circuit->buses[left].llx + circuit->buses[left].urx ) / 2 );
            int y1 = circuit->buses[left].lly; //(int)( 1.0 * ( circuit->buses[left].lly + circuit->buses[left].ury ) / 2 );
            int x2 = circuit->buses[right].llx; //(int)( 1.0 * ( circuit->buses[right].llx + circuit->buses[right].urx ) / 2 );
            int y2 = circuit->buses[right].lly; //(int)( 1.0 * ( circuit->buses[right].lly + circuit->buses[right].ury ) / 2 );
            int numBits1 = circuit->buses[left].numBits;
            int numBits2 = circuit->buses[right].numBits;
            return (y1 > y2); // || ((y1 == y2) && (x1 < x2));
            });

    cout << "< Sorted Bus Sequence >\n" << endl;
    for(auto& busid : sorted)
    {
        Bus* curbus = &ckt->buses[busid];
        printf("%s (%d %d) (%d %d)\n", curbus->name.c_str(), curbus->llx, curbus->lly, curbus->urx, curbus->ury);
    }
    cout << endl;


    for(auto& busid : sorted){
        Bus* curbus = &ckt->buses[busid];
        
        // check elapse time and runtime limit
        if(should_stop())
        {
            curbus->assign = false;
            break;
        }

        if(route_bus(busid))
        {
            curbus->assign = true;
            total_success++;
        }
        else
        {
            curbus->assign = false;
            remove_all(busid);
            cout << "[INFO] " << curbus->name << " routing failed" << endl;
        }
        
    }

    printf("- - - - - - - - - - - - - - - - -\n");
    printf("[INFO] # failed tw         : %d\n", failed_tw);
    printf("[INFO] # failed tp         : %d\n", failed_tp);
    printf("[INFO] # routing success   : %d\n", total_success);
    printf("[INFO] # routing failed    : %d\n", total_buses - total_success);
    printf("- - - - - - - - - - - - - - - - -\n");
    
}

bool OABusRouter::Router::route_bus(int busid)
{
    bool routing_success = true;
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

    //////
    sort(comb.begin(), comb.end(), [&,this](pair<int,int> left, pair<int,int> right){
            box b1, b2;
            float dist1, dist2;
            int l1, l2;
            int dx, dy;
            b1 = box(pt(this->multipin2llx[left.first], this->multipin2lly[left.first])
                        ,pt(this->multipin2urx[left.first], this->multipin2ury[left.first]));
            b2 = box(pt(this->multipin2llx[left.second], this->multipin2lly[left.second])
                        ,pt(this->multipin2urx[left.second], this->multipin2ury[left.second]));
            dist1 = bg::distance(b1,b2);
            //dx = this->multipin2llx[left.first] + this->multipin2urx[left.first]
            //    - this->multipin2llx[left.second] - this->multipin2urx[left.second];
            //dy = this->multipin2lly[left.first] + this->multipin2ury[left.first]
            //    - this->multipin2lly[left.second] - this->multipin2ury[left.second];
            //l1 = min(abs(dx),abs(dy));


            b1 = box(pt(this->multipin2llx[right.first], this->multipin2lly[right.first])
                        ,pt(this->multipin2urx[right.first], this->multipin2ury[right.first]));
            b2 = box(pt(this->multipin2llx[right.second], this->multipin2lly[right.second])
                        ,pt(this->multipin2urx[right.second], this->multipin2ury[right.second]));
            dist2 = bg::distance(b1,b2);
            //dx = this->multipin2llx[right.first] + this->multipin2urx[right.first]
            //    - this->multipin2llx[right.second] - this->multipin2urx[right.second];
            //dy = this->multipin2lly[right.first] + this->multipin2ury[right.first]
            //    - this->multipin2lly[right.second] - this->multipin2ury[right.second];
            //l2 = min(abs(dx),abs(dy));
            //return l1 > l2;
            return dist1 > dist2;
            });


    // loop for route_two_pin_net
    while(comb.size() > 0)
    {

        // check elapse time and runtime limit
        if(should_stop())
        {
            routing_success = false;
            break;
        }

        candi = *comb.begin();
        comb.erase(comb.begin());
        //candi = pop_random(comb);
        mp1 = candi.first;
        mp2 = candi.second;
        // pick two multipins for routing
        // routing topologies
        vector<Segment> tp;

        cout << "[INFO] start route twopin net" << endl;
        if(route_twopin_net_v6(busid, mp1, mp2, tp))
        {

            cout << "[INFO] success route twopin net" << endl;
            cout << "[INFO] start wire re-ordering" << endl;
            wire_reordering(busid, tp);
            
            routing_success = true;
            mps.erase(find(mps.begin(), mps.end(), mp1));
            mps.erase(find(mps.begin(), mps.end(), mp2));

            // route until no remained multipins
            while(mps.size() > 0)
            {

                // check elapse time and runtime limit
                if(should_stop())
                {
                    routing_success = false;
                    break;
                }
                
                mp1 = pop_random(mps);
                // route target multipin to net topologies (tp)
                
                
                cout << "[INFO] start route mp to tp" << endl;
                if(!route_multipin_to_tp(busid, mp1, tp))
                {

                    // rip-up
                    //congested = get_congested_bus(busid);
                    //rip_up(congested);
                    //reroutes.push_back(congested);
                    // re-try
                    //mps.push_back(mp1);
                    //break;
                    routing_success = false;
                    break;
                }
                cout << "[INFO] finished route mp to tp" << endl;
            }

            
            // reroute rip-upped buses
            for(int i=0; i < reroutes.size() ; i++)
            {
                // if failed again
                if(!reroute(reroutes[i]))
                {
                    // do something
                    //return false;
                    routing_success = false;
                    break;
                }
            }

            // update to global structure
            update_net_tp(busid, tp);
            break;
            // routing success
            //return true;
        }
        else
        {
            routing_success = false;
        }
    }


    // routing fail
    return routing_success;
}


void OABusRouter::Router::sort_pins_routing_sequence(int m1, int m2, bool reverse, vector<int>& sorted1, vector<int>& sorted2)
{
    // variables
    int align1, align2, numpins, i;
    int bbllx, bblly, bburx, bbury;
    bool left, below, intersect_h, intersect_v;
    dense_hash_map<int,int> bit2pin;
    bit2pin.set_empty_key(INT_MAX);
    MultiPin *mp1, *mp2;
    Circuit* cir = ckt;
    mp1 = &ckt->multipins[m1];
    mp2 = &ckt->multipins[m2];
    align1 = mp1->align;
    align2 = mp2->align;
    numpins = mp1->pins.size();

    for(i=0; i < numpins; i++)
    {
        bit2pin[pin2bit[mp2->pins[i]]] = mp2->pins[i];
    }
    
    //
    sorted1.clear();
    sorted2.clear();
    sorted1 = mp1->pins;
    left = multipin2llx[m1] < multipin2llx[m2] ? true : false;
    below = multipin2lly[m1] < multipin2lly[m2] ? true : false;
    intersect_h = bi::intersects(IntervalT::closed(multipin2llx[m1], multipin2urx[m1]), IntervalT::closed(multipin2llx[m2], multipin2urx[m2]));
    intersect_v = bi::intersects(IntervalT::closed(multipin2lly[m1], multipin2ury[m1]), IntervalT::closed(multipin2lly[m2], multipin2ury[m2]));
   
    bbllx = min(multipin2llx[m1], multipin2llx[m2]);
    bblly = min(multipin2lly[m1], multipin2lly[m2]);
    bburx = max(multipin2urx[m1], multipin2urx[m2]);
    bbury = max(multipin2ury[m1], multipin2ury[m2]);


    if(align1 == align2)
    {
        if(align1 == VERTICAL && align2 == VERTICAL)
        {
            if(intersect_v)
                
                sort(sorted1.begin(), sorted1.end(), [&,below,cir](int p1, int p2){
                    return  below ? (cir->pins[p1].lly > cir->pins[p2].lly) : (cir->pins[p1].lly < cir->pins[p2].lly); });
            else
            {
                sort(sorted1.begin(), sorted1.end(), [&,cir,below,bblly,bbury](int p1, int p2){
                    int dist1 = (below) ? abs(bblly - cir->pins[p1].lly) : abs(bbury - cir->pins[p1].ury);
                    int dist2 = (below) ? abs(bblly - cir->pins[p2].lly) : abs(bbury - cir->pins[p2].ury);
                    return dist1 < dist2; });
            }    
        }
        
        if(align1 == HORIZONTAL && align2 == HORIZONTAL)
        {
            if(intersect_h)
                sort(sorted1.begin(), sorted1.end(), [&,left,cir](int p1, int p2){
                    return  left ? (cir->pins[p1].llx > cir->pins[p2].llx) : (cir->pins[p1].llx < cir->pins[p2].llx); });
            else
                 sort(sorted1.begin(), sorted1.end(), [&,cir,left,bbllx, bburx](int p1, int p2){
                    int dist1 = (left) ? abs(bbllx - cir->pins[p1].llx) : abs(bburx - cir->pins[p1].urx);
                    int dist2 = (left) ? abs(bbllx - cir->pins[p2].llx) : abs(bburx - cir->pins[p2].urx);
                    return dist1 < dist2; });
        }
    }
  
    if(reverse)
    {
        vector<int> tmp;
        tmp.insert(tmp.end(), sorted1.rbegin(), sorted1.rend());
        sorted1 = tmp;
    }

    for(auto& pinid : sorted1)
    {
        sorted2.push_back(bit2pin[pin2bit[pinid]]);
    }
}


void OABusRouter::Router::sort_pins_routing_sequence(int m1, int m2, vector<int>& sorted1, vector<int>& sorted2)
{
    // variables
    int align1, align2, numpins, i;
    int bbllx, bblly, bburx, bbury;
    bool left, below, intersect_h, intersect_v;
    dense_hash_map<int,int> bit2pin;
    bit2pin.set_empty_key(INT_MAX);
    MultiPin *mp1, *mp2;
    Circuit* cir = ckt;
    mp1 = &ckt->multipins[m1];
    mp2 = &ckt->multipins[m2];
    align1 = mp1->align;
    align2 = mp2->align;
    numpins = mp1->pins.size();

    for(i=0; i < numpins; i++)
    {
        bit2pin[pin2bit[mp2->pins[i]]] = mp2->pins[i];
    }
    
    //
    sorted1.clear();
    sorted2.clear();
    sorted1 = mp1->pins;
    left = multipin2llx[m1] < multipin2llx[m2] ? true : false;
    below = multipin2lly[m1] < multipin2lly[m2] ? true : false;
    intersect_h = bi::intersects(IntervalT::closed(multipin2llx[m1], multipin2urx[m1]), IntervalT::closed(multipin2llx[m2], multipin2urx[m2]));
    intersect_v = bi::intersects(IntervalT::closed(multipin2lly[m1], multipin2ury[m1]), IntervalT::closed(multipin2lly[m2], multipin2ury[m2]));
   
    bbllx = min(multipin2llx[m1], multipin2llx[m2]);
    bblly = min(multipin2lly[m1], multipin2lly[m2]);
    bburx = max(multipin2urx[m1], multipin2urx[m2]);
    bbury = max(multipin2ury[m1], multipin2ury[m2]);


    if(align1 == align2)
    {
        if(align1 == VERTICAL && align2 == VERTICAL)
        {
            if(intersect_v)
                
                sort(sorted1.begin(), sorted1.end(), [&,below,cir](int p1, int p2){
                    return  below ? (cir->pins[p1].lly > cir->pins[p2].lly) : (cir->pins[p1].lly < cir->pins[p2].lly); });
            else
            {
                sort(sorted1.begin(), sorted1.end(), [&,cir,below,bblly,bbury](int p1, int p2){
                    int dist1 = (below) ? abs(bblly - cir->pins[p1].lly) : abs(bbury - cir->pins[p1].ury);
                    int dist2 = (below) ? abs(bblly - cir->pins[p2].lly) : abs(bbury - cir->pins[p2].ury);
                    return dist1 < dist2; });
            }    
        }
        
        if(align1 == HORIZONTAL && align2 == HORIZONTAL)
        {
            if(intersect_h)
                sort(sorted1.begin(), sorted1.end(), [&,left,cir](int p1, int p2){
                    return  left ? (cir->pins[p1].llx > cir->pins[p2].llx) : (cir->pins[p1].llx < cir->pins[p2].llx); });
            else
                 sort(sorted1.begin(), sorted1.end(), [&,cir,left,bbllx, bburx](int p1, int p2){
                    int dist1 = (left) ? abs(bbllx - cir->pins[p1].llx) : abs(bburx - cir->pins[p1].urx);
                    int dist2 = (left) ? abs(bbllx - cir->pins[p2].llx) : abs(bburx - cir->pins[p2].urx);
                    return dist1 < dist2; });
        }
    }
   
    for(auto& pinid : sorted1)
    {
        sorted2.push_back(bit2pin[pin2bit[pinid]]);
    }
}

bool is_inside(int x, int y, int ll[], int ur[])
{
    return ((ll[0] <= x && x <= ur[0]) && (ll[1] <= y && y <= ur[1])) ? true : false;
}

void OABusRouter::Router::local_search_area(int m1, int m2, int count, int ll[], int ur[])
{
    if(count == MAX_ITERATION_COUNT)
    {
        ll[0] = ckt->originX;
        ll[1] = ckt->originY;
        ur[0] = ckt->originX + ckt->width;
        ur[1] = ckt->originY + ckt->height;
    }
    else
    {
        int i, numlayers, numbits;
        int maxWidth, maxSpacing;
        int expand;
        Bus* curbus;
        MultiPin *mp1, *mp2;
        mp1 = &ckt->multipins[m1];
        mp2 = &ckt->multipins[m2];
        curbus = &ckt->buses[mp1->busid];
        numlayers = ckt->layers.size();
        numbits = curbus->numBits;
        maxWidth = INT_MIN;
        maxSpacing = INT_MIN;


        for(i=0; i < numlayers; i++)
        {
            maxWidth = max(maxWidth, curbus->width[i]);
            maxSpacing = max(maxSpacing, spacing[i]);
        }
        expand = (maxWidth + maxSpacing)*numbits*(count+1)*20;
        ll[0] = min(multipin2llx[m1], multipin2llx[m2]) - expand;
        ll[1] = min(multipin2lly[m1], multipin2lly[m2]) - expand;
        ur[0] = max(multipin2urx[m1], multipin2urx[m2]) + expand;
        ur[1] = max(multipin2ury[m1], multipin2ury[m2]) + expand;
        ll[0] = max(ckt->originX, ll[0]);
        ll[1] = max(ckt->originY, ll[1]);
        ur[0] = min(ckt->originX + ckt->width, ur[0]);
        ur[1] = min(ckt->originY + ckt->height, ur[1]);
    }
}


bool OABusRouter::Router::route_twopin_net_v6(int busid, int m1, int m2, vector<Segment> &tp)
{
    // check elapse time and runtime limit
    if(should_stop())
        return false;
    
    typedef PointBG pt;
    typedef SegmentBG seg;
    typedef BoxBG box;
    typedef tuple<int,int,int,int> ituple;
    // Variables
    int i, j, wireid, bitid, trackid, l, seq;
    int numwires, numpins, numbits, count, index;
    int cost, c1, c2, c3, e1, e2, w1, w2, t1, t2, l1, l2;
    int wirelength, estimated, penalty;
    int x1, y1, x2, y2, x3, y3, x, y, dep1, dep2, dep;
    int minElem, minCost, totalSPV, numDestSPV, num_spacing_vio;
    int sx1, sx2, sy1, sy2;
    int llx, lly, urx, ury;
    int curDir, prevDir, maxDepth;
    int maxWidth, align1, align2, minPanelty;
    int local_area_ll[2], local_area_ur[2];
    int xs[2], ys[2], wirex[2], wirey[2];
    int pin1x[2], pin1y[2], pin2x[2], pin2y[2];
    int mx1[2], my1[2], mx2[2], my2[2];
    
    bool pin, isRef, hasMinElem, isDestination;
    bool vertical_arrange1, vertical_arrange2;
    bool vertical, vertical1, vertical2;
    bool solution = true;

    // Element index ID, Wire length, Estimated Distance, Penalty
    vector<int> sorted1, sorted2;
    dense_hash_map<int,int> width;
    dense_hash_map<int,int> sequence;
    sequence.set_empty_key(INT_MAX);
    
    Pin *pin1, *pin2;
    Bus* curbus;
    MultiPin *mp1, *mp2;

    //Rtree localrtree(rtree);
    mp1 = &ckt->multipins[m1];
    mp2 = &ckt->multipins[m2];
            
    if(multipin2llx[mp1->id] > multipin2llx[mp2->id])
    {
        swap(mp1, mp2);
        swap(m1,m2);
    }

    curbus = &ckt->buses[busid];
    numbits = curbus->numBits;
    width = curbus->width;
    int visit_count=0;
    int failed_count=0;
    int iterCount=0;

    while(iterCount < 4)
    {
        // check elapse time and runtime limit
        if(should_stop())
            return false;

        
        if(iterCount / 2 == 1)
        {
            swap(mp1, mp2);
            swap(m1,m2);
        }

        if(iterCount % 2 == 1)
            sort_pins_routing_sequence(m1, m2, true, sorted1, sorted2);
        else
            sort_pins_routing_sequence(m1, m2, false, sorted1, sorted2);

        iterCount++;

        //sort_pins_routing_sequence(m1, m2, sorted1, sorted2);
        align1 = mp1->align;
        align2 = mp2->align;
        for(i=0; i < numbits; i++)
        {
#ifdef DEBUG_ROUTE_TWOPIN_NET
            printf("p%d -> p%d seq %d bitid %d\n", mp1->pins[i], mp2->pins[i], i, curbus->bits[i]);
#endif
            sequence[mp1->pins[i]] = i;
        }

        
        // get copied rtree
        tp.clear();
        maxDepth = INT_MAX;
        isRef = true;
        solution = true;
        vector<int> tracelNum, traceDir;  // trace layer number
        vector<int> tracelx, tracely, traceux, traceuy;
        vector<int> tracePtx, tracePty;
        vector<Wire> created;
        vector<pair<int, int>> edges;
        vector<pair<int, int>> pts;
        TrackRtree local_rtree_t(rtree_t);
        ObstacleRtree local_rtree_o(rtree_o);

        // local search area
        if(iterCount % 2 == 1)
        { 
            local_area_ll[0] = ckt->originX;
            local_area_ll[1] = ckt->originY;
            local_area_ur[0] = ckt->originX + ckt->width;
            local_area_ur[1] = ckt->originY + ckt->height;
        }
        else
            local_search_area(m1, m2, iterCount/1, local_area_ll, local_area_ur);


        for(i=0; i < numbits; i++)
        {
            // check elapse time and runtime limit
            if(should_stop())
                return false;

            // always pin1 located leftside of pin2
            pin1 = &ckt->pins[sorted1[i]];
            pin2 = &ckt->pins[sorted2[i]];
            align1 = mp1->align;
            align2 = mp2->align;
            bitid = pin2bit[pin1->id];
            seq = sequence[sorted1[i]];//i; 
            vertical_arrange1 = (mp1->align == VERTICAL) ? true : false;
            vertical_arrange2 = (mp2->align == VERTICAL) ? true : false;

            into_array(multipin2llx[mp1->id], multipin2urx[mp1->id], multipin2lly[mp1->id], multipin2ury[mp1->id], mx1, my1);
            into_array(multipin2llx[mp2->id], multipin2urx[mp2->id], multipin2lly[mp2->id], multipin2ury[mp2->id], mx2, my2);

            // Get Pin Area valid
            box pinbox1, pinbox2;
            box ext1, ext2;
            box orig1, orig2;
            orig1 = box(pt(pin1->llx, pin1->lly), pt(pin1->urx, pin1->ury));
            orig2 = box(pt(pin2->llx, pin2->lly), pt(pin2->urx, pin2->ury));
            into_array(pin1->llx, pin1->urx, pin1->lly, pin1->ury, pin1x, pin1y);
            into_array(pin2->llx, pin2->urx, pin2->lly, pin2->ury, pin2x, pin2y);
            pin_area(pin1x, pin1y, align1, width[pin1->l], ext1);
            pin_area(pin2x, pin2y, align2, width[pin2->l], ext2);
#ifdef DEBUG_ROUTE_TWOPIN_NET
            printf("start %d seqence bit routing\n", seq);
            printf("p%d (%d %d) (%d %d) -> p%d (%d %d) (%d %d) seq %d bitid %d\n",
                    pin1->id,
                    pin1->llx, pin1->lly, pin1->urx, pin1->ury,
                    pin2->id,
                    pin2->llx, pin2->lly, pin2->urx, pin2->ury, seq, bitid);
#endif

            //
            BitRtree bit_rtree;
            construct_bit_rtree(bitid, bit_rtree);
            set<int> except1;
            set<int> except2;
            except1.insert(pin1->id);
            except1.insert(pin2->id);

            // variables
            seg elem1, elem2;
            minElem = INT_MAX;
            minCost = INT_MAX;
            minPanelty = INT_MAX;
            hasMinElem = false;

            int numelems = local_rtree_t.elemindex;
            vector<int> elemCost(numelems, INT_MAX);
            dense_hash_map<int,int> numSV;
            dense_hash_map<int,int> backtrace;
            dense_hash_map<int,int> depth;
            dense_hash_map<int,int> iterWL;
            dense_hash_map<int,int> estDist;
            dense_hash_map<int,int> accPanelty;
            dense_hash_map<int,int> iterPtx;
            dense_hash_map<int,int> iterPty;
            dense_hash_map<int,int> lastPtx;
            dense_hash_map<int,int> lastPty;
            dense_hash_map<int,seg> element;
            

            iterWL.set_empty_key(INT_MAX);
            estDist.set_empty_key(INT_MAX);
            accPanelty.set_empty_key(INT_MAX);
            numSV.set_empty_key(INT_MAX);
            backtrace.set_empty_key(INT_MAX);
            depth.set_empty_key(INT_MAX);
            iterPtx.set_empty_key(INT_MAX);
            iterPty.set_empty_key(INT_MAX);
            lastPtx.set_empty_key(INT_MAX);
            lastPty.set_empty_key(INT_MAX);
            element.set_empty_key(INT_MAX);
            vector<pair<seg, int>> queries;

            // Priority Queue
            auto cmp = [](const ituple &left, const ituple &right){
                return (get<1>(left) + get<2>(left) + get<3>(left) > get<1>(right) + get<2>(right) + get<3>(right));
            };
            priority_queue<ituple , vector<ituple>, decltype(cmp)> PQ(cmp);

            //
            queries.clear();
            local_rtree_t.query(QueryMode::Intersects, ext1, pin1->l-1, pin1->l+1, queries);
            
            // Initial candidate
            for(auto& it : queries)
            {
                elem1 = it.first;
                e1 = it.second;
                t1 = local_rtree_t.get_trackid(e1);
                l1 = local_rtree_t.get_layer(e1);
                vertical1 = local_rtree_t.is_vertical(e1); 
                maxWidth = local_rtree_t.get_width(e1);
                
                // width constraint
                if(maxWidth < width[l1])
                    continue;
                
                // condition
                if(vertical_arrange1 == vertical1)
                    continue;

                if(pin1->l != l1 && !bg::intersects(orig1, elem1))
                    continue;

                // visit
                backtrace[e1] = e1;         // e1 is root
                element[e1] = it.first;
                depth[e1] = 0;

                // get point
                lpt(elem1, sx1, sy1);
                upt(elem1, sx2, sy2);
                into_array(min(sx1, sx2), max(sx1, sx2),  min(sy1, sy2), max(sy1, sy2), wirex, wirey);
                intersection_pin(pin1x, pin1y, pin1->l, wirex, wirey, l1, -1, -1, x1, y1);
                iterPtx[e1] = x1;
                iterPty[e1] = y1;
                numSV[e1] = 0;
                // Find iterating point of e1
                c1 = VIA_COST * abs(pin1->l - l1);
                c2 = 0;
                c3 = 0; 
                
                isDestination = false; 
                num_spacing_vio = 0;
                if((abs(l1 - pin2->l) ==0 && bg::intersects(elem1, ext2)) || (abs(l1 - pin2->l) == 1 && bg::intersects(elem1, orig2)))
                {
                    isDestination = true;
                    intersection_pin(pin2x, pin2y, pin2->l, wirex, wirey, l1, x1, y1, x2, y2);
                    into_array(min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2), xs, ys);
                    num_spacing_vio = local_rtree_o.num_spacing_violations(bitid, xs, ys, l1, width[l1], spacing[l1], vertical1);
                    c1 += manhatan_distance(x1, y1, x2, y2);
                    c3 += num_spacing_vio * SPACING_VIOLATION;
                }
                
                c3 += num_spacing_vio * SPACING_VIOLATION;
                numSV[e1] = num_spacing_vio;
                elemCost[e1] = c1 + c2 + c3;
                iterWL[e1] = c1;
                estDist[e1] = c2;
                accPanelty[e1] = c3;
                PQ.push(make_tuple(e1,c1,c2,c3));
                
                if(isDestination && minPanelty > c3)
                {
                    lastPtx[e1] = x2;
                    lastPty[e1] = y2;
                    hasMinElem = true;
                    minCost = c1 + c2 + c3;
                    minPanelty = c3;
                    minElem = e1;
                    numDestSPV = num_spacing_vio;
                }
            }
            //


            while(PQ.size() > 0)
            {
                int cost1, cost2, cost3;
                ituple e = PQ.top();
                PQ.pop();

                e1 = get<0>(e);
                cost1 = get<1>(e);
                cost2 = get<2>(e);
                cost3 = get<3>(e);

                //
                visit_count++;
                //
                if(minElem == e1)
                    break;
                
                if(elemCost[e1] < cost1 + cost2 + cost3)
                    continue;

                if(minPanelty <= cost3)
                    continue;

                // routing condition
                if(maxDepth <= depth[e1])
                    continue;


                elem1 = element[e1];
                t1 = local_rtree_t.get_trackid(e1);
                l1 = local_rtree_t.get_layer(e1);
                x1 = iterPtx[e1];
                y1 = iterPty[e1];
                vertical1 = local_rtree_t.is_vertical(e1);
                dep1 = depth[e1];

                // query intersected tracks
                queries.clear();
                if(isRef)
                {
                    local_rtree_t.query(QueryMode::Intersects, elem1, l1-1, queries);
                    local_rtree_t.query(QueryMode::Intersects, elem1, l1+1, queries);
                }
                else
                    local_rtree_t.query(QueryMode::Intersects, elem1, tracelNum[dep1+1], queries);


                // Intersected available tracks
                for(auto& it : queries)
                {
                    e2 = it.second;
                    t2 = local_rtree_t.get_trackid(e2);
                    l2 = local_rtree_t.get_layer(e2);
                    maxWidth = local_rtree_t.get_width(e2);
                    elem2 = it.first;
                    dep2 = depth[e1] + 1;
                    vertical2 = local_rtree_t.is_vertical(e2);
                    //destination = false;                

                    if(e1 == e2)
                        continue;
                    
                    // intersection
                    if(!intersection(elem1, elem2, x2, y2))
                        continue;   
                    
                    // cost
                    c1 = cost1 + manhatan_distance(x1, y1, x2, y2) + abs(l1-l2) * VIA_COST + DEPTH_COST;
                    c2 = 0;
                    c3 = cost3;
                    
                    // local area
                    if(!is_inside(x2, y2, local_area_ll, local_area_ur))
                        continue;

                    // width constraint
                    if(maxWidth < width[l2])
                        continue;
                    
                    isDestination = false;
                    if((abs(pin2->l-l2) == 0 && bg::intersects(elem2, ext2)) || (abs(pin2->l-l2) == 1 && bg::intersects(elem2, orig2)))
                        isDestination = true;


                    if(!isRef)
                    {
                        // layer condition
                        if(tracelNum[dep2] != l2)
                            continue;

                        curDir = routing_direction(x1, y1, x2, y2, vertical1);
                        // if current routing direction is different,
                        // don't push into the priority queue
                        if(traceDir[dep1] != curDir)
                            continue; 
                        
                        //if(!(abs(l2 - pin2->l) ==0 && bg::intersects(elem2, ext2)) && !(abs(l2 - pin2->l) == 1 && bg::intersects(elem2, orig2)))
                        //{
                        if(!isDestination)
                        {
                            // wire ordering 
                            if(curDir != Direction::Point)
                            {
                                if(tracelx[dep2] != traceux[dep2])
                                {
                                    if(tracePtx[dep2] == tracelx[dep2])
                                    {
                                        if(tracePtx[dep2] <= x2)
                                            continue;
                                    }

                                    if(tracePtx[dep2] == traceux[dep2])
                                    {
                                        if(tracePtx[dep2] >= x2)
                                            continue;
                                    }
                                }

                                if(tracely[dep2] != traceuy[dep2])
                                {
                                    if(tracePty[dep2] == tracely[dep2])
                                    {
                                        if(tracePty[dep2] <= y2)
                                            continue;
                                    }

                                    if(tracePty[dep2] == traceuy[dep2])
                                    {
                                        if(tracePty[dep2] >= y2)
                                            continue;
                                    }
                                }
                            }
                        }

                        ////////////
                        c3 += manhatan_distance(tracePtx[dep2], tracePty[dep2], x2, y2);
                    }

                    // condition
                    if(dep1 == 0)
                    {
                        if(x1 == x2 && y1 == y2)
                            continue;

                        if(isRef)
                        {
                            // check compactness cost
                            curDir = routing_direction(x1, y1, x2, y2, vertical1);

                            if(!local_rtree_o.compactness((numbits-i), mx1, my1, x2, y2, l1, l2, align1, curDir, width[l2], spacing[l2]))
                                c3 += NOTCOMPACT;
                        }
                    }

                    // check spacing violation
                    into_array(min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2), xs, ys); 

                    num_spacing_vio = local_rtree_o.num_spacing_violations(bitid, xs, ys, l1, width[l1], spacing[l1], vertical1);
                    //c3 += num_spacing_vio * SPACING_VIOLATION;
                    
                    expand_width(xs, ys, width[l1], vertical1);
                    if(bit_rtree.short_violation(xs, ys, l1, except1, except2))
                        continue;
                    
                    //isDestination = false;
                    //if((abs(pin2->l-l2) == 0 && bg::intersects(elem2, ext2)) || (abs(pin2->l-l2) == 1 && bg::intersects(elem2, orig2)))
                    //{
                        //isDestination = true;
                    if(isDestination)
                    {
                        lpt(elem2, sx1, sy1);
                        upt(elem2, sx2, sy2);
                        into_array(sx1, sx2, sy1, sy2, wirex, wirey);
                        intersection_pin(pin2x, pin2y, pin2->l, wirex, wirey, l2, x2, y2, x3, y3); //lastPtx[e2], lastPty[e2]); 
                        //
                        if(vertical_arrange2 == vertical2)
                            continue;
                        // condition(routing direction)
                        if(!isRef)
                        {
                            curDir = routing_direction(x2, y2, x3, y3, vertical2);
                            // if current routing direction is different,
                            // don't push into the priority queue
                            if(traceDir[dep2] != curDir)
                                continue;
                            
                            if(maxDepth != dep2)
                                continue;
                        }

                        if(isRef && backtrace[e1] != e1)
                        {
                            int tmp = backtrace[e1];
                            prevDir = routing_direction(iterPtx[tmp], iterPty[tmp], x1, y1, local_rtree_t.is_vertical(tmp));
                            curDir = routing_direction(x2, y2, x3, y3, vertical2);
                            if(prevDir != curDir)
                                curDir = routing_direction(x3, y3, x2, y2, vertical2);

                            if(!local_rtree_o.compactness((numbits-i), mx2, my2, x2, y2, l1, l2, align2, curDir, width[l1], spacing[l1]))
                                c3 += NOTCOMPACT;
                        }

                        // check spacing violation
                        into_array(min(x2, x3), max(x2, x3), min(y2, y3), max(y2, y3), xs, ys);
                        num_spacing_vio += local_rtree_o.num_spacing_violations(bitid, xs, ys, l2, width[l2], spacing[l2], vertical2);
                        expand_width(xs, ys, width[l2], vertical2);
                        if(bit_rtree.short_violation(xs, ys, l2, except1, except2))
                            continue;
                        
                        c1 += manhatan_distance(x2, y2, x3, y3);
                    }
                    
                    c3 += num_spacing_vio*SPACING_VIOLATION;

                    if(elemCost[e2] <= c1 + c2 + c3)
                        continue;

                    element[e2] = elem2;
                    depth[e2] = dep2;
                    backtrace[e2] = e1;
                    iterPtx[e2] = x2;
                    iterPty[e2] = y2;
                    iterWL[e2] = c1;
                    estDist[e2] = c2;
                    accPanelty[e2] = c3;
                    elemCost[e2] = c1 + c2 + c3;
                    numSV[e2] = numSV[e1] + num_spacing_vio;
                    PQ.push(make_tuple(e2, c1, c2, c3));

                    if(isDestination && minPanelty > c3)
                    {
                        lastPtx[e2] = x3;
                        lastPty[e2] = y3;
                        hasMinElem = true;
                        minCost = c1 + c2 + c3;
                        minPanelty = c3;
                        minElem = e2;
                        numDestSPV = numSV[e2];
                    }

                }
                //
            }


            if(hasMinElem)
            {

#ifdef DEBUG_ROUTE_TWOPIN_NET
                //if(isRef)
                //{
                    printf("< backtrace >\n");
                    printf("minPanelty %d depth %d\n(%d %d) -> [%d] (%d %d)", 
                            minPanelty, depth[minElem], lastPtx[minElem], lastPty[minElem], minElem, iterPtx[minElem], iterPty[minElem]);

                    int tmp = minElem;
                    while(tmp != backtrace[tmp])
                    {
                        printf(" -> [%d] (%d %d)", backtrace[tmp], iterPtx[backtrace[tmp]], iterPty[backtrace[tmp]]);
                        tmp = backtrace[tmp];
                    }
                    printf("\n\n");
                //}
#endif

                // initial element
                e2 = minElem;
                l2 = local_rtree_t.get_layer(e2);

                // if reference routing, store
                if(isRef)
                {
                    totalSPV = 0;
                    maxDepth = depth[e2];
                    curDir = routing_direction(iterPtx[e2], iterPty[e2], lastPtx[e2], lastPty[e2], local_rtree_t.is_vertical(e2));
                    tracelNum.insert(tracelNum.begin(), l2);
                    traceDir.insert(traceDir.begin(), curDir);
                    tracelx = vector<int>((maxDepth+1), INT_MAX);
                    tracely = vector<int>((maxDepth+1), INT_MAX);
                    traceux = vector<int>((maxDepth+1), INT_MIN);
                    traceuy = vector<int>((maxDepth+1), INT_MIN);
                    tracePtx = vector<int>((maxDepth+1), INT_MIN);
                    tracePty = vector<int>((maxDepth+1), INT_MIN);

                    // Create default wires
                    created = vector<Wire>((maxDepth+1)*numbits);
                }


                // reverse count because of backtrace
                totalSPV += numDestSPV;
                count = maxDepth;
                w2 = (maxDepth+1)*seq + count--;
                trackid = local_rtree_t.get_trackid(e2);

                pin = true;
                // data
                x2 = iterPtx[e2];
                y2 = iterPty[e2];
                x3 = lastPtx[e2];
                y3 = lastPty[e2];

                into_array(min(x2,x3), max(x2,x3), min(y2,y3), max(y2,y3), xs, ys);
                wirex[0] = xs[0];
                wirex[1] = xs[1];
                wirey[0] = ys[0];
                wirey[1] = ys[1];
                expand_width(wirex, wirey, width[l2], local_rtree_t.is_vertical(e2));

                created[w2].get_info(bitid, trackid, xs, ys, l2, seq, pin);
                created[w2].add_intersection(PINTYPE, {lastPtx[e2], lastPty[e2]});
                // rtree update

                local_rtree_t.insert_element(trackid, xs, ys, l2, true);
                local_rtree_o.insert_obstacle(bitid, wirex, wirey, l2, false);

                while(backtrace[e2] != e2)
                {
                    // Iterating
                    e1 = backtrace[e2];
                    x1 = iterPtx[e1];
                    y1 = iterPty[e1];
                    x2 = iterPtx[e2];
                    y2 = iterPty[e2];
                    l1 = local_rtree_t.get_layer(e1);
                    vertical = local_rtree_t.is_vertical(e1);
                    tracelx[depth[e2]] = min(x2, tracelx[depth[e2]]);
                    tracely[depth[e2]] = min(y2, tracely[depth[e2]]);
                    traceux[depth[e2]] = max(x2, traceux[depth[e2]]);
                    traceuy[depth[e2]] = max(y2, traceuy[depth[e2]]);
                    tracePtx[depth[e2]] = x2;
                    tracePty[depth[e2]] = y2;

                    if(isRef)
                    {
                        tracelNum.insert(tracelNum.begin(), l1);
                        traceDir.insert(traceDir.begin(), routing_direction(x1, y1, x2, y2, vertical)); 
                    }

                    // data
                    w1 = (maxDepth+1)*seq + count--;
                    pin = (e1 == backtrace[e1]) ? true : false;
                    trackid = local_rtree_t.get_trackid(e1);
                    into_array(min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2), xs, ys);

                    wirex[0] = xs[0];
                    wirex[1] = xs[1];
                    wirey[0] = ys[0];
                    wirey[1] = ys[1];

                    // rtree update
                    expand_width(wirex, wirey, width[l1], local_rtree_t.is_vertical(e1));
                   
                    created[w1].get_info(bitid, trackid, xs, ys, l1, seq, pin);
                    local_rtree_t.insert_element(trackid, xs, ys, l1, true);
                    local_rtree_o.insert_obstacle(bitid, wirex, wirey, l1, false);


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
                cout << "[INFO] " << curbus->name << " routing failed at seq " << seq << endl << endl;
                solution = false;
                failed_tw++;
                failed_count++;
                break;
            }
            //
            isRef = false;
        }

        
        if(curbus->numPinShapes > 2)
            if(EPSILON < 2 * DELTA * totalSPV)
                solution = false;
        

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

                    Wire* curw = &created[index];
                    into_array(curw->x1, curw->x2, curw->y1, curw->y2, xs, ys);
                    l = curw->l;
                    seq = curw->seq;
                    pin = curw->pin;
                    trackid = curw->trackid;
                    bitid = curw->bitid;
                    //curw = CreateWire(bitid, trackid, xs, ys, l, seq, pin);
                    wireid = create_wire(bitid, trackid, xs, ys, l, seq, pin);
                    local2global[index] = wireid;

#ifdef DEBUG_ROUTE_TWOPIN_NET
                    if(count==0)
                    {
                        pin1 = &ckt->pins[mp1->pins[i]];
                        pin2 = &ckt->pins[mp2->pins[i]];
                        printf("p%d -> p%d seq %d bitid %d\n", mp1->pins[i], mp2->pins[i], seq, bitid);
                    }

                    printf("    %s (%d %d) (%d %d) M%d ",
                            ckt->bits[bitid].name.c_str(), xs[0], ys[0], xs[1], ys[1], l);

                    if(count == 0)
                        printf("p1 %d -> wire %d", pin1->id, wireid);
                    else if(count == maxDepth)
                        printf("p2 %d -> wire %d", pin2->id, wireid);

                    printf("\n");
#endif
                    if(count == 0)
                    {
                        pin2wire[mp1->pins[i]] = wireid;
                        wire2pin[wireid] = mp1->pins[i];
                        wires[wireid].add_intersection(PINTYPE, created[index].intersection[PINTYPE]);
                    }

                    if(count == maxDepth)
                    {
                        pin2wire[mp2->pins[i]] = wireid;
                        wire2pin[wireid] = mp2->pins[i];
                        wires[wireid].add_intersection(PINTYPE, created[index].intersection[PINTYPE]);
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
            {
                tp[count].id = count;
                tp[count].leaf = (count ==0 || count == maxDepth) ? true : false;

                if(count != 0)
                {
                    tp[count].neighbor.push_back(count-1);
                    tp[count-1].neighbor.push_back(count);
                }

                sort(tp[count].wires.begin(), tp[count].wires.end(), [&,this](int left, int right){
                        return this->wires[left].seq < this->wires[right].seq;
                        });
            }

            for(i=0; i < edges.size(); i++)
            {
                int w1 = local2global[edges[i].first];
                int w2 = local2global[edges[i].second];
                set_neighbor(w1, w2, pts[i].first, pts[i].second);
            }

            break;
        }else{
            failed_count++;
        }
    }

    cout << "< Route Twopin Net Report >" << endl;
    cout << "Bus        : " << curbus->name << endl;
    cout << "MultiPin1  : (" 
        << multipin2llx[m1] << " " << multipin2lly[m1] << ") (" 
        << multipin2urx[m1] << " " << multipin2ury[m1] << ") M" << mp1->l << endl;
    cout << "MultiPin2  : (" 
        << multipin2llx[m2] << " " << multipin2lly[m2] << ") (" 
        << multipin2urx[m2] << " " << multipin2ury[m2] << ") M" << mp2->l << endl;
    cout << "# visiting : " << visit_count << endl;
    cout << "# failed   : " << failed_count << endl << endl;
   
    cout << "[INFO] " << totalSPV << " * " <<  DELTA << " penalty occurs" << endl << endl;
    return solution; 

}

int pair_hash_key(int t1, int t2, int moduler)
{
    return t1 * moduler + t2;
}

size_t get_hash_key(int v1, int v2, int v3)
{
    size_t seed = 0;
    boost::hash_combine(seed, v1);
    boost::hash_combine(seed, v2);
    boost::hash_combine(seed, v3);
    cout << "hash combine (" << v1 << " " << v2 << " " << v3 << ") -> " << seed << endl;
    return seed;
}


bool OABusRouter::Router::t_junction_available(int busid, int x[], int y[], int l)
{
    int lower = max(0, l-1);
    int upper = min(l+1, (int)ckt->layers.size()-1);
    int required, numbits;
    int len_x, len_y;
    bool vertical1, vertical2;
    vertical1 = ckt->is_vertical(l);
    dense_hash_map<int,int> width = ckt->buses[busid].width;
    numbits = ckt->buses[busid].numBits;
    len_x = abs(x[0] - x[1]);
    len_y = abs(y[0] - y[1]);

    for(;lower <= upper;lower++)
    {
        vertical2 = ckt->is_vertical(lower);
        
        if(vertical1 != vertical2)
        {
            required = numbits*(spacing[lower] + width[lower]);
            if(vertical1)
            {
                if(len_y <= required)
                    return false;
            }
            else
            {
                if(len_x <= required)
                    return false;
            }
        }
    }
    return true;
}


void OABusRouter::Router::range_of_tj(Segment& target, int ll[], int ur[])
{
    bool is_vertical = target.vertical;
    ll[0] = (is_vertical) ? INT_MAX : INT_MIN;
    ll[1] = (is_vertical) ? INT_MIN : INT_MAX;
    ur[0] = (is_vertical) ? INT_MIN : INT_MAX;
    ur[1] = (is_vertical) ? INT_MAX : INT_MIN;
    
    for(int i=0; i < target.wires.size(); i++)
    {
        Wire* curw = &wires[target.wires[i]];
        ll[0] = is_vertical ? min(curw->x1, ll[0]) : max(curw->x1, ll[0]);
        ll[1] = is_vertical ? max(curw->y1, ll[1]) : min(curw->y1, ll[1]);
        ur[0] = is_vertical ? max(curw->x2, ur[0]) : min(curw->x2, ur[0]);
        ur[1] = is_vertical ? min(curw->y2, ur[1]) : max(curw->y2, ur[1]);
    }
}

bool OABusRouter::Router::route_multipin_to_tp(int busid, int m, vector<Segment> &tp)
{

    // check elapse time and runtime limit
    if(should_stop())
    {
        return false;
    }

    // Variables
    int i, j, wireid;
    int numwires, numpins;
    int numbits, cost, num_spacing_vio;
    int x1, y1, x2, y2, x, y;
    int ptx, pty, curShape;
    int llx, lly, urx, ury;
    int l1, l2, curDir, dist;
    int bitid, trackid, l, seq;
    int maxWidth, align;
    int ll[2], ur[2];
    int xs[2], ys[2];
    int wirex[2], wirey[2];
    int pinx[2], piny[2];
    int segx[2], segy[2];
    int mx[2], my[2];
    int maxDepth = INT_MAX;
    int count, index;
    int totalSPV;
    bool pin;
    bool vertical_align;
    bool vertical;
    bool solution = false;
    bool isRef;
    bool t_junction;
    bool ep_ll;
    bool ep_ur;

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
    Circuit* circuit;
    Segment target;
    
    
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
    into_array(llx, urx, lly, ury, mx, my);
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
    //
    int total_visit_count=0;


    // segment list
    for(auto seg : tp)
        PQ1.push(seg);
    
    int iterCount=0;
    int firstDir;
    bool retry = false;
    bool ref_fail = false;
    
    while(PQ1.size() > 0)
    {
        // check elapse time and runtime limit
        if(should_stop())
        {
            return false;
        }

        target  = PQ1.top();
        //PQ1.pop();
        
        if(!retry)
        {
            ep_ll = true;
            ep_ur = true;
            into_array(target.x1, target.x2, target.y1, target.y2, segx, segy);
            t_junction = t_junction_available(busid, segx, segy, target.l);
            if(t_junction)
                range_of_tj(target, ll, ur);
        }
           
        //ep_ll = true;
        //ep_ur = true;
        //into_array(target.x1, target.x2, target.y1, target.y2, segx, segy);
        //t_junction = t_junction_available(busid, segx, segy, target.l);
        //if(t_junction)
        //    range_of_tj(target, ll, ur);
        
        // copy local rtree
        TrackRtree local_rtree_t(rtree_t);
        ObstacleRtree local_rtree_o(rtree_o);
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
        vector<int> tracelx;
        vector<int> tracely;
        vector<int> traceux;
        vector<int> traceuy;
        vector<int> tracePtx;
        vector<int> tracePty;
        //sorted = curmp->pins;

        maxDepth = INT_MAX;
        solution = true;
        isRef = true;

        llx = target.x1;
        lly = target.y1;
        urx = target.x2;
        ury = target.y2;

        // Ordering
        Pin* firstPin = &ckt->pins[curmp->pins[0]];
        Pin* lastPin = &ckt->pins[curmp->pins[numpins-1]];
        Wire* firstWire = &wires[target.wires[0]];
        Wire* lastWire = &wires[target.wires[numpins-1]];
        box b1(pt(firstPin->llx, firstPin->lly), pt(firstPin->urx, firstPin->ury));
        box b2(pt(lastPin->llx, lastPin->lly), pt(lastPin->urx, lastPin->ury));
        seg s1(pt(firstWire->x1, firstWire->y1), pt(firstWire->x2, firstWire->y2));
        seg s2(pt(lastWire->x1, lastWire->y1), pt(lastWire->x2, lastWire->y2));
        float dist1 = bg::distance(b1, s1);
        float dist2 = bg::distance(b2, s2);
        if(dist1 < dist2)
            sorted.insert(sorted.end(), curmp->pins.begin(), curmp->pins.end());
        else
            sorted.insert(sorted.end(), curmp->pins.rbegin(), curmp->pins.rend()); 

        /*        
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
        */

        
        //
        for(i=0; i < numpins; i++)
        {

            // check elapse time and runtime limit
            if(should_stop())
            {
                return false;
            }


            seq = sequence[sorted[i]];
            curpin = &ckt->pins[sorted[i]];
            tarwire = &wires[target.wires[seq]];
            bitid = curbus->bits[seq];
            box ext;
            box orig;
            seg wireseg(pt(tarwire->x1, tarwire->y1), pt(tarwire->x2, tarwire->y2));
            // get 
            into_array(curpin->llx, curpin->urx, curpin->lly, curpin->ury, pinx, piny);
            orig = box(pt(pinx[0], piny[0]), pt(pinx[1], piny[1]));
            pin_area(pinx, piny, align, width[curpin->l], ext);
          
            ///////////////////////////////////////
            BitRtree bit_rtree;
            construct_bit_rtree(bitid, bit_rtree);
            set<int> except1;
            set<int> except2;
            except1.insert(curpin->id);
            except2.insert(tarwire->id);
            //////////////////////////////////////

            // variables
            seg elem;
            int dep;
            int w1, w2;
            int e1, e2, t1, t2;
            int c1, c2;
            int xDest, yDest;
            int minElem = INT_MAX;
            int minCost = INT_MAX;
            int numDestSV;
            int numelems;
            bool hasMinElem = false;
            bool destination;
            numelems = local_rtree_t.elemindex;
            dense_hash_map<int,int> numSV;
            dense_hash_map<int,int> backtrace;
            dense_hash_map<int,int> depth;
            dense_hash_map<int,int> iterPtx;
            dense_hash_map<int,int> iterPty;
            dense_hash_map<int,int> lastPtx;
            dense_hash_map<int,int> lastPty;
            dense_hash_map<int,seg> element;
            //
            dense_hash_map<int,int> wirelength;
            dense_hash_map<int,int> penalty;

            // wire length, penalty
            wirelength.set_empty_key(INT_MAX);
            penalty.set_empty_key(INT_MAX);
            numSV.set_empty_key(INT_MAX);
            backtrace.set_empty_key(INT_MAX);
            depth.set_empty_key(INT_MAX);
            iterPtx.set_empty_key(INT_MAX);
            iterPty.set_empty_key(INT_MAX);
            lastPtx.set_empty_key(INT_MAX);
            lastPty.set_empty_key(INT_MAX);
            element.set_empty_key(INT_MAX);
            vector<int> elemCost(numelems, INT_MAX);
            vector<pair<seg, int>> queries;

            auto cmp2 = [](const ituple &left, const ituple &right){
                return (get<1>(left) + get<2>(left) > get<1>(right) + get<2>(right));
            };
            priority_queue<ituple , vector<ituple>, decltype(cmp2)> PQ2(cmp2);



            for(int k=0; k < 2; k++)
            {
                minElem = INT_MAX;
                minCost = INT_MAX;
                hasMinElem = false;

                queries.clear();
                if(vertical_align == ckt->is_vertical(curpin->l))
                {
                    local_rtree_t.query(QueryMode::Intersects, orig, curpin->l-1, queries);
                    local_rtree_t.query(QueryMode::Intersects, orig, curpin->l+1, queries);
                }
                else
                {
                    if(k==0)
                        local_rtree_t.query(QueryMode::Intersects, orig, curpin->l, queries);
                    else
                        local_rtree_t.query(QueryMode::Intersects, ext, curpin->l, queries);
                }

                // Initial candidates
                for(auto& it : queries)
                {
                    e1 = it.second;
                    t1 = local_rtree_t.get_trackid(e1);
                    l1 = local_rtree_t.get_layer(e1);
                    maxWidth = local_rtree_t.get_width(e1);

                    // width constraint
                    if(maxWidth < width[l1])
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

                    c1 = VIA_COST * abs(curpin->l - l1);
                    c2 = 0;
                    iterPtx[e1] = x;
                    iterPty[e1] = y;
                    elemCost[e1] = c1 + c2;
                    wirelength[e1] = c1;
                    penalty[e1] = c2;
                    numSV[e1] = 0;

                    // initial candidates
                    PQ2.push(make_tuple(e1, c1, c2));

                    // if current segment is connected to target wire,
                    // check conditions and cost
                    if(abs(tarwire->l - l1) <= 1 && bg::intersects(element[e1],wireseg))
                    {
                        intersection(wireseg, element[e1], x, y);

                        into_array(min(iterPtx[e1], x), max(iterPtx[e1], x), min(iterPty[e1], y), max(iterPty[e1], y), xs, ys);
                        vertical = local_rtree_t.is_vertical(e1);

                        // Routing Shape
                        if(x == tarwire->x1 && y == tarwire->y1)
                            curShape = Direction::EndPointLL;
                        else if(x == tarwire->x2 && y == tarwire->y2)
                            curShape = Direction::EndPointUR;
                        else
                            curShape = Direction::T_Junction;


                        if(isRef)
                        {
                            if(curShape == Direction::T_Junction)
                                if(!t_junction)
                                    continue;
                                else
                                    if(!is_inside(x,y,ll,ur))
                                        continue;

                            if(curShape == Direction::EndPointLL && !ep_ll)
                                continue;

                            if(curShape == Direction::EndPointUR && !ep_ur)
                                continue;
                        }
                        else
                        {
                            curDir = routing_direction(iterPtx[e1], iterPty[e1], x, y, vertical);
                            if(traceDir[depth[e1]] != curDir)
                                continue;
                            // if current bit isn't reference bit,
                            // check layer sequence and direction, also shape
                            if(x == tarwire->x1 && y == tarwire->y1)
                                curShape = Direction::EndPointLL;
                            else if(x == tarwire->x2 && y == tarwire->y2)
                                curShape = Direction::EndPointUR;
                            else
                                curShape = Direction::T_Junction;

                            if(curShape != lastRoutingShape)
                                continue;

                            if(maxDepth != depth[e1])
                                continue;
                        }
                        
                        num_spacing_vio = local_rtree_o.num_spacing_violations(bitid, xs, ys, l1, width[l1], spacing[l1], vertical);
                        c1 += manhatan_distance(x, y, iterPtx[e1], iterPty[e1]);
                        c2 += num_spacing_vio * SPACING_VIOLATION;
                        //if(local_rtree_o.spacing_violations(bitid, xs, ys, l1, width[l1], spacing[l1], vertical))
                        //{
                        //    c2 += SPACING_VIOLATION;
                        //}
                        
                        ////////////////////////////////////////////////////////
                        expand_width(xs,ys,width[l1],vertical);
                        if(bit_rtree.short_violation(xs, ys, l1, except1, except2))
                            continue;
                        ///////////////////////////////////////////////////////

                        if(minCost > c1 + c2)
                        {
                            hasMinElem = true;
                            minCost = c1 + c2;
                            minElem = e1;
                            numDestSV = numSV[e1] + num_spacing_vio;
                            lastPtx[e1] = x;
                            lastPty[e1] = y;
                            PQ2.push(make_tuple(DESTINATION, c1, c2));
                        }
                    }

                }
                
                if(PQ2.size() > 0)
                    break;
            }
#ifdef DEBUG_ROUTE_MP_TO_TP
            printf("\ncurrent pin (%d %d) (%d %d) M%d -> target (%d %d) (%d %d) M%d\n",
                    pinx[0], piny[0], pinx[1], piny[1], curpin->l, 
                    tarwire->x1, tarwire->y1, tarwire->x2, tarwire->y2, tarwire->l);
            //printf("# first candidates %d\n", PQ2.size());
#endif

            //
            while(PQ2.size() > 0)
            {
                int cost1, cost2;
                ituple e = PQ2.top();
                PQ2.pop();

                e1 = get<0>(e);
                cost1 = get<1>(e);
                cost2 = get<2>(e);

                if(e1 == DESTINATION)
                    break;

                if(elemCost[e1] < cost1 + cost2)
                    continue;

                //
                total_visit_count++;
                // depth condition
                if(maxDepth <= depth[e1])
                    continue;
                // 
                t1 = local_rtree_t.get_trackid(e1);
                l1 = local_rtree_t.get_layer(e1);

               
                // query
                queries.clear();
                if(isRef)
                    local_rtree_t.query(QueryMode::Intersects, element[e1], l1-1, l1+1, queries);
                else
                    local_rtree_t.query(QueryMode::Intersects, element[e1], tracelNum[depth[e1]+1], tracelNum[depth[e1]+1], queries);

#ifdef DEBUG_ROUTE_MP_TO_TP
                //printf("e %d (%d %d) cost %d dep %d MinElem %d MinCost %d\n", e1, iterPtx[e1], iterPty[e1], cost1 + cost2, depth[e1], minElem, minCost); 
                //printf("# intersects segments : %d\n", queries.size());
#endif
                
                //
                for(auto& it : queries)
                {
                    e2 = it.second;
                    t2 = local_rtree_t.get_trackid(e2);
                    l2 = local_rtree_t.get_layer(e2);
                    maxWidth = local_rtree_t.get_width(e2);
                    elem = it.first;
                    dep = depth[e1] + 1;

                    
                    if(e1 == e2)
                        continue;
                   
                    if(abs(l1-l2) > 1)
                        continue;
        
                    // intersection
                    intersection(element[e1], elem, x, y);
                    // cost
                    c1 = cost1 + manhatan_distance(iterPtx[e1], iterPty[e1], x, y) + abs(l1-l2) * VIA_COST + DEPTH_COST;
                    c2 = cost2;
                    
                    // width constraint
                    if(maxWidth < width[l2])
                        continue;

                    // condition
                    if(depth[e1] == 0)
                    {
                        if((x == iterPtx[e1] && y == iterPty[e1]))
                            continue;

                        if(isRef)
                        {
                            curDir = routing_direction(iterPtx[e1], iterPty[e1], x, y, local_rtree_t.is_vertical(e1));
                            if(!local_rtree_o.compactness((numbits-i), mx, my, x, y, l1, l2, align, curDir, width[l2], spacing[l2]))
                                c2 += NOTCOMPACT;
                        }
                    }


                    if(!isRef)
                    {
                        // layer sequence
                        if(tracelNum[dep] != l2)
                            continue;
                        
                        //cout << "pass layer sequence" << endl;

                        //curDir = routing_direction(iterPtx[e1], iterPty[e1], x, y, localrtree.vertical(e1));
                        curDir = routing_direction(iterPtx[e1], iterPty[e1], x, y, local_rtree_t.is_vertical(e1));
                        if(traceDir[depth[e1]] != curDir)
                        {
                            continue;
                        }

                        if(curDir != Direction::Point)
                        {
                            if(tracelx[dep] != traceux[dep])
                            {
                                if(tracePtx[dep] == tracelx[dep])
                                {
                                    if(tracePtx[dep] <= x)
                                        continue;
                                }

                                if(tracePtx[dep] == traceux[dep])
                                {
                                    if(tracePtx[dep] >= x)
                                        continue;
                                }   
                            }

                            if(tracely[dep] != traceuy[dep])
                            {
                                if(tracePty[dep] == tracely[dep])
                                {
                                    if(tracePty[dep] <= y)
                                        continue;
                                }
                                else if(tracePty[dep] == traceuy[dep])
                                {
                                    if(tracePty[dep] >= y)
                                        continue;
                                }   
                            }
                        }

                        //cout << "pass wire ordering" << endl;
                        c2 += manhatan_distance(tracePtx[dep], tracePty[dep], x, y);
                    }
               

                    // check spacing violation
                    into_array(min(iterPtx[e1], x), max(iterPtx[e1], x), min(iterPty[e1], y), max(iterPty[e1], y), xs, ys);
                    vertical = local_rtree_t.is_vertical(e1);

                    num_spacing_vio = local_rtree_o.num_spacing_violations(bitid, xs, ys, l1, width[l1], spacing[l1], vertical);
                    c2 += num_spacing_vio * SPACING_VIOLATION;

                    //if(local_rtree_o.spacing_violations(bitid, xs, ys, l1, width[l1], spacing[l1], vertical))
                    //    c2 += SPACING_VIOLATION;

                    ////////////////////////////////////////////////////////
                    expand_width(xs, ys, width[l1], vertical);
                    if(bit_rtree.short_violation(xs, ys, l1, except1, except2))
                        continue;
                    ///////////////////////////////////////////////////////



                    if(elemCost[e2] <= c1 + c2)
                    {
                        continue;
                    }

                    element[e2] = elem;
                    depth[e2] = dep;
                    backtrace[e2] = e1;
                    iterPtx[e2] = x;
                    iterPty[e2] = y;
                    //
                    wirelength[e2] = c1;
                    penalty[e2] = c2;
                    numSV[e2] = numSV[e1] + num_spacing_vio;
                    //
                    elemCost[e2] = c1 + c2;
                    PQ2.push(make_tuple(e2, c1, c2));

                    // if destination
                    if(abs(tarwire->l - l2) <= 1 && bg::intersects(elem, wireseg))
                    {

                        intersection(elem, wireseg, ptx, pty); //
                        into_array(min(x, ptx), max(x, ptx), min(y, pty), max(y, pty), xs, ys);
                        vertical = local_rtree_t.is_vertical(e2);
                        // Routing Shape
                        if(ptx == tarwire->x1 && pty == tarwire->y1)
                            curShape = Direction::EndPointLL;
                        else if(ptx == tarwire->x2 && pty == tarwire->y2)
                            curShape = Direction::EndPointUR;
                        else
                            curShape = Direction::T_Junction;

                        if(isRef)
                        {
                            if(curShape == Direction::T_Junction)
                                if(!t_junction)
                                    continue;
                                else if(!is_inside(ptx,pty,ll,ur))
                                    continue;

                            if(curShape == Direction::EndPointLL && !ep_ll)
                                continue;

                            if(curShape == Direction::EndPointUR && !ep_ur)
                                continue;


                        }
                        else
                        {
                            if(curShape != lastRoutingShape)
                                continue;
                            
                            if(maxDepth != dep)
                                continue;
                            
                            curDir = routing_direction(x, y, ptx, pty, vertical);
                            if(traceDir[dep] != curDir)
                                continue;
                        }


                        num_spacing_vio = local_rtree_o.num_spacing_violations(bitid, xs, ys, l2, width[l2], spacing[l2], vertical);
                        c1 += manhatan_distance(x, y, ptx, pty);
                        c2 += num_spacing_vio * SPACING_VIOLATION;
                        // spacing violation check
                        //if(local_rtree_o.spacing_violations(bitid, xs, ys, l2, width[l2], spacing[l2], vertical))
                        //    c2 += SPACING_VIOLATION;

                        ////////////////////////////////////////////////////////
                        expand_width(xs,ys,width[l2],vertical);
                        if(bit_rtree.short_violation(xs, ys, l2, except1, except2))
                            continue;
                        ///////////////////////////////////////////////////////


                        
                        // if current minimum cost is bigger than e2
                        // assign minimum element to e2
                        if(minCost > c1 + c2)
                        {
 #ifdef DEBUG_ROUTE_MP_TO_TP
                            int tmp = e2;//minElem;
                            printf("\n\nDestination!\n\n");
                            printf("(%d %d) (%d %d) M%d\n", ptx, pty, iterPtx[e2], iterPty[e2], local_rtree_t.get_layer(e2));
                            while(tmp != backtrace[tmp])
                            {
                                int tmp2 = tmp;
                                tmp = backtrace[tmp];
                                printf("(%d %d) (%d %d) M%d\n", iterPtx[tmp], iterPty[tmp], iterPtx[tmp2], iterPty[tmp2], local_rtree_t.get_layer(tmp));
                            }
                            printf("<minCost %d curCost %d>\n",minCost, c1+c2);
                            printf("\n");
#endif
                            hasMinElem = true;
                            minCost = c1 + c2;
                            minElem = e2;
                            numDestSV = numSV[e2] + num_spacing_vio;
                            lastPtx[e2] = ptx;
                            lastPty[e2] = pty;
                            PQ2.push(make_tuple(DESTINATION, c1, c2));
                        }
                    }
                }
            }


            if(hasMinElem)
            {
                e2 = minElem;
                l2 = local_rtree_t.get_layer(e2);
                // if reference routing, store
                if(isRef)
                {
                    totalSPV = 0;
                    maxDepth = depth[e2];

                    if(lastPtx[e2] == tarwire->x1 && lastPty[e2] == tarwire->y1)
                        lastRoutingShape = Direction::EndPointLL;        
                    else if(lastPtx[e2] == tarwire->x2 && lastPty[e2] == tarwire->y2)
                        lastRoutingShape = Direction::EndPointUR;
                    else
                        lastRoutingShape = Direction::T_Junction;

                    curDir = routing_direction(iterPtx[e2], iterPty[e2], lastPtx[e2], lastPty[e2], local_rtree_t.is_vertical(e2));
                    tracelNum.insert(tracelNum.begin(), l2);
                    traceDir.insert(traceDir.begin(), curDir);
                    tracelx = vector<int>((maxDepth+1), INT_MAX);
                    tracely = vector<int>((maxDepth+1), INT_MAX);
                    traceux = vector<int>((maxDepth+1), INT_MIN);
                    traceuy = vector<int>((maxDepth+1), INT_MIN);
                    tracePtx = vector<int>((maxDepth+1), INT_MIN);
                    tracePty = vector<int>((maxDepth+1), INT_MIN);
                    created = vector<Wire>((maxDepth+1)*numbits);
                }

                count = maxDepth;
                totalSPV += numDestSV;

                w2 = (maxDepth+1)*seq + count--;
                //trackid = localrtree.trackid(e2);
                trackid = local_rtree_t.get_trackid(e2);
                pin = (backtrace[e2] == e2)? true : false;

                //
                into_array(min(lastPtx[e2], iterPtx[e2]), max(lastPtx[e2], iterPtx[e2]),
                           min(lastPty[e2], iterPty[e2]), max(lastPty[e2], iterPty[e2]), xs, ys);

                wirex[0] = xs[0];
                wirex[1] = xs[1];
                wirey[0] = ys[0];
                wirey[1] = ys[1];
                //expand_width(wirex, wirey, width[l2], localrtree.vertical(e2));
                expand_width(wirex, wirey, width[l2], local_rtree_t.is_vertical(e2));

                created[w2].get_info(bitid, trackid, xs, ys, l2, seq, pin);
                if(pin)
                    created[w2].add_intersection(PINTYPE, {iterPtx[e2], iterPty[e2]});

                //
                local_rtree_t.insert_element(trackid, xs, ys, l2, true);
                local_rtree_o.insert_obstacle(bitid, wirex, wirey, l2, false);
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
                    l1 = local_rtree_t.get_layer(e1);
                    vertical = local_rtree_t.is_vertical(e1);
                    
                    //
                    tracelx[depth[e2]] = min(x2, tracelx[depth[e2]]);
                    tracely[depth[e2]] = min(y2, tracely[depth[e2]]);
                    traceux[depth[e2]] = max(x2, traceux[depth[e2]]);
                    traceuy[depth[e2]] = max(y2, traceuy[depth[e2]]);
                    tracePtx[depth[e2]] = x2;
                    tracePty[depth[e2]] = y2;

                    if(isRef)
                    {
                        tracelNum.insert(tracelNum.begin(), l1);
                        traceDir.insert(traceDir.begin(), routing_direction(x1, y1, x2, y2, vertical));
                    }

                    // data
                    w1 = (maxDepth+1)*seq + count--;
                    pin = (e1 == backtrace[e1]) ? true : false;
                    //trackid = localrtree.trackid(e1);
                    trackid = local_rtree_t.get_trackid(e1);
                    into_array(min(x1,x2), max(x1,x2), min(y1,y2), max(y1,y2), xs, ys);

                    wirex[0] = xs[0];
                    wirex[1] = xs[1];
                    wirey[0] = ys[0];
                    wirey[1] = ys[1];
                    expand_width(wirex, wirey, width[l1], local_rtree_t.is_vertical(e1));

                    created[w1].get_info(bitid, trackid, xs, ys, l1, seq, pin);
                    local_rtree_t.insert_element(trackid, xs, ys, l1, true);
                    local_rtree_o.insert_obstacle(bitid, wirex, wirey, l1, false);

                    if(pin)
                        created[w1].add_intersection(PINTYPE, {x1, y1});

                    local_edges.push_back({w1,w2});
                    local_pts.push_back({x2,y2});
                    w2 = w1;
                    e2 = e1;
                }
                
                if(isRef)
                {
                    printf("< Reference Bit Routing Report >\n");
                    for(int tmp = minElem; ; tmp = backtrace[tmp])
                    {
                        if(tmp == minElem)
                        {
                            printf("bus name : %s\n", curbus->name.c_str());
                            printf("pin loc (%d %d) (%d %d) M%d\n",
                                    curpin->llx, curpin->lly, curpin->urx, curpin->ury, curpin->l);
                            printf("tar loc (%d %d) (%d %d) M%d\n", 
                                    tarwire->x1, tarwire->y1, tarwire->x2, tarwire->y2, tarwire->l);
                            printf("trace layer num = { ");
                            printf("M%d (%d %d) -> ", tracelNum[depth[tmp]], lastPtx[tmp], lastPty[tmp]);
                        }
                        
                        if(tmp == backtrace[tmp])
                        {
                            printf("M%d (%d %d) }\n", tracelNum[depth[tmp]], iterPtx[tmp], iterPty[tmp]);
                            break;
                        }
                        else
                            printf("M%d (%d %d) -> ", tracelNum[depth[tmp]], iterPtx[tmp], iterPty[tmp]);
                    }

                    for(int tmp = minElem; ; tmp = backtrace[tmp])
                    //for(int tmp = minElem; tmp != backtrace[tmp]; tmp = backtrace[tmp])
                    {
                        if(tmp == minElem)
                            printf("trace direction = { ");
                        if(traceDir[depth[tmp]] == Direction::Left)
                            printf("Left ");
                        if(traceDir[depth[tmp]] == Direction::Right)     
                            printf("Right ");
                        if(traceDir[depth[tmp]] == Direction::Up)     
                            printf("Up ");
                        if(traceDir[depth[tmp]] == Direction::Down)     
                            printf("Down ");
                        if(traceDir[depth[tmp]] == Direction::Point)     
                            printf("Point ");
                        if(tmp == backtrace[tmp])
                        {
                            printf("}\n");
                            printf("routing shape   = { ");
                            switch(lastRoutingShape)
                            {
                                case Direction::EndPointLL:
                                    printf("end point LL }\n");
                                    break;
                                case Direction::EndPointUR:
                                    printf("end point UR }\n");
                                    break;
                                case Direction::T_Junction:
                                    printf("T junction }\n ");
                                    break;
                                default:
                                    printf("?? }\n");
                                    break;
                            }
                            break;
                        }
                        else
                            printf("<- ");
                    }
                    printf("\n\n");
                }
#ifdef DEBUG_ROUTE_MP_TO_TP
#endif

            }
            else
            {
                solution = false;
                
                failed_tp++;
                printf("[Info] %s mp to tp routing failed (cur seq %d)\n", curbus->name.c_str(), seq);
#ifdef DEBUG_ROUTE_MP_TO_TP
                //printf("%s routing to tp failed\n", curbus->name.c_str());
                //printf("current sequence %d\n", seq);
#endif
                break;
            }
            //
            isRef = false;
        }
        //

        // if spacing violation penalty bigger enough,
        // retry
        
        if(EPSILON < 2 * DELTA * totalSPV)
           solution = false;

        if(solution)
        {
            //printf("get solution\n");
            dense_hash_map<int,int> local2global;
            local2global.set_empty_key(INT_MAX);
            topology = vector<Segment>(maxDepth+1);

            for(i=0; i < numbits; i++)
            {   
                for(count = 0; count < maxDepth+1 ; count++)
                {
                    index = i*(maxDepth+1) + count;
                    //wireid = wires.size();
                    Wire* curw = &created[index];
                    into_array(curw->x1, curw->x2, curw->y1, curw->y2, xs, ys);
                    l = curw->l;
                    seq = curw->seq;
                    pin = curw->pin;
                    trackid = curw->trackid;
                    bitid = curw->bitid;

                    //curw = CreateWire(bitid, trackid, xs, ys, l, seq, pin);
                    wireid = create_wire(bitid, trackid, xs, ys, l, seq, pin);
#ifdef DEBUG_ROUTE_MP_TO_TP
                    curpin = &ckt->pins[curmp->pins[i]];
                    if(count==0)
                    {
                        printf("\n\np %d (%d %d) (%d %d) M%d -> \n",
                                curpin->id, curpin->llx, curpin->lly, curpin->urx, curpin->ury, curpin->l);
                    }

                    printf("%s w%d (%d %d) (%d %d) M%d\n",
                            ckt->bits[bitid].name.c_str(), wireid, xs[0], ys[0], xs[1], ys[1], l);

                    if(count == maxDepth)
                    {
                        tarwire = &wires[target.wires[i]];
                        printf("%s w%d (%d %d) (%d %d) M%d\n",
                                ckt->bits[bitid].name.c_str(), tarwire->id, tarwire->x1, tarwire->y1, tarwire->x2, tarwire->y2, tarwire->l);
                    }
#endif
                    local2global[index] = wireid;
                    if(count == 0)
                    {
                        pin2wire[curmp->pins[i]] = wireid;
                        wire2pin[wireid] = curmp->pins[i];
                        wires[wireid].add_intersection(PINTYPE, created[index].intersection[PINTYPE]);
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
                set_neighbor(w1, w2, local_pts[i].first, local_pts[i].second);
            }

            for(i=0; i < global_edges.size(); i++)
            {
                int w1 = local2global[global_edges[i].first];
                int w2 = global_edges[i].second;
                set_neighbor(w1, w2, global_pts[i].first, global_pts[i].second);
            }

            // update topology
            for(i=0; i < topology.size(); i++)
            {
                Segment &seg = topology[i];
                seg.id = tp.size();
                tp.push_back(seg);
                tp[count].leaf = (i == 0) ? true : false;
                if(i != 0)
                {
                    tp[seg.id].neighbor.push_back(seg.id-1);
                    tp[seg.id-1].neighbor.push_back(seg.id);
                }

                if(i == topology.size() -1)
                {
                    tp[seg.id].neighbor.push_back(target.id);
                    tp[target.id].neighbor.push_back(seg.id);
                }
            }


            //tp.insert(tp.end(), topology.begin(), topology.end());
            break;
        }
        else
        {
            // re try
            ref_fail = isRef ? true : false;
            
            if(!retry && !ref_fail)
            {
                retry = true;
                
                if(lastRoutingShape == Direction::T_Junction)
                    t_junction = false;

                if(lastRoutingShape == Direction::EndPointLL)
                {
                    ep_ll = false;
                    ep_ur = true;
                }
                
                if(lastRoutingShape == Direction::EndPointUR)
                {
                    ep_ll = true;
                    ep_ur = false;
                }
            }
            else
            {
                PQ1.pop();
                ref_fail = false;
                ep_ur = true;
                ep_ll = true;
                retry = false;
            }
        
        }
        //
    }

    if(!solution)
    {
        cout << "[INFO] " << curbus->name << " mp to tp routing failed" << endl;
        failed++;
    }
    else
    {

        cout << "< Route Multipin to tp Report >" << endl;
        cout << "Bus        : " << curbus->name << endl;
        cout << "MultiPin   : (" 
            << mx[0] << " " << my[0] << ") (" << mx[1] << " " << my[1] << ") M" << l << endl;
        cout << "Segment    : ("
            << segx[0] << " " << segy[0] << ") (" << segx[1] << " " << segy[1] << ") M" << target.l << endl;
        cout << "# visiting : " << total_visit_count << endl;
        cout << "# failed   : " << failed_tp << endl << endl;
        
        cout << "[INFO] " << totalSPV << " * " << DELTA << " penalty occurs" << endl << endl;
    }
    return solution;
}

void OABusRouter::Router::remove_wire(int wireid)
{
    int trackid, l, width;
    int x[2], y[2];
    bool vertical;
    Wire* curwire;
    Bus* curbus;
    Bit* curbit;
    curwire = &wires[wireid];
    curbit = &ckt->bits[curwire->bitid];
    curbus = &ckt->buses[ckt->busHashMap[curbit->busName]];
    trackid = curwire->trackid;
    l = curwire->l;
    width = curbus->width[l];
    vertical = curwire->vertical;
    // remove wire
    into_array(curwire->x1, curwire->x2, curwire->y1, curwire->y2, x, y);
    rtree_t.insert_element(trackid, x, y, l, false);
    // remove obstacle
    expand_width(x, y, width, vertical);
    rtree_o.insert_obstacle(curbit->id, x, y, l, true);
}

void OABusRouter::Router::remove_all(int busid)
{
    int numbits, numwires, i, j;
    Bus* curbus = &ckt->buses[busid];
    Bit* curbit;
    numbits = curbus->numBits;
    // remove all wires
    for(i=0; i < numbits; i++)
    {
        curbit = &ckt->bits[curbus->bits[i]];
        numwires = curbit->wires.size();
        for(j=0; j < numwires; j++)
            remove_wire(curbit->wires[j]);
        curbit->wires.clear();
    }
}

void OABusRouter::Router::penalty_cost()
{
    int numBuses, numLayers, numObs, numBits, numWires, numPins;
    int delta, epsilon;
    int i, j, l;
    bool vertical;

    numLayers = ckt->layers.size();
    numBuses = ckt->buses.size();
    numObs = ckt->obstacles.size();
    numBits = ckt->bits.size();

    vector<int> Ps(numBuses, 0);
    vector<int> Pf(numBuses, 0);
    set<pair<int,int>> SV;

    int bitid;
    int elemid;
    int wireid;
    int obsid;
    int x[2], y[2];
    vector<BoxRtree> rtree(numLayers);
    vector<box> elems;
   
    dense_hash_map<int,int> elem2bus;
    dense_hash_map<int,int> elem2pin;
    dense_hash_map<int,int> elem2bit;
    dense_hash_map<int,int> elem2wire;
    dense_hash_map<int,int> elem2obs;
    dense_hash_map<int,int> elem2type;
    dense_hash_map<int,int> wire2elem;

    elem2wire.set_empty_key(INT_MAX);
    elem2obs.set_empty_key(INT_MAX);
    elem2type.set_empty_key(INT_MAX);
    elem2bus.set_empty_key(INT_MAX);
    elem2pin.set_empty_key(INT_MAX);
    elem2bit.set_empty_key(INT_MAX);
    wire2elem.set_empty_key(INT_MAX);


    delta = ckt->delta;
    epsilon = ckt->epsilon;

    for(i=0; i < numObs; i++)
    {
        Obstacle* obs = &ckt->obstacles[i];
        into_array(obs->llx, obs->urx, obs->lly, obs->ury, x, y);
        elemid = elems.size();
        l = obs->l;
        //
        box elem(pt(x[0], y[0]), pt(x[1], y[1]));
        rtree[l].insert({ elem, elemid });
        
        elems.push_back(elem);
        elem2type[elemid] = OBSTACLE;
        elem2obs[elemid] = obs->id;
    }


    for(i=0; i < numBits; i++)
    {
        Bit* curbit = &ckt->bits[i];
        Bus* curbus = &ckt->buses[ckt->busHashMap[curbit->busName]];
        numPins = curbit->pins.size();
        numWires = curbit->wires.size();
        bitid = curbit->id;
        for(j=0; j < numPins; j++)
        {
            Pin* curpin = &ckt->pins[curbit->pins[j]];
            into_array(curpin->llx, curpin->urx, curpin->lly, curpin->ury, x, y);
            elemid = elems.size();
            l = curpin->l;

            //
            box elem(pt(x[0], y[0]), pt(x[1], y[1]));
            rtree[l].insert({ elem, elemid });
            elems.push_back(elem);
            elem2type[elemid] = PINTYPE;
            elem2pin[elemid] = curpin->id;
            elem2bus[elemid] = curbus->id;
            elem2bit[elemid] = curbit->id;
        }
       

        for(j=0; j < numWires; j++)
        {
            Wire* curw = &wires[curbit->wires[j]];
            l = curw->l;
            vertical = curw->vertical;
            into_array(curw->x1, curw->x2, curw->y1, curw->y2, x, y);
            expand_width(x, y, curbus->width[l], vertical);
            elemid = elems.size();

            //
            box elem(pt(x[0], y[0]), pt(x[1], y[1]));
            rtree[l].insert({ elem, elemid });
            elems.push_back(elem);
            
            elem2type[elemid] = WIRETYPE;
            elem2wire[elemid] = curw->id;
            elem2bus[elemid] = curbus->id;
            elem2bit[elemid] = curbit->id;
            wire2elem[curw->id] = elemid;
        }
    }
   
    for(i=0; i < numBits; i++)
    {
        Bit* curbit = &ckt->bits[i];
        Bus* curbus = &ckt->buses[ckt->busHashMap[curbit->busName]];
        numWires = curbit->wires.size();

        for(j=0; j < numWires; j++)
        {
            int e1, e2;
            vector<pair<box,int>> queries;
            Wire* curw = &wires[curbit->wires[j]];
            
            //if(curw->via)
            //    continue;

            l = curw->l;
            vertical = curw->vertical;
            into_array(curw->x1, curw->x2, curw->y1, curw->y2, x, y);
            design_ruled_area(x, y, curbus->width[l], spacing[l], vertical);

            box area(pt(x[0], y[0]), pt(x[1], y[1]));
            e1 = wire2elem[curw->id];

            rtree[l].query(bgi::intersects(area), back_inserter(queries));
            for(auto& it : queries)
            {
                e2 = it.second;
                
                if(e1==e2)
                    continue;
                
                //if(!bg::overlaps(area, it.first))
                //    continue;

                if(bg::touches(area, it.first))
                    continue;


                if(SV.find({e1,e2}) == SV.end() && SV.find({e2,e1}) == SV.end())
                {
                    int type = elem2type[e2];
                    if(elem2type[e2] == PINTYPE)
                    {
                        if(elem2bit[e1] != elem2bit[e2])
                        {
                            SV.insert({e1,e2});
                            if(elem2bus[e1] == elem2bus[e2])
                            {
                                Ps[elem2bus[e1]] += delta;
                            }
                            else
                            {
                                Ps[elem2bus[e1]] += delta;
                                Ps[elem2bus[e2]] += delta;
                            }
                        }
                    }
                    else
                    {
                        if(elem2type[e2] == WIRETYPE)
                        {
                            //if(wires[elem2wire[e2]].via)
                            //    continue;

                            if(elem2bit[e1] == elem2bit[e2])
                                continue;
                            
                            if(elem2bus[e1] == elem2bus[e2])
                            {
                                Ps[elem2bus[e1]] += delta;
                            }
                            else
                            {
                                Ps[elem2bus[e1]] += delta;
                                Ps[elem2bus[e2]] += delta;
                            }
                        }
                        else
                        {
                            Ps[elem2bus[e1]] += delta;
                        }
                        SV.insert({e1,e2});
                    }
                    //
                }
                //
            }
            //
        }
        // end wires
    }
    // end bits
  
    printf("\n\n------------------------------------\n\n");
    printf("Ps : #Spacing violations * %d\nPf : #routing failed * %d\n\n", delta, epsilon);
    for(i=0; i < numBuses; i++)
    {
        printf("%s  -> Ps %d\n", ckt->buses[i].name.c_str(), Ps[i]);
    }
    printf("total SV %d\n", SV.size());
    printf("\n\n------------------------------------\n\n");
}

/////////////////
// BACKUP CODE //
/////////////////

/*
   if(tracelx[dep] != traceux[dep] && tracely[dep] != traceuy[dep])
   {
   if(tracePtx[dep] == tracelx[dep])
   {
   if(tracePtx[dep] < x)
   continue;
   }
   else if(tracePtx[dep] == traceux[dep])
   {
   if(tracePtx[dep] > x)
   continue;
   }

   if(tracePty[dep] == tracely[dep])
   {
   if(tracePty[dep] < y)
   continue;
   }
   else if(tracePty[dep] == traceuy[dep])
   {
   if(tracePty[dep] > y)
   continue;
   }
   }
   */
/*
   bool wire_order_failed = false;
   if(tracePtx[dep] == tracelx[dep])
   {
   if(tracePtx[dep] < x)
   wire_order_failed = true;

   }
   else if(tracePtx[dep] == traceux[dep])
   {
   if(tracePtx[dep] > x)
   wire_order_failed = true;
   }

   if(tracePty[dep] == tracely[dep])
   {
   if(tracePty[dep] < y)
   wire_order_failed = true;
   }
   else if(tracePty[dep] == traceuy[dep])
   {
   if(tracePty[dep] > y)
   wire_order_failed = true;
   }
   if(wire_order_failed)
   {
//printf("bound (%d %d) (%d %d) previous (%d %d) current (%d %d)\n",
//        tracelx[dep], tracely[dep], traceux[dep], traceuy[dep], tracePtx[dep], tracePty[dep], x, y);

continue;
}
*/
//}
//}

/*
bool OABusRouter::Router::route_multipin_to_tp(int busid, int m, vector<Segment> &tp)
{

    // Variables
    int i, j, wireid;
    int numwires, numpins;
    int numbits, cost;
    int x1, y1, x2, y2, x, y;
    int ptx, pty; 
    int llx, lly, urx, ury;
    int l1, l2, curDir, dist;
    int bitid, trackid, l, seq;
    int maxWidth, align;
    int xs[2], ys[2];
    int wirex[2], wirey[2];
    int pinx[2], piny[2];
    int mx[2], my[2];
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
    //SegRtree* trackrtree;
    Circuit* circuit;

    //Rtree localrtree(rtree);
    //trackrtree = &localrtree.track;
    
    
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
    into_array(llx, lly, urx, ury, mx, my);
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
    //
    int total_visit_count=0;


    // segment list
    for(auto seg : tp)
        PQ1.push(seg);
    

    while(PQ1.size() > 0)
    {
        Segment target = PQ1.top();
        PQ1.pop();
       
        
        // copy local rtree
        TrackRtree local_rtree_t(rtree_t);
        ObstacleRtree local_rtree_o(rtree_o);
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
        vector<int> tracelx;
        vector<int> tracely;
        vector<int> traceux;
        vector<int> traceuy;
        vector<int> tracePtx;
        vector<int> tracePty;
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
            box ext;
            box orig;
            seg wireseg(pt(tarwire->x1, tarwire->y1), pt(tarwire->x2, tarwire->y2));
            // get 
            into_array(curpin->llx, curpin->urx, curpin->lly, curpin->ury, pinx, piny);
            orig = box(pt(pinx[0], piny[0]), pt(pinx[1], piny[1]));
            pin_area(pinx, piny, align, width[curpin->l], ext);
           

            // variables
            seg elem;
            int dep;
            int w1, w2;
            int e1, e2, t1, t2;
            int c1, c2;
            int xDest, yDest;
            int minElem = INT_MAX;
            int minCost = INT_MAX;
            int numelems;
            bool hasMinElem = false;
            bool destination;
            numelems = local_rtree_t.elemindex;
            dense_hash_map<int,int> backtrace;
            dense_hash_map<int,int> depth;
            dense_hash_map<int,int> iterPtx;
            dense_hash_map<int,int> iterPty;
            dense_hash_map<int,int> lastPtx;
            dense_hash_map<int,int> lastPty;
            dense_hash_map<int,seg> element;
            backtrace.set_empty_key(INT_MAX);
            depth.set_empty_key(INT_MAX);
            iterPtx.set_empty_key(INT_MAX);
            iterPty.set_empty_key(INT_MAX);
            lastPtx.set_empty_key(INT_MAX);
            lastPty.set_empty_key(INT_MAX);
            element.set_empty_key(INT_MAX);
            vector<int> elemCost(numelems, INT_MAX);
            vector<pair<seg, int>> queries;

            auto cmp2 = [](const ituple &left, const ituple &right){
                return (get<1>(left) + get<2>(left) > get<1>(right) + get<2>(right));
            };
            priority_queue<ituple , vector<ituple>, decltype(cmp2)> PQ2(cmp2);

            queries.clear();
            local_rtree_t.query(QueryMode::Intersects, ext, curpin->l-1, curpin->l+1, queries);
            // Initial candidates
            for(auto& it : queries)
            {
                e1 = it.second;
                t1 = local_rtree_t.get_trackid(e1);
                l1 = local_rtree_t.get_layer(e1);
                maxWidth = local_rtree_t.get_width(e1);

                if(abs(l1 - curpin->l) == 1 && !bg::intersects(orig, it.first))
                    continue;

                // width constraint
                if(maxWidth < width[l1])
                    continue;

                //
                //if(vertical_align == localrtree.vertical(e1))
                if(vertical_align == local_rtree_t.is_vertical(e1))
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

                c1 = VIA_COST * abs(curpin->l - l1);
                c2 = 0;
                iterPtx[e1] = x;
                iterPty[e1] = y;
                elemCost[e1] = c1 + c2;

                if(abs(tarwire->l - l1) <= 1 && bg::intersects(element[e1],wireseg))
                {
                    intersection(wireseg, element[e1], x, y);
                    lastPtx[e1] = x;
                    lastPty[e1] = y;

                    into_array(min(iterPtx[e1], lastPtx[e1]), max(iterPtx[e1], lastPtx[e1]),
                            min(iterPty[e1], lastPty[e1]), max(iterPty[e1], lastPty[e1]), xs, ys);
                    vertical = local_rtree_t.is_vertical(e1);
                    
                    if(!local_rtree_o.spacing_violations(bitid, xs, ys, l1, width[l1], spacing[l1], vertical))
                    {
                        if(!isRef)
                        {
                            int curShape;
                            if(lastPtx[e2] == tarwire->x1 && lastPty[e2] == tarwire->y1)
                                curShape = Direction::EndPointLL;
                            else if(lastPtx[e2] == tarwire->x2 && lastPty[e2] == tarwire->y2)
                                curShape = Direction::EndPointUR;
                            else
                                curShape = Direction::T_Junction;

                            if(curShape != lastRoutingShape)
                                continue;
                        }

                        if(minCost > c1 + c2)
                        {
                            hasMinElem = true;
                            minCost = c1 + c2;
                            minElem = e1;
                        }
                    }
                }

                //
                PQ2.push(make_tuple(e1, c1, c2));
            }
#ifdef DEBUG_ROUTE_MP_TO_TP
            printf("current pin (%d %d) (%d %d) M%d -> target (%d %d) (%d %d) M%d\n",
                    pinx[0], piny[0], pinx[1], piny[1], curpin->l, 
                    tarwire->x1, tarwire->y1, tarwire->x2, tarwire->y2, tarwire->l);
            printf("# first candidates %d\n", PQ2.size());
#endif

            //
            while(PQ2.size() > 0)
            {
                int cost1, cost2;
                ituple e = PQ2.top();
                PQ2.pop();

                e1 = get<0>(e);
                cost1 = get<1>(e);
                cost2 = get<2>(e);

                //
                total_visit_count++;
                // depth condition
                if(maxDepth <= depth[e1])
                    continue;

                if(isRef)
                {
                    //printf("e %d (%d %d) cost %d dep %d\n", e1, iterPtx[e1], iterPty[e1], cost1 + cost2, depth[e1]); 
                    //printf("MinElem %d MinCost %d CurCost %d\n", minElem, minCost, cost1 + cost2);
                }
                // 
                t1 = local_rtree_t.get_trackid(e1);
                l1 = local_rtree_t.get_layer(e1);

               
                // query
                queries.clear();
                local_rtree_t.query(QueryMode::Intersects, element[e1], l1-1, l1+1, queries);

                //
                for(auto& it : queries)
                {
                    e2 = it.second;
                    t2 = local_rtree_t.get_trackid(e2);
                    l2 = local_rtree_t.get_layer(e2);
                    maxWidth = local_rtree_t.get_width(e2);
                    elem = it.first;
                    dep = depth[e1] + 1;
                    destination = false;

                    // cost
                    c1 = cost1 + manhatan_distance(iterPtx[e1], iterPty[e1], x, y) + abs(l1-l2) * VIA_COST + DEPTH_COST;
                    c2 = cost2;
                    
                    if(e1 == e2)
                        continue;
                    
                    if(abs(l1-l2) > 1)
                        continue;
         
                    if(maxWidth < width[l2])
                        continue;

                    // intersection
                    intersection(element[e1], elem, x, y);

                    // condition
                    if(depth[e1] == 0)
                    {

                        if((x == iterPtx[e1] && y == iterPty[e1]))
                            continue;

                        if(isRef)
                        {
                            //curDir = routing_direction(iterPtx[e1], iterPty[e1], x, y, localrtree.vertical(e1));
                            curDir = routing_direction(iterPtx[e1], iterPty[e1], x, y, local_rtree_t.is_vertical(e1));

                            if(!local_rtree_o.compactness((numbits-i), mx, my, x, y, l1, l2, align, curDir, width[l2], spacing[l2]))
                                c2 += NOTCOMPACT;
                        }
                    }


                    if(!isRef)
                    {
                        // layer sequence
                        if(tracelNum[dep] != l2)
                            continue;

                        //curDir = routing_direction(iterPtx[e1], iterPty[e1], x, y, localrtree.vertical(e1));
                        curDir = routing_direction(iterPtx[e1], iterPty[e1], x, y, local_rtree_t.is_vertical(e1));
                        if(traceDir[depth[e1]] != curDir)
                            continue;
                    
                        // wire ordering 
                        if(tracelx[dep] <= x && x <= traceux[dep])
                            continue;

                        if(tracely[dep] <= y && y <= traceuy[dep])
                            continue;
                        
                        if(tracelx[dep] != traceux[dep] && tracely[dep] != traceuy[dep])
                        {

                            if(tracePtx[dep] == tracelx[dep])
                            {
                                if(tracePtx[dep] < x)
                                    continue;
                            }
                            else if(tracePtx[dep] == traceux[dep])
                            {
                                if(tracePtx[dep] > x)
                                    continue;
                            }

                            if(tracePty[dep] == tracely[dep])
                            {
                                if(tracePty[dep] < y)
                                    continue;
                            }
                            else if(tracePty[dep] == traceuy[dep])
                            {
                                if(tracePty[dep] > y)
                                    continue;
                            }
                            c2 += manhatan_distance(tracePtx[dep], tracePty[dep], x, y);
                        }
                    }
               

                    if((depth[e1] == 0) && (x == iterPtx[e1] && y == iterPty[e1]))
                        continue;


                    // check spacing violation
                    into_array(min(iterPtx[e1], x), max(iterPtx[e1], x), min(iterPty[e1], y), max(iterPty[e1], y), xs, ys);
                    vertical = local_rtree_t.is_vertical(e1);

                    if(local_rtree_o.spacing_violations(bitid, xs, ys, l1, width[l1], spacing[l1], vertical))
                        c2 += SPACING_VIOLATION;

                    // if destination
                    if(abs(tarwire->l - l2) <= 1 && bg::intersects(elem, wireseg))
                    {
                        destination = true;

                        intersection(elem, wireseg, ptx, pty); //
                        lastPtx[e2] = ptx;
                        lastPty[e2] = pty;
                        into_array(min(x, lastPtx[e2]), max(x, lastPtx[e2]), min(y, lastPty[e2]), max(y, lastPty[e2]), xs, ys);
                        vertical = local_rtree_t.is_vertical(e2);
                        if(!isRef)
                        {
                            curDir = routing_direction(x, y, lastPtx[e2], lastPty[e2], vertical);

                            if(traceDir[dep] != curDir)
                                continue;

                            int curShape;
                            if(lastPtx[e2] == tarwire->x1 && lastPty[e2] == tarwire->y1)
                                curShape = Direction::EndPointLL;
                            else if(lastPtx[e2] == tarwire->x2 && lastPty[e2] == tarwire->y2)
                                curShape = Direction::EndPointUR;
                            else
                                curShape = Direction::T_Junction;

                            if(curShape != lastRoutingShape)
                                continue;
           
                            if(maxDepth != dep)
                                continue;
                        
                        }
    
                        //if(localrtree.spacing_violations(bitid, xs, ys, l2, width[l2], spacing[l2], vertical))
                        if(local_rtree_o.spacing_violations(bitid, xs, ys, l2, width[l2], spacing[l2], vertical))
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
                if(hasMinElem && (e1 == minElem))
                    break;
            }


            if(hasMinElem)
            {
                e2 = minElem;
                l2 = local_rtree_t.get_layer(e2);

                // if reference routing, store
                if(isRef)
                {
                    cout << "==========" << endl;
                    cout << "Backtrace" << endl;
                    printf("MinElem %d MinCost %d CurCost %d\n", minElem, minCost, elemCost[e2]);
                    maxDepth = depth[e2];

                    if(lastPtx[e2] == tarwire->x1 && lastPty[e2] == tarwire->y1)
                        lastRoutingShape = Direction::EndPointLL;        
                    else if(lastPtx[e2] == tarwire->x2 && lastPty[e2] == tarwire->y2)
                        lastRoutingShape = Direction::EndPointUR;
                    else
                        lastRoutingShape = Direction::T_Junction;

                    curDir = routing_direction(iterPtx[e2], iterPty[e2], lastPtx[e2], lastPty[e2], local_rtree_t.is_vertical(e2));
                    tracelNum.insert(tracelNum.begin(), l2);
                    traceDir.insert(traceDir.begin(), curDir);
                    tracelx = vector<int>((maxDepth+1), INT_MAX);
                    tracely = vector<int>((maxDepth+1), INT_MAX);
                    traceux = vector<int>((maxDepth+1), INT_MIN);
                    traceuy = vector<int>((maxDepth+1), INT_MIN);
                    tracePtx = vector<int>((maxDepth+1), INT_MIN);
                    tracePty = vector<int>((maxDepth+1), INT_MIN);
                    created = vector<Wire>((maxDepth+1)*numbits);
                }

                count = maxDepth;

                w2 = (maxDepth+1)*seq + count--;
                //trackid = localrtree.trackid(e2);
                trackid = local_rtree_t.get_trackid(e2);
                pin = (backtrace[e2] == e2)? true : false;

                //
                into_array(min(lastPtx[e2], iterPtx[e2]), max(lastPtx[e2], iterPtx[e2]),
                           min(lastPty[e2], iterPty[e2]), max(lastPty[e2], iterPty[e2]), xs, ys);

                wirex[0] = xs[0];
                wirex[1] = xs[1];
                wirey[0] = ys[0];
                wirey[1] = ys[1];
                //expand_width(wirex, wirey, width[l2], localrtree.vertical(e2));
                expand_width(wirex, wirey, width[l2], local_rtree_t.is_vertical(e2));

                created[w2].get_info(bitid, trackid, xs, ys, l2, seq, pin);
                if(pin)
                    created[w2].add_intersection(PINTYPE, {iterPtx[e2], iterPty[e2]});

                //
                local_rtree_t.insert_element(trackid, xs, ys, l2, true);
                local_rtree_o.insert_obstacle(bitid, wirex, wirey, l2, false);
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
                    l1 = local_rtree_t.get_layer(e1);
                    vertical = local_rtree_t.is_vertical(e1);
                    
                    
                    //
                    tracelx[depth[e2]] = min(x2, tracelx[depth[e2]]);
                    tracely[depth[e2]] = min(y2, tracely[depth[e2]]);
                    traceux[depth[e2]] = max(x2, traceux[depth[e2]]);
                    traceuy[depth[e2]] = max(y2, traceuy[depth[e2]]);
                    tracePtx[depth[e2]] = x2;
                    tracePty[depth[e2]] = y2;

                    if(isRef)
                    {
                        tracelNum.insert(tracelNum.begin(), l1);
                        traceDir.insert(traceDir.begin(), routing_direction(x1, y1, x2, y2, vertical));
                    }

                    // data
                    w1 = (maxDepth+1)*seq + count--;
                    pin = (e1 == backtrace[e1]) ? true : false;
                    //trackid = localrtree.trackid(e1);
                    trackid = local_rtree_t.get_trackid(e1);
                    into_array(min(x1,x2), max(x1,x2), min(y1,y2), max(y1,y2), xs, ys);

                    wirex[0] = xs[0];
                    wirex[1] = xs[1];
                    wirey[0] = ys[0];
                    wirey[1] = ys[1];
                    expand_width(wirex, wirey, width[l1], local_rtree_t.is_vertical(e1));

                    created[w1].get_info(bitid, trackid, xs, ys, l1, seq, pin);
                    local_rtree_t.insert_element(trackid, xs, ys, l1, true);
                    local_rtree_o.insert_obstacle(bitid, wirex, wirey, l1, false);

                    if(pin)
                        created[w1].add_intersection(PINTYPE, {x1, y1});

                    local_edges.push_back({w1,w2});
                    local_pts.push_back({x2,y2});
                    w2 = w1;
                    e2 = e1;
                }

                if(isRef)
                {
                    int tmp = minElem;

                    printf("trace layer = { ");
                    while(tmp != backtrace[tmp])
                    {
                        printf("M%d -> ", tracelNum[depth[tmp]]);
                        tmp = backtrace[tmp];

                    }
                    printf("M%d }\n", tracelNum[depth[tmp]]);

                    printf("trace dir   = { ");
                    tmp = minElem;
                    while(true)
                    {
                        switch(traceDir[depth[tmp]])
                        {
                            case Direction::Left:
                                printf("Left ");
                                break;
                            case Direction::Right:
                                printf("Right ");
                                break;
                            case Direction::Up:
                                printf("Up ");
                                break;
                            case Direction::Down:
                                printf("Down ");
                                break;
                            case Direction::Point:
                                printf("Point ");
                                break;
                            default:
                                printf("?? ");
                                break;
                        }
                        if(tmp == backtrace[tmp])
                        {
                            printf("}\n");
                            break;
                        }else{
                            printf("-> ");
                        }
                        tmp = backtrace[tmp];
                    }

                    printf("last    = { ");
                    switch(lastRoutingShape)
                    {
                        case Direction::EndPointLL:
                            printf("end point LL }\n");
                            break;
                        case Direction::EndPointUR:
                            printf("end point UR }\n");
                            break;
                        case Direction::T_Junction:
                            printf("T junction }\n ");
                            break;
                        default:
                            printf("?? }\n");
                            break;
                    }
                    printf("\n\n\n");
                }
#ifdef DEBUG_ROUTE_MP_TO_TP
#endif


            }
            else
            {
                solution = false;
                failed_tp++;
                break;
            }
            //
            isRef = false;
            
            //delete[] elemCost;
        }
        //
        
        if(solution)
        {
            //printf("get solution\n");
            dense_hash_map<int,int> local2global;
            local2global.set_empty_key(INT_MAX);

            topology = vector<Segment>(maxDepth+1);

            for(i=0; i < numbits; i++)
            {   
                for(count = 0; count < maxDepth+1 ; count++)
                {
                    index = i*(maxDepth+1) + count;
                    //wireid = wires.size();
                    Wire* curw = &created[index];
                    into_array(curw->x1, curw->x2, curw->y1, curw->y2, xs, ys);
                    l = curw->l;
                    seq = curw->seq;
                    pin = curw->pin;
                    trackid = curw->trackid;
                    bitid = curw->bitid;
#ifdef DEBUG_ROUTE_MP_TO_TP
                    curpin = &ckt->pins[curmp->pins[i]];
                    if(count==0)
                    {
                        printf("\n\np %d (%d %d) (%d %d) M%d -> \n",
                                curpin->id, curpin->llx, curpin->lly, curpin->urx, curpin->ury, curpin->l);
                    }

                    printf("%s (%d %d) (%d %d) M%d\n",
                            ckt->bits[bitid].name.c_str(), xs[0], ys[0], xs[1], ys[1], l);

                    if(count == maxDepth)
                    {
                        tarwire = &wires[target.wires[i]];
                        printf("%s (%d %d) (%d %d) M%d\n",
                                ckt->bits[bitid].name.c_str(), tarwire->x1, tarwire->y1, tarwire->x2, tarwire->y2, tarwire->l);
                    }
#endif
                    //curw = CreateWire(bitid, trackid, xs, ys, l, seq, pin);
                    wireid = create_wire(bitid, trackid, xs, ys, l, seq, pin);
                    local2global[index] = wireid;
                    if(count == 0)
                    {
                        pin2wire[curmp->pins[i]] = wireid;
                        wire2pin[wireid] = curmp->pins[i];
                        wires[wireid].add_intersection(PINTYPE, created[index].intersection[PINTYPE]);
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
                //SetNeighbor(&wires[w1], &wires[w2], local_pts[i].first, local_pts[i].second);
                set_neighbor(w1, w2, local_pts[i].first, local_pts[i].second);
            }

            for(i=0; i < global_edges.size(); i++)
            {
                int w1 = local2global[global_edges[i].first];
                int w2 = global_edges[i].second;
                //SetNeighbor(&wires[w1], &wires[w2], global_pts[i].first, global_pts[i].second);
                set_neighbor(w1, w2, global_pts[i].first, global_pts[i].second);
            }

            tp.insert(tp.end(), topology.begin(), topology.end());
            break;

            //return true;
        }
        //
    }

    if(!solution)
    {
        printf("No solution...\n");
        failed++;
    }
    printf("total visit count : %d\n", total_visit_count);
    return solution;
}
*/


