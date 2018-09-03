
#include "circuit.h"
#include "route.h"
#include "func.h"
#include "rtree.h"

#include <stdlib.h>
#include <time.h>
#include <unordered_map>
#include <tuple>

#define MAX_ITERATION_COUNT 10
#define VIA_COST 1000 
#define DEPTH_COST 1000
#define SPACING_VIOLATION max(ckt->width + ckt->height,100000)
#define NOTCOMPACT 10000
#define DESTINATION -12311

#define DEBUG_ROUTE_MP_TO_TP
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
            curbus->assign = true;
            printf("%s -> routing success\n", curbus->name.c_str());   
        }
        else
        {
            curbus->assign = false;
            printf("%s -> routing failed\n", curbus->name.c_str());
        }
        
    }

    printf("\n======================\n");
    printf("    # failed tw : %d\n", failed_tw);
    printf("    # failed tp : %d\n", failed_tp);
    printf("======================\n");
    
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

    //////
    sort(comb.begin(), comb.end(), [&,this](pair<int,int> left, pair<int,int> right){
            box b1, b2;
            float dist1, dist2;
            b1 = box(pt(this->multipin2llx[left.first], this->multipin2lly[left.first])
                        ,pt(this->multipin2urx[left.first], this->multipin2ury[left.first]));
            b2 = box(pt(this->multipin2llx[left.second], this->multipin2lly[left.second])
                        ,pt(this->multipin2urx[left.second], this->multipin2ury[left.second]));
            dist1 = bg::distance(b1,b2);

            b1 = box(pt(this->multipin2llx[right.first], this->multipin2lly[right.first])
                        ,pt(this->multipin2urx[right.first], this->multipin2ury[right.first]));
            b2 = box(pt(this->multipin2llx[right.second], this->multipin2lly[right.second])
                        ,pt(this->multipin2urx[right.second], this->multipin2ury[right.second]));
            dist2 = bg::distance(b1,b2);
            return dist1 > dist2;
            });

    // loop for route_two_pin_net
    while(comb.size() > 0)
    {
        candi = *comb.begin();
        comb.erase(comb.begin());
        //candi = pop_random(comb);
        mp1 = candi.first;
        mp2 = candi.second;
        // pick two multipins for routing
        // routing topologies
        vector<Segment> tp;

        if(route_twopin_net(busid, mp1, mp2, tp))
        {
            mps.erase(find(mps.begin(), mps.end(), mp1));
            mps.erase(find(mps.begin(), mps.end(), mp2));

            // route until no remained multipins
            while(mps.size() > 0)
            {
                mp1 = pop_random(mps);
                // route target multipin to net topologies (tp)
                
                
                if(!route_multipin_to_tp(busid, mp1, tp))
                {
                    // rip-up
                    congested = get_congested_bus(busid);
                    rip_up(congested);
                    reroutes.push_back(congested);
                    // re-try
                    mps.push_back(mp1);
                    break;
                
                }
                
                 
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
        expand = (maxWidth + maxSpacing)*numbits*count*10;
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

bool OABusRouter::Router::route_twopin_net(int busid, int m1, int m2, vector<Segment> &tp)
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
    int maxWidth, align1, align2;
    int local_area_ll[2], local_area_ur[2];
    int xs[2], ys[2];
    int wirex[2], wirey[2];
    int pin1x[2], pin1y[2];
    int pin2x[2], pin2y[2];
    int mx1[2], my1[2], mx2[2], my2[2];
    int count, index;
    int prevDir;
    bool pin;
    bool vertical_arrange1;
    bool vertical_arrange2;
    bool vertical;
    bool solution = true;
    
    
    typedef PointBG pt;
    typedef SegmentBG seg;
    typedef BoxBG box;
    typedef tuple<int,int,int> ituple;

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
    /*
    sort_pins_routing_sequence(m1, m2, sorted1, sorted2);
    */
    curbus = &ckt->buses[busid];
    numbits = curbus->numBits;
    width = curbus->width;
    int visit_count=0;
    int failed_count=0;
    cout << "====================" << endl;
    printf("start routing %s\n", curbus->name.c_str());
    printf("m1 (%d %d) (%d %d) : ", multipin2llx[m1], multipin2lly[m1], multipin2urx[m1], multipin2ury[m1]);
    if(align1 == VERTICAL)
        printf("VERTICAL");
    else
        printf("HORIZONTAL");
    printf(" -> ");
    printf("m2 (%d %d) (%d %d) : ", multipin2llx[m2], multipin2lly[m2], multipin2urx[m2], multipin2ury[m2]);
    if(align2 == VERTICAL)
        printf("VERTICAL");
    else
        printf("HORIZONTAL");
    printf("\n");

    int iterCount=0;
    bool maximum_search_area = false;
    bool swapped = false;

    while(iterCount++ < 4)
    {
        if(iterCount % 2 == 0)
        {
            swap(mp1, mp2);
            swap(m1,m2);
        }
        sort_pins_routing_sequence(m1, m2, sorted1, sorted2);
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
        int maxDepth = INT_MAX;
        bool isRef = true;
        vector<int> tracelNum;  // trace layer number
        vector<int> traceDir;   // trace direction
        vector<int> tracelx, tracely, traceux, traceuy;
        vector<int> tracePtx;
        vector<int> tracePty;
        vector<Wire> created;
        vector<pair<int, int>> edges;
        vector<pair<int, int>> pts;
        TrackRtree local_rtree_t(rtree_t);
        ObstacleRtree local_rtree_o(rtree_o);
        
        // local search area
        if((int)(1.0 * iterCount / 2) > 1)
        {
            local_area_ll[0] = ckt->originX;
            local_area_ll[1] = ckt->originY;
            local_area_ur[0] = ckt->originX + ckt->width;
            local_area_ur[1] = ckt->originY + ckt->height;
        }
        else
        {
            local_search_area(m1, m2, iterCount, local_area_ll, local_area_ur);
        }
        /*
        if(local_area_ll[0] == ckt->originX && local_area_ur[0] == ckt->originX + ckt->width
                && local_area_ll[1] == ckt->originY && local_area_ur[1] == ckt->originY + ckt->height)
        {
            maximum_search_area = true;

            //iterCount = MAX_ITERATION_COUNT-1;
        }
        */
        //printf("local search area (%d %d) (%d %d)\n", local_area_ll[0], local_area_ll[1],
        //        local_area_ur[0], local_area_ur[1]);
        solution = true;

        for(i=0; i < numbits; i++)
        {
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
            printf("\n\n\n");
            printf("=======================================\n");
            printf("local search area (%d %d) (%d %d)\n", local_area_ll[0], local_area_ll[1],
                    local_area_ur[0], local_area_ur[1]);
            printf("start %d seqence bit routing\n", seq);
            printf("p%d (%d %d) (%d %d) -> p%d (%d %d) (%d %d) seq %d bitid %d\n",
                    pin1->id,
                    pin1->llx, pin1->lly, pin1->urx, pin1->ury,
                    pin2->id,
                    pin2->llx, pin2->lly, pin2->urx, pin2->ury, seq, bitid);


            printf("m1 (%d %d) (%d %d) : ", multipin2llx[m1], multipin2lly[m1], multipin2urx[m1], multipin2ury[m1]);
            if(align1 == VERTICAL)
                printf("VERTICAL");
            else
                printf("HORIZONTAL");
            printf(" -> ");
            printf("m2 (%d %d) (%d %d) : ", multipin2llx[m2], multipin2lly[m2], multipin2urx[m2], multipin2ury[m2]);
            if(align2 == VERTICAL)
                printf("VERTICAL");
            else
                printf("HORIZONTAL");
            printf("\n");
#endif

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
            vector<int> elemCost(numelems, INT_MAX);
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
            vector<pair<seg, int>> queries;

            // Priority Queue
            auto cmp = [](const ituple &left, const ituple &right){
                return (get<1>(left) + get<2>(left) > get<1>(right) + get<2>(right));
            };
            priority_queue<ituple , vector<ituple>, decltype(cmp)> PQ(cmp);


            //
            queries.clear();
            local_rtree_t.query(QueryMode::Intersects, ext1, pin1->l-1, pin1->l+1, queries);
            // Initial candidate
            for(auto& it : queries)
            {
                e1 = it.second;
                t1 = local_rtree_t.get_trackid(e1);
                l1 = local_rtree_t.get_layer(e1);
                maxWidth = local_rtree_t.get_width(e1);
                
                // width constraint
                if(maxWidth < width[l1])
                {
                    continue;
                }
                // condition
                if(vertical_arrange1 == ckt->is_vertical(l1))
                {
                    continue;
                }

                if(abs(pin1->l - l1) > 1) 
                {
                    continue;
                }

                if(abs(pin1->l != l1) && !bg::intersects(orig1, it.first))
                {
                    continue;
                }

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
                if((abs(pin2->l-l1) == 0 && bg::intersects(element[e1], ext2)) || (abs(pin2->l-l1) == 1 && bg::intersects(element[e1], orig2)))
                {
                    intersection_pin(pin2x, pin2y, pin2->l, wirex, wirey, l1, iterPtx[e1], iterPty[e1], x, y);
                    lastPtx[e1] = x;
                    lastPty[e1] = y;


                    into_array(min(iterPtx[e1], lastPtx[e1]), max(iterPtx[e1], lastPtx[e1]), 
                            min(iterPty[e1], lastPty[e1]), max(iterPty[e1], lastPty[e1]), xs, ys);
                    vertical = local_rtree_t.is_vertical(e1);

                    if(local_rtree_o.spacing_violations(bitid, xs, ys, l1, width[l1], spacing[l1], vertical))
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
#ifdef DEBUG_ROUTE_TWOPIN_NET
                //printf("\n\ncurrent Queue size %d\n", PQ.size());
#endif
                int cost1, cost2;
                ituple e = PQ.top();
                PQ.pop();

                e1 = get<0>(e);
                cost1 = get<1>(e);
                cost2 = get<2>(e);

                //
                visit_count++;
                //

                if(elemCost[e1] < cost1 + cost2)
                    continue;


                if(isRef)
                {
                    //printf("e %d (%d %d) cost %d dep %d\n", e1, iterPtx[e1], iterPty[e1], cost1 + cost2, depth[e1]); //iterPtx[e2], iterPty[e2], elemCost[e2]);
                }
                //printf("MinCost %d CurCost %d\n", minCost, cost1 + cost2);
                // 

                if(hasMinElem)
                {
                    if(e1 == minElem)
                        break;

                    if(cost1 + cost2 >= minCost)
                        continue;
                }

                // routing condition
                if(maxDepth <= depth[e1])
                    continue;


                t1 = local_rtree_t.get_trackid(e1);
                l1 = local_rtree_t.get_layer(e1);

                // query intersected tracks
                queries.clear();
                local_rtree_t.query(QueryMode::Intersects, element[e1], l1-1, l1+1, queries);

                // Intersected available tracks
                for(auto& it : queries)
                {
                    e2 = it.second;
                    t2 = local_rtree_t.get_trackid(e2);
                    l2 = local_rtree_t.get_layer(e2);
                    maxWidth = local_rtree_t.get_width(e2);
                    elem = it.first;
                    dep = depth[e1] + 1;
                    destination = false;                

                    if(e1 == e2)
                        continue;
                    
                    // intersection
                    intersection(element[e1], elem, x, y);
                    // cost
                    c1 = cost1 + manhatan_distance(iterPtx[e1], iterPty[e1], x, y) + abs(l1-l2) * VIA_COST + DEPTH_COST;
                    c2 = cost2; 
                    
                    


                    // local area
                    if(!is_inside(x, y, local_area_ll, local_area_ur))
                    {
                        //printf("point (%d %d) area (%d %d) (%d %d)\n", x, y, local_area_ll[0], local_area_ll[1], local_area_ur[0], local_area_ur[1]);
                        continue;
                    }

                    // condition
                    if(abs(l1 - l2) > 1)
                        continue;




                    // width constraint
                    if(maxWidth < width[l2])
                        continue;

                    if(!isRef)
                    {
                        // layer condition
                        if(tracelNum[dep] != l2)
                            continue;

                        curDir = routing_direction(iterPtx[e1], iterPty[e1], x, y, local_rtree_t.is_vertical(e1));

                        // if current routing direction is different,
                        // don't push into the priority queue
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
                        }
                        ////////////
                        //이거때문에 example_1_3, alpha_1_2결과가 달라짐
                        c2 += manhatan_distance(tracePtx[dep], tracePty[dep], x, y);
                    }

                    // condition
                    if(depth[e1] == 0)
                    {

                        if((x == iterPtx[e1] && y == iterPty[e1]))
                            continue;

                        if(isRef)
                        {
                            //curDir = routing_direction(iterPtx[e1], iterPty[e1], x, y, localrtree.vertical(e1));
                            curDir = routing_direction(iterPtx[e1], iterPty[e1], x, y, local_rtree_t.is_vertical(e1));
                            
                            /////////////////////////////////
                            //if(!routability_check(m1, t2, curDir))
                            //    continue;

                            if(!local_rtree_o.compactness((numbits-i), mx1, my1, x, y, l1, l2, align1, curDir, width[l2], spacing[l2]))
                                c2 += NOTCOMPACT;
                        
                        }
                    }
                    // check spacing violation
                    into_array(min(iterPtx[e1], x), max(iterPtx[e1], x), min(iterPty[e1], y), max(iterPty[e1], y), xs, ys); 
                    vertical = local_rtree_t.is_vertical(e1);

                    if(local_rtree_o.spacing_violations(bitid, xs, ys, l1, width[l1], spacing[l1], vertical))
                        c2 += SPACING_VIOLATION;

                    // if destination
                    if((abs(pin2->l-l2) == 0 && bg::intersects(elem, ext2)) || (abs(pin2->l-l2) == 1 && bg::intersects(elem, orig2)))
                    {
                        destination = true;
                        lpt(elem, x1, y1);
                        upt(elem, x2, y2);
                        into_array(x1, x2, y1, y2, wirex, wirey);
                        intersection_pin(pin2x, pin2y, pin2->l, wirex, wirey, l2, x, y, ptx, pty); //lastPtx[e2], lastPty[e2]); 
                        
                        //
                        if(vertical_arrange2 == local_rtree_t.is_vertical(e2))
                            continue;

                        if(isRef && backtrace[e1] != e1)
                        {
                            int tmp = backtrace[e1];
                            prevDir = routing_direction(iterPtx[tmp], iterPty[tmp], iterPtx[e1], iterPty[e1], local_rtree_t.is_vertical(tmp));
                            curDir = routing_direction(x, y, ptx, pty, local_rtree_t.is_vertical(e2));
                            if(prevDir != curDir)
                                curDir = routing_direction(ptx, pty, x, y, local_rtree_t.is_vertical(e2));

                            if(!local_rtree_o.compactness((numbits-i), mx2, my2, x, y, l1, l2, align2, curDir, width[l1], spacing[l1]))
                                c2 += NOTCOMPACT;
                        }


                        // condition(routing direction)
                        if(!isRef)
                        {
                            curDir = routing_direction(x, y, ptx, pty, local_rtree_t.is_vertical(e2));
                            // if current routing direction is different,
                            // don't push into the priority queue
                            if(traceDir[dep] != curDir)
                                continue;
                            
                            if(maxDepth != dep)
                                continue;
                        }

                        // check spacing violation
                        into_array(min(x, ptx), max(x, ptx), min(y, pty), max(y, pty), xs, ys);
                        vertical = local_rtree_t.is_vertical(e2);

                        if(local_rtree_o.spacing_violations(bitid, xs, ys, l2, width[l2], spacing[l2], vertical))
                            c2 += SPACING_VIOLATION;
                        
                        lastPtx[e2] = ptx;
                        lastPty[e2] = pty;
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
#ifdef DEBUG_ROUTE_TWOPIN_NET
                    //printf("PUSH e %d (%d %d) cost %d dep %d\n", e2, x, y, c1+c2, dep);
#endif
                    
                    // if current element arrivals destination
                    // compare minimum cost
                    if(destination)
                    {
                        if(minCost > c1 + c2)
                        {
                            hasMinElem = true;
                            minCost = c1 + c2;
                            minElem = e2;
#ifdef DEBUG_ROUTE_TWOPIN_NET
                            printf("\n\nFind destination\nmin cost %d current depth %d\n(%d %d) -> [%d] (%d %d)", 
                                    minCost,depth[e2], lastPtx[e2], lastPty[e2], e2, iterPtx[e2], iterPty[e2]);

                            int tmp = e2;
                            while(tmp != backtrace[tmp])
                            {
                                printf(" -> [%d] (%d %d)", backtrace[tmp], iterPtx[backtrace[tmp]], iterPty[backtrace[tmp]]);
                                tmp = backtrace[tmp];
                            }
                            printf("\n\n");
#endif
                        }
                    }
                    PQ.push(make_tuple(e2, c1, c2));
                }
                //
            }


            if(hasMinElem)
            {

#ifdef DEBUG_ROUTE_TWOPIN_NET
                printf("backtrace\n");
                printf("min cost %d depth %d\n(%d %d) -> [%d] (%d %d)", minCost, depth[minElem], lastPtx[minElem], lastPty[minElem], minElem, iterPtx[minElem], iterPty[minElem]);

                int tmp = minElem;
                while(tmp != backtrace[tmp])
                {
                    printf(" -> [%d] (%d %d)", backtrace[tmp], iterPtx[backtrace[tmp]], iterPty[backtrace[tmp]]);
                    tmp = backtrace[tmp];
                }
                printf("\n\n");
#endif

                // initial element
                e2 = minElem;
                //l2 = localrtree.layer(e2);
                l2 = local_rtree_t.get_layer(e2);

                // if reference routing, store
                if(isRef)
                {
                    maxDepth = depth[e2];
                    //curDir = routing_direction(iterPtx[e2], iterPty[e2], lastPtx[e2], lastPty[e2], localrtree.vertical(e2));
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
                count = maxDepth;
                w2 = (maxDepth+1)*seq + count--;
                trackid = local_rtree_t.get_trackid(e2);

                pin = true;
                // data
                into_array(min(lastPtx[e2], iterPtx[e2]), max(lastPtx[e2], iterPtx[e2]), 
                        min(lastPty[e2], iterPty[e2]), max(lastPty[e2], iterPty[e2]), xs, ys);

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
                    //
                    //printf("index %d -> bitid %d\n", w1, bitid);
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
                solution = false;
                failed_tw++;
                failed_count++;
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
                    //wireid = wires.size();
                    //local2global[index] = wireid;

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
                    if(bitid != curbus->bits[i])
                    {
                        cout << "illegal bitid ..." << endl;
                        cout << "b1 : " << bitid << endl;
                        cout << "b2 : " << curbus->bits[i] << endl;
                        cout << "seq : " << seq << endl;
                        cout << "index : " << index << endl;
                        exit(0);
                    }

                    if(i != seq)
                    {
                        cout << "illegal seq ..." << endl;
                        exit(0);
                    }

                    if(count==0)
                    {
                        pin1 = &ckt->pins[mp1->pins[i]];
                        pin2 = &ckt->pins[mp2->pins[i]];
                        printf("p%d -> p%d seq %d bitid %d\n", mp1->pins[i], mp2->pins[i], seq, bitid);
                        /*
                           printf("%s p1 (%d %d) (%d %d) -> p2 (%d %d) (%d %d)\n", 
                           ckt->bits[curbus->bits[bitid]].name.c_str(),
                           pin1->llx, pin1->lly, pin1->urx, pin1->ury,
                           pin2->llx, pin2->lly, pin2->urx, pin2->ury);
                           */
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
                sort(tp[count].wires.begin(), tp[count].wires.end(), [&,this](int left, int right){
                        return this->wires[left].seq < this->wires[right].seq;
                        });

            for(i=0; i < edges.size(); i++)
            {
                int w1 = local2global[edges[i].first];
                int w2 = local2global[edges[i].second];
                set_neighbor(w1, w2, pts[i].first, pts[i].second);
                //SetNeighbor(&wires[w1], &wires[w2], pts[i].first, pts[i].second);
            }
            break;
        }
    }

    cout << "# visiting : " << visit_count << endl;
    cout << "# failed   : " << failed_count << endl;
    cout << "====================" << endl << endl;
    return solution; 
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

bool OABusRouter::Router::route_multipin_to_tp(int busid, int m, vector<Segment> &tp)
{

    // Variables
    int i, j, wireid;
    int numwires, numpins;
    int numbits, cost;
    int x1, y1, x2, y2, x, y;
    int ptx, pty, curShape;
    int llx, lly, urx, ury;
    int l1, l2, curDir, dist;
    int bitid, trackid, l, seq;
    int maxWidth, align;
    int xs[2], ys[2];
    int wirex[2], wirey[2];
    int pinx[2], piny[2];
    int segx[2], segy[2];
    int mx[2], my[2];
    int maxDepth = INT_MAX;
    int count, index;

    bool pin;
    bool vertical_align;
    bool vertical;
    bool solution = false;
    bool isRef;
    bool t_junction;
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
    

    while(PQ1.size() > 0)
    {
        Segment target = PQ1.top();
        PQ1.pop();
       
        printf("\n=========================================\n");
        printf("%s routing to tp start\n", curbus->name.c_str());
        printf("m (%d %d) (%d %d) m%d -> s (%d %d) (%d %d) m%d\n",
                mx[0], my[0], mx[1], my[1], l, target.x1, target.y1, target.x2, target.y2, target.l);

        into_array(target.x1, target.x2, target.y1, target.y2, segx, segy);
        t_junction = t_junction_available(busid, segx, segy, target.l);

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
            //
            dense_hash_map<int,int> wirelength;
            dense_hash_map<int,int> penalty;

            // wire length, penalty
            wirelength.set_empty_key(INT_MAX);
            penalty.set_empty_key(INT_MAX);
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

                    //if(abs(l1 - curpin->l) == 1 && !bg::intersects(orig, it.first))
                    //    continue;

                    // width constraint
                    if(maxWidth < width[l1])
                        continue;

                    //
                    //if(vertical_align == local_rtree_t.is_vertical(e1))
                    //    continue;

                    //
                    //if(abs(curpin->l - l1) > 1)
                    //    continue;


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
                            if(vertical != target.vertical)
                                if(!t_junction && curShape == Direction::T_Junction)
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

                        c1 += manhatan_distance(x, y, iterPtx[e1], iterPty[e1]);
                        if(!local_rtree_o.spacing_violations(bitid, xs, ys, l1, width[l1], spacing[l1], vertical))
                        {
                            c2 += SPACING_VIOLATION;
                        }


                        if(minCost > c1 + c2)
                        {
                            hasMinElem = true;
                            minCost = c1 + c2;
                            minElem = e1;
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
                {
                    continue;
                }
                /*
                if(cost1 > wirelength[e1] || cost2 > penalty[e1])
                {
                    //cout << "different cost!!" << endl;
                    //cout << "cost1 : " << cost1 << endl;
                    //cout << "orig1 : " << wirelength[e1] << endl;
                    //cout << "cost2 : " << cost2 << endl;
                    //cout << "orig2 : " << penalty[e1] << endl;
                    //exit(0);
                    continue;
                }
                */
                //
                total_visit_count++;
                // depth condition
                if(maxDepth <= depth[e1])
                    continue;
#ifdef DEBUG_ROUTE_MP_TO_TP
                printf("e %d (%d %d) cost %d dep %d MinElem %d MinCost %d\n", e1, iterPtx[e1], iterPty[e1], cost1 + cost2, depth[e1], minElem, minCost); 
#endif
                // 
                t1 = local_rtree_t.get_trackid(e1);
                l1 = local_rtree_t.get_layer(e1);

               
                // query
                ////
                queries.clear();
                if(isRef)
                    local_rtree_t.query(QueryMode::Intersects, element[e1], l1-1, l1+1, queries);
                else
                    local_rtree_t.query(QueryMode::Intersects, element[e1], tracelNum[depth[e1]+1], tracelNum[depth[e1]+1], queries);

                
                printf("# intersects segments : %d\n", queries.size());
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
                    
                    ///////////////////////////////////
                    //if(isRef)
                    //{
                    //    printf("candidate (%d %d) (%d %d) M%d dep %d maxWidth %d intersect (%d %d)\n",
                    //            (int)bg::get<0,0>(elem), (int)bg::get<0,1>(elem), (int)bg::get<1,0>(elem), (int)bg::get<1,1>(elem), l2, dep, maxWidth,
                    //            x, y);
                    //}
                    ///////////////////////////////////

                    if(maxWidth < width[l2])
                    {
                        cout << "width condtion..." << endl;
                        continue;
                    }
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

                        //cout << "pass direction" << endl;
                        //if(curDir == Direction::Point)
                        //{
                        //    printf("current direction : Point\n");
                        //}


                        if(curDir != Direction::Point)
                        {
                            // wire ordering 
                            //if(tracelx[dep] <= x && x <= traceux[dep])
                            //    continue;

                            //if(tracely[dep] <= y && y <= traceuy[dep])
                            //    continue;

                            if(tracelx[dep] != traceux[dep] && tracely[dep] != traceuy[dep])
                            {

                                /*
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
                                */

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
                                    printf("bound (%d %d) (%d %d) previous (%d %d) current (%d %d)\n",
                                            tracelx[dep], tracely[dep], traceux[dep], traceuy[dep], tracePtx[dep], tracePty[dep], x, y);

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

                    if(local_rtree_o.spacing_violations(bitid, xs, ys, l1, width[l1], spacing[l1], vertical))
                        c2 += SPACING_VIOLATION;

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
                            /*
                            if(tarwire->l != l2)
                            {
                                printf("intersects!!!\n");
                                cout << bg::dsv(elem) << endl;
                                cout << bg::dsv(wireseg) << endl;
                                cout << "intersection (" << ptx << " " << pty <<")" << endl;
                                if(curShape == Direction::EndPointLL)
                                    cout << "EndPointLL" << endl;
                                else if(curShape == Direction::EndPointUR)
                                    cout << "EndPOintUR" << endl;
                                else
                                    cout << "T Junction" << endl;
                                cout << endl;
                            }
                       
                            */
                            if(local_rtree_t.is_vertical(e2) != target.vertical)
                                if(!t_junction && curShape == Direction::T_Junction)
                                {
                                    //printf("intersection (%d %d)\n wire (%d %d) (%d %d) M%d \n target (%d %d) (%d %d) M%d\n",
                                    //        ptx, pty, iterPtx[e2], iterPty[e2], ptx, pty, local_rtree_t.get_layer(e2),
                                    //        tarwire->x1, tarwire->y1, tarwire->x2, tarwire->y2, tarwire->l);
                                    continue;
                                }
                        }
                        else
                        //if(!isRef)
                        {

                            if(curShape != lastRoutingShape)
                            {
                                cout << "different shape" << endl;
                                continue;
                            }
                            if(maxDepth != dep)
                            {
                                cout << "different depth" << endl;
                                continue;
                            }
                            curDir = routing_direction(x, y, ptx, pty, vertical);
                            if(traceDir[dep] != curDir)
                            {
                                cout << "different direction" << endl;
                                continue;
                            }
                        }


                        c1 += manhatan_distance(x, y, ptx, pty);
                        // spacing violation check
                        if(local_rtree_o.spacing_violations(bitid, xs, ys, l2, width[l2], spacing[l2], vertical))
                            c2 += SPACING_VIOLATION;

                        
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
                            //if(minCost <= c1 + c2)
                            //{
                            printf("<minCost %d curCost %d>\n",minCost, c1+c2);

                            //}
                            printf("\n");
#endif
                            hasMinElem = true;
                            minCost = c1 + c2;
                            minElem = e2;
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
                    printf("=========================================\n");
                    int tmp = minElem;


                    printf("current pin (%d %d) (%d %d) M%d -> target wire (%d %d) (%d %d) M%d\n",
                            curpin->llx, curpin->lly, curpin->urx, curpin->ury, curpin->l,
                            tarwire->x1, tarwire->y1, tarwire->x2, tarwire->y2, tarwire->l);

                    printf("trace layer     = { ");
                    printf("M%d (%d %d) -> ", tracelNum[depth[tmp]], lastPtx[tmp], lastPty[tmp]);
                    while(tmp != backtrace[tmp])
                    {
                        printf("M%d (%d %d) -> ", tracelNum[depth[tmp]], iterPtx[tmp], iterPty[tmp]);
                        tmp = backtrace[tmp];

                    }
                    printf("M%d (%d %d) }\n", tracelNum[depth[tmp]], iterPtx[tmp], iterPty[tmp]);

                    printf("trace dir       = { ");
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

                    printf("last            = { ");
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
                    printf("=========================================\n");
                }
#ifdef DEBUG_ROUTE_MP_TO_TP
#endif
            }
            else
            {
                solution = false;
                failed_tp++;
                printf("%s routing to tp failed\n", curbus->name.c_str());
                printf("current sequence %d\n", seq);
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
    printf("=========================================\n");
    return solution;
}



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


