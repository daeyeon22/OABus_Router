#include "typedef.h"
#include "route.h"
#include "circuit.h"
#include "func.h"

#define REPORT


using OABusRouter::is_vertical;
using OABusRouter::should_stop;

void OABusRouter::Router::route_all()
{

    int numBuses = ckt->buses.size();
    int numSuccess = 0;
    
    vector<int> busSorted(numBuses);
    #pragma omp parallel for num_threads(NUM_THREADS)
    for(int i=0; i < numBuses; i++)
    {
        busSorted[i] = i;
    }

    Circuit* cir = ckt;
    sort(busSorted.begin(), busSorted.end(), [&,cir](int left, int right){
            int y1 = cir->buses[left].lly; 
            int y2 = cir->buses[right].lly; 
            return (y1 > y2);
            });

#ifdef REPORT
    for(int i=0; i < numBuses; i++)
    {
                
    }    
#endif

    
    for(int i=0; i < numBuses; i++)
    {
        // runtime check
        if(should_stop())
        {
            break;
        }

        Bus* curBus = &ckt->buses[busSorted[i]];

        if(route_bus(curBus->id))
        {
            curBus->assign = true;
            numSuccess++;
        }
        else
        {
#ifdef REPORT
            printf("[INFO] %s routing failed\n", curBus->name.c_str());
#endif
        }

    }
    
#ifdef REPORT
    printf("- - - - ROUTE ALL REPORT - - - -\n");
    
    printf("[INFO] # buses              : %d\n", numBuses);
    printf("[INFO] # routing success    : %d\n", numSuccess);
    printf("[INFO] # routing failed     : %d\n", numBuses - numSuccess);

    printf("- - - - - - - - - - - - - - - -\n");

#endif


}

bool OABusRouter::Router::route_bus(int busid)
{

    bool routingSuccess = false;
    int m1, m2, numMPs;
    vector<pair<int,int>> mpCombs;
    Bus* curBus = &ckt->buses[busid];

    numMPs = curBus->multipins.size();

    //
    for(int i=0; i < numMPs-1; i++)
    {
        for(int j=i+1; j < numMPs; j++)
        {
            mpCombs.push_back({curBus->multipins[i], curBus->multipins[j]});
        }
    }

    if(mpCombs.size() > 1)
    {
        sort(mpCombs.begin(), mpCombs.end(), [&,this](const pair<int,int> &left, const pair<int,int> &right){
                box b1, b2;
                float dist1, dist2;
                int l1, l2;
                int dx, dy;
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
    }


    while(mpCombs.size() > 0)
    {
        if(should_stop())
        {
            break;
        }

        m1 = mpCombs.begin()->first;
        m2 = mpCombs.begin()->second;
        mpCombs.erase(mpCombs.begin());

        vector<Segment> tp;
        
        if(route_twopin_net(busid, m1, m2, tp))
        {
            routingSuccess = true;
            break;
        }
    }

    return routingSuccess;

}


int OABusRouter::Router::create_wire(int b, int t, int x1, int y1, int x2, int y2, int l, int seq, int width)
{
    Wire curWire;
    curWire.id = wires.size();
    curWire.x1 = min(x1, x2);
    curWire.y1 = min(y1, y2);
    curWire.x2 = max(x1, x2);
    curWire.y2 = max(y1, y2);
    curWire.l = l;
    curWire.seq = seq;
    curWire.bitid = b;
    curWire.busid = bit2bus[b];
    curWire.width = width;
    curWire.vertical = is_vertical(l);
    curWire.via = (x1 == x2) && (y1 == y2) ? true : false;
    wires.push_back(curWire);
    return curWire.id;
}
