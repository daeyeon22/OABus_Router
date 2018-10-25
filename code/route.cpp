#include "typedef.h"
#include "route.h"
#include "circuit.h"
#include "func.h"

#define REPORT

/* helper function */
using OABusRouter::is_vertical;
using OABusRouter::should_stop;
using OABusRouter::expand_width;
using OABusRouter::into_array;
using OABusRouter::design_ruled_area;

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
    //
    //return;

#ifdef REPORT
    printf("\n< Start reroute >\n");
#endif

    vector<int> panelty;
    get_panelty_cost(panelty);
    int iterCount = 0;
    while(iterCount++ < 10)
    {
        int deltaCost = 0;

        for(auto& busid : busSorted)
        {
            // check elapse time and runtime limit
            if(should_stop())
                break;


            cout << ckt->buses[busid].name << " panelty cost : " << get_panelty_cost(busid) << endl;


            int previous_Ps, current_Ps;
            //previous_Ps = panelty[busid];
            previous_Ps = get_panelty_cost(busid);
            
            
            if(!ckt->buses[busid].assign)
                continue;

            if(previous_Ps>0)
            {
                bool routing_success;
                dense_hash_map<int, vector<int>> backup_wires;
                dense_hash_map<int,int> backup_pin2wire = pin2wire;
                dense_hash_map<int,int> backup_wire2pin = wire2pin;
                backup_wires.set_empty_key(INT_MAX);
                for(auto& bitid : ckt->buses[busid].bits)
                {
                    backup_wires[bitid] = ckt->bits[bitid].wires;
                }

                remove_all(busid);
                routing_success = route_bus(busid);

                // if routing failed, recreate previous wires
                if(!routing_success)
                {
                    pin2wire = backup_pin2wire;
                    wire2pin = backup_wire2pin;
                    for(auto& bitid : ckt->buses[busid].bits)
                    {
                        ckt->bits[bitid].wires = backup_wires[bitid];
                        reconstruction(busid, ckt->bits[bitid].wires);
                    }
                }
                else
                {
                    // compare previous Ps and current Ps score
                    //get_panelty_cost(panelty);
                    //current_Ps = panelty[busid];

                    current_Ps = get_panelty_cost(busid);

                    // if previous score is better, go back
                    if(previous_Ps < current_Ps)
                    {
                        pin2wire = backup_pin2wire;
                        wire2pin = backup_wire2pin;
                        for(auto& bitid : ckt->buses[busid].bits)
                        {
                            ckt->bits[bitid].wires = backup_wires[bitid];
                            reconstruction(busid, ckt->bits[bitid].wires);
                        }
#ifdef REPORT
                        printf("[INFO] %s reroute failed %d cost decrease... go back!\n", ckt->buses[busid].name.c_str(), previous_Ps - current_Ps);
#endif
                    }
                    else
                    {
#ifdef REPORT
                        printf("[INFO] %s reroute success %d cost improvement!\n", ckt->buses[busid].name.c_str(), previous_Ps - current_Ps);
#endif
                        deltaCost += previous_Ps - current_Ps;

                    }
                }
            }
        }

        get_panelty_cost(panelty);
        printf("[INFO] %d iteration %d improvement\n", iterCount, deltaCost);

        if(deltaCost == 0)
            break;
    }
#ifdef REPORT
    printf("[INFO] reroute finished\n");
    printf("[INFO] routing phase 2 finished\n\n");
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


    vector<Segment> tp;
    
    while(mpCombs.size() > 0)
    {
        if(should_stop())
        {
            break;
        }

        m1 = mpCombs.begin()->first;
        m2 = mpCombs.begin()->second;
        mpCombs.erase(mpCombs.begin());

        if(!downside(m1, m2))
        {
            swap(m1,m2);
        }

        
        if(route_twopin_net(busid, m1, m2, tp))
        {
            routingSuccess = true;
            break;
        }
    }

    if(routingSuccess)
    {
        update_net_tp(busid, tp);
    }

    return routingSuccess;

}

void OABusRouter::Router::update_net_tp(int busid, vector<Segment> &tp)
{
    Bus* curBus = &ckt->buses[busid];
    int numBits = curBus->numBits;
    int numSegs = tp.size();

    for(int i=0; i < numSegs; i++)
    {
        Segment* s = &tp[i];

        for(int j=0; j < numBits; j++)
        {
            Bit* curBit = &ckt->bits[curBus->bits[j]];
            curBit->wires.push_back(s->wires[j]);
        }
    }
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

void OABusRouter::Router::remove_wire(int wireid)
{
    Wire* curwire = &wires[wireid];
    Bus* curbus = &ckt->buses[bit2bus[curwire->bitid]];
    // remove obstacle
    rtree_o.insert(WIRETYPE, curwire->bitid, curwire->x1, curwire->y1, curwire->x2, curwire->y2, curwire->l, curbus->width[curwire->l]);
}

void OABusRouter::Router::reconstruction(int busid, vector<int> &ws)
{
    Bus* curbus = &ckt->buses[busid];
    for(auto& wireid : ws)
    {
        Wire* curwire = &wires[wireid];
        // remove obstacle
        rtree_o.insert(WIRETYPE, curwire->bitid, curwire->x1, curwire->y1, curwire->x2, curwire->y2, curwire->l, curbus->width[curwire->l]);
    }
}


int OABusRouter::Router::get_panelty_cost(int busid)
{
    Bus* curBus = &ckt->buses[busid];
    int totalSPV = 0;
    int numBits = curBus->numBits;


    #pragma omp parallel for num_threads(NUM_THREADS)
    for(int i=0; i < numBits; i++)
    {
        Bit* curBit = &ckt->bits[curBus->bits[i]];
        int localSPV = 0;
        for(int j=0; j < curBit->wires.size(); j++)
        {
            Wire* curWire = &wires[curBit->wires[j]];
            
            localSPV += 
                rtree_o.num_spacing_violation(
                        curBit->id, curWire->x1, curWire->y1, curWire->x2, curWire->y2, curWire->l, 
                        curBus->width[curWire->l], spacing[curWire->l], is_vertical(curWire->l)
                        );
        }


        #pragma omp critical(GLOBAL)
        {
            totalSPV += localSPV;
        }

    }

    return totalSPV;
}

void OABusRouter::Router::get_panelty_cost(vector<int> &panelty)
{
   
    int numBuses, numLayers, numObs, numBits, numWires, numPins;
    int delta, epsilon;
    int i, j, l;
    bool vertical;

    numLayers = ckt->layers.size();
    numBuses = ckt->buses.size();
    numObs = ckt->obstacles.size();
    numBits = ckt->bits.size();

    panelty = vector<int>(numBuses,0);
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

    for(i=0; i < numBuses; i++)
        panelty[i] = Ps[i];

#ifdef REPORT 
    printf("\n------------------------------------\n");
    printf("Ps : #Spacing violations * %d\nPf : #routing failed * %d\n\n", delta, epsilon);
    for(i=0; i < numBuses; i++)
    {
        printf("%s  -> Ps %d\n", ckt->buses[i].name.c_str(), Ps[i]);
    }
    printf("total SV %d\n", SV.size());
    printf("\n------------------------------------\n");
#endif
}




bool OABusRouter::Router::leftside(int m1, int m2)
{
    return multipin2llx[m1] < multipin2llx[m2];
}

bool OABusRouter::Router::downside(int m1, int m2)
{
    return multipin2lly[m1] < multipin2lly[m2];
}
