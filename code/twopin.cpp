#include "route.h"
#include "circuit.h"
#include "func.h"

#define REPORT
//#define DEBUG_TWOPIN_NET
//#define DEBUG_NEXT_NODE
//#define DEBUG_GET_DRIVE

#define ALPHA ckt->alpha
#define BETA ckt->beta
#define GAMMA ckt->gamma
#define DELTA ckt->delta
#define EPSILON ckt->epsilon

using OABusRouter::should_stop;
using OABusRouter::is_vertical;
using OABusRouter::manhatan_distance;
using OABusRouter::direction;

bool OABusRouter::Router::route_twopin_net(int busid, int m1, int m2, vector<Segment> &tp)
{

    Bus* curBus = &ckt->buses[busid];
    MultiPin* mp1 = &ckt->multipins[m1];
    MultiPin* mp2 = &ckt->multipins[m2];
    dense_hash_map<int,int> &width = curBus->width;
    int numBits = curBus->numBits;
    int iterCount = 0;


    // initialize heap nodes
    vector<HeapNode> nodes(rtree_t.nodes.size(), HeapNode(numBits));
    #pragma omp parallel for num_threads(NUM_THREADS)
    for(int i=0; i < nodes.size(); i++)
    {
        nodes[i].id = i;
        nodes[i].nodes[0] = i;
    }
    
    // Queue, vector
    Heap PQ;
    //
   
    vector<int> bitSorted;
    int refbit_index = 0;//get_refbit_index(m1, m2, bitSorted);
    int refbit = curBus->bits[refbit_index];

    dense_hash_map<int,int> bit2pin_1;
    dense_hash_map<int,int> bit2pin_2;
    bit2pin_1.set_empty_key(INT_MAX);
    bit2pin_2.set_empty_key(INT_MAX);

    for(int i=0; i < numBits; i++)
    {
        bit2pin_1[pin2bit[mp1->pins[i]]] = mp1->pins[i];
        bit2pin_2[pin2bit[mp2->pins[i]]] = mp2->pins[i];
    }
    
    Pin* refPin1 = &ckt->pins[bit2pin_1[refbit]]; //mp1->pins[refbit_index]];
    Pin* refPin2 = &ckt->pins[bit2pin_2[refbit]]; //mp2->pins[refbit_index]];
    int minElem = INT_MAX;
    bool hasMinElem = false;
    double minCost = DBL_MAX;
    double hpwl = HPWL(refPin1->id, refPin2->id);
    double optimalCost = ALPHA + BETA;
    int lbs = lower_bound_num_segments(m1, m2);
   
    printf("\n");
    printf("[INFO] route twopin net : %s\n", curBus->name.c_str());
    printf("(OptCost) %f\n(hpwl) %f\n(lbs) %d\n", optimalCost, hpwl, lbs);
    printf("(1) %s (%d %d) (%d %d) M%d\n", refPin1->bitName.c_str(), refPin1->llx, refPin1->lly, refPin1->urx, refPin1->ury, refPin1->l);
    printf("(2) %s (%d %d) (%d %d) M%d\n", refPin2->bitName.c_str(), refPin2->llx, refPin2->lly, refPin2->urx, refPin2->ury, refPin2->l);
 
    //////////////////////////////////////////////////////////
    bool printLog = (curBus->name == "bus") ? true : false;
    //////////////////////////////////////////////////////////
    

    // Get initial heap nodes
    vector<int> initHeapNodes;
    get_access_rtree_nodes(m1, refPin1->id, initHeapNodes);
    
    for(int i=0; i < initHeapNodes.size(); i++)
    {
        int n = initHeapNodes[i];
        get_drive_node(m1, false, nodes[n]);

        nodes[n].depth = 0;
        nodes[n].CR = 0;
        nodes[n].PS = 0;
        nodes[n].CW = 0;
        nodes[n].CC = 0;
        nodes[n].CS = 0;
        nodes[n].EC = hpwl;
        nodes[n].backtrace = n;
        
        printf("\n");
        printf("- - - - Get drive node - - - -\n");
        for(int j=0; j < nodes[n].nodes.size(); j++)
        {
            Pin* curPin = &ckt->pins[bit2pin_1[curBus->bits[j]]];
            RtreeNode* curNode = rtree_t.get_node(nodes[n].nodes[j]);
            printf("%s <p%d> (%d %d) (%d %d) M%d --> <n%d> (%d %d) (%d %d) M%d\n", 
                    ckt->bits[curBus->bits[j]].name.c_str(),
                    curPin->id, curPin->llx, curPin->lly, curPin->urx, curPin->ury, curPin->l,
                    curNode->id, curNode->x1, curNode->y1, curNode->x2, curNode->y2, curNode->l);
                    
        }
        printf("\n");

        PQ.push(nodes[n]);
    }


   
    // Iterating Priority Queue
    while(!PQ.empty())
    {
        //
        iterCount++;

        HeapNode currNode = PQ.top();
        PQ.pop();

        if(currNode.id == minElem)
            break;

        int n1 = currNode.id;
        RtreeNode* rn1 = rtree_t.get_node(n1);
//#ifdef DEBUG_TWOPIN_NET
        if(printLog)
            printf("current (%d %d) (%d %d) M%d cost %f depth %d queue size %d\n", 
                    rtree_t.get_node(n1)->x1, rtree_t.get_node(n1)->y1, rtree_t.get_node(n1)->x2, rtree_t.get_node(n1)->y2, rtree_t.get_node(n1)->l,
                    currNode.cost(), currNode.depth, PQ.size());
//#endif

        // if current node is minimum element, break the loop
        if(currNode.cost() != nodes[n1].cost())
            continue;
        
        //cout << "current (" << currNode.id << ") -> " << currNode.cost() <<  " [" << PQ.size() << "]" << endl;
        //if(1.3*currNode.cost() <= optimalCost *(-1 + (1.0*iterCount/1000)))
        //    continue;

        if(iterCount > 30000 && hasMinElem)
            break;

        // search depth condition
        if(currNode.depth > lbs * 2)
            continue;

        if(currNode.cost() > minCost)
            continue;


        int x_iter_bnd1[2] = { min(currNode.xPrev[0], currNode.xPrev[numBits-1]), max(currNode.xPrev[0], currNode.xPrev[numBits-1]) };
        int y_iter_bnd1[2] = { min(currNode.yPrev[0], currNode.yPrev[numBits-1]), max(currNode.yPrev[0], currNode.yPrev[numBits-1]) };
        box bending_area(pt(x_iter_bnd1[0], y_iter_bnd1[0]), pt(x_iter_bnd1[1], y_iter_bnd1[1]));

        //filtering
        vector<int> filtered;
        #pragma omp parallel for num_threads(NUM_THREADS)
        for(int i=0; i < rn1->neighbor.size(); i++)
        {
           

            int n2 = rn1->neighbor[i]; //rtree_t.get_node(n1)->neighbor[i];
            RtreeNode* rn2 = rtree_t.get_node(n2);
            

            if(currNode.cost() >= nodes[n2].cost())
                // if iterating node's cost is smaller,
                // it doesn't need to push into the queue
                continue;


            int x = rn1->intersection[n2].first;
            int y = rn1->intersection[n2].second;
            
            // check iterating point valid
            if(bg::intersects(pt(x,y), bending_area))
                continue;

            double hpwl1 = HPWL(x, y, refPin1->id);
            double hpwl2 = HPWL(x, y, refPin2->id);

            // local search area
            if(hpwl1 + hpwl2 >= 2*hpwl)
                continue;

            #pragma omp critical(GLOBAL)
            {
                filtered.push_back(n2);
            }
        }


        if(printLog)
            cout << "#neighbor reduced from " << rtree_t.get_node(n1)->neighbor.size() << " to " << filtered.size() << endl;

        
        int NUMTHREAD = printLog ? 1 : NUM_THREADS;

        //
        #pragma omp parallel for num_threads(NUMTHREAD)//num_threads(NUM_THREADS)
        for(int i=0; i < filtered.size(); i++)
        {
            int n2 = filtered[i]; //rtree_t.get_node(n1)->neighbor[i];

            if(printLog)
            {
                cout << "n2 : " << n2 << endl;
                RtreeNode* rn = rtree_t.get_node(n2);
                printf("n%d (%d %d) (%d %d) M%d\n", rn->id, rn->x1, rn->y1, rn->x2, rn->y2, rn->l);
            }

            bool valid = true;
            bool isDestination = is_destination(n2, m2, refPin2->id);

            if(!rtree_t.is_valid({n1, n2}))
            {
                cout << "??????????????" << endl;
                exit(0);
            }


            HeapNode nextNode = nodes[n2];


            // update node infomation
            if(isDestination)
            {   

                // check whether this is destination or not.
                //cout << "get drive node" << endl;
                valid = get_drive_node(m2, true, nextNode);
            }

            if(printLog)
                cout << "1" << endl;


            // if valid, get next node's infomation
            if(valid)
            {
                //////////////////////////////////////////////////////////
                if(printLog)
                {                
                    #pragma omp critical(GLOBAL)
                    {
                        valid = get_next_node_debug(busid, isDestination, hpwl, lbs, currNode, nextNode);
                    }
                }
                else
                    valid = get_next_node(busid, isDestination, hpwl, lbs, currNode, nextNode); 

            }
            // if invalid, skip.
            if(!valid)
                continue;


            // compare previous and current cost
            //if(nextNode.cost() >= nodes[n2].cost())
            //    continue;
            

            /*
            if(!isDestination)
            {
                double hpwlEst = 0;//BETA * 1.0 * abs(rtree_t.get_node(n2)->l - refPin2->l) / lbs;
                for(int j=0; j < numBits; j++)
                {
                    hpwlEst += ALPHA * HPWL(nextNode.xPrev[j], nextNode.yPrev[j], mp2->pins[j]) / hpwl / numBits;
                }
                nextNode.EC = hpwlEst;
            }
            else
            {
                nextNode.EC = 0;
            }
            */

            

            // parallel critical 
            #pragma omp critical(GLOBAL)
            {
                // compare previous and current cost
                if(nextNode.cost() < nodes[n2].cost())
                {

                    if(isDestination && minCost > nextNode.cost())
                    {

#ifdef DEBUG_TWOPIN_NET
                        if(hasMinElem)
                        {
                            printf("Minimum Element Changed node(%d) cost(%f) dep(%d) -> node(%d) cost(%f) dep(%d)\n",
                                    minElem, minCost, nodes[minElem].depth, n2, nextNode.cost(), nextNode.depth);
                        }
#endif
                        hasMinElem = true;
                        minElem = n2;
                        minCost = nextNode.cost();

                    }

                    nodes[n2] = nextNode;
                    PQ.push(nextNode);
                }
            }
        }


        // If the solution is near optimal,
        // then break the loop.
        if(hasMinElem)
        {
            // Panelty of spacing violation is clean,
            // break the loop
            double optRatio = nodes[minElem].cost() / optimalCost;

            if(optRatio < 1.5)
            {
                printf("[INFO] Escape the loop { Opt ratio %2.2f%  / Opt %2.2f / Min %2.2f }\n", optRatio*100, optimalCost, nodes[minElem].cost());
                break;
            }
            
            /*
            if(nodes[minElem].PS == 0)
            {
                break;
            }
            */
        }
    }


    // if solution exists, update
    if(hasMinElem)
    {
        //cout << "Has Min Elem!" << endl;
        vector<Segment> tpCreated(nodes[minElem].depth + 1);
        // 
        int maxDepth = nodes[minElem].depth+1;
        int dep1, dep2, w1, w2, w3; 
        int n1, n2, x1, x2, y1, y2, x3, y3,  l1, l2;
        n2 = minElem;
        dep2 = nodes[n2].depth;

        //printf("#SPV %d\n", (int)(nodes[n2].PS/DELTA));
        for(int i=0; i < dep2+1; i++)
        {
            tpCreated[i].wires = vector<int>(numBits);
        }


#ifdef REPORT
        printf("[INFO] #iterating %d\n", iterCount);
        printf("[INFO] Start backtrace search\n");
        printf("[INFO} CW %3.2f CS %3.2f CC %3.2f PS %.1f EST %.1f\n\n", nodes[n2].CW, nodes[n2].CS, nodes[n2].CC, nodes[n2].PS, nodes[n2].EC);
#endif

        while(n2 != nodes[n2].backtrace)
        {
            n1 = nodes[n2].backtrace;
            dep1 = nodes[n1].depth;
            dep2 = nodes[n2].depth;

            for(int i=0; i < numBits; i++)
            {

                x1 = nodes[n1].xPrev[i];
                y1 = nodes[n1].yPrev[i];
                l1 = rtree_t.get_node(n1)->l;
                x2 = nodes[n2].xPrev[i];
                y2 = nodes[n2].yPrev[i];
                l2 = rtree_t.get_node(n2)->l;

                // add
                tpCreated[dep1].wires[i] = create_wire(curBus->bits[i], n1, x1, y1, x2, y2, l1, i, curBus->width[l1]);
                // add into rtree
                rtree_o.insert(WIRETYPE, curBus->bits[i], x1, y1, x2, y2, l1, curBus->width[l1]);


                if(n2 == minElem)
                {
                    x3 = nodes[n2].xLast[i];
                    y3 = nodes[n2].yLast[i];
                    // add
                    tpCreated[dep2].wires[i] = create_wire(curBus->bits[i], n2, x2, y2, x3, y3, l2, i, curBus->width[l2]);

                    // add into rtree
                    rtree_o.insert(WIRETYPE, curBus->bits[i], x2, y2, x3, y3, l2, curBus->width[l2]);
                }

                w2 = tpCreated[dep2].wires[i];
                w1 = tpCreated[dep1].wires[i];

                if(dep1 == 0)
                {
                    int p1 = bit2pin_1[curBus->bits[i]]; //refbit]; //mp1->pins[bit2pini];
                    pin2wire[p1] = w1;
                    wire2pin[w1] = p1;
                    wires[w1].intersection[PINTYPE] = { nodes[n1].xPrev[i], nodes[n1].yPrev[i] };
                }

                if(dep2 == maxDepth-1)
                {
                    int p2 = bit2pin_2[curBus->bits[i]]; //mp2->pins[i];
                    pin2wire[p2] = w2;
                    wire2pin[w2] = p2;
                    wires[w2].intersection[PINTYPE] = { nodes[n2].xLast[i], nodes[n2].yLast[i] };
                }

                // VIA inserttion
                 


                // intersection (w1,w2)
                wires[w2].intersection[w1] = {x2, y2};
                wires[w1].intersection[w2] = {x2, y2};
            }
            n2 = n1;
        }

#ifdef DEBUG_TWOPIN_NET
        for(int dep=0; dep < maxDepth; dep++)
        {
            printf("\n<Depth %d>\n", dep);
            for(int i=0; i < numBits; i++)
            {
                x1 = wires[tpCreated[dep].wires[i]].x1;
                x2 = wires[tpCreated[dep].wires[i]].x2;
                y1 = wires[tpCreated[dep].wires[i]].y1;
                y2 = wires[tpCreated[dep].wires[i]].y2;
                l1 = wires[tpCreated[dep].wires[i]].l;

                printf("(%d %d) (%d %d) M%d\n", x1, y1, x2, y2, l1);
            }
        }
#endif

        tp = tpCreated;

    }
    else
    {
        cout << "Failed" << endl;
    }

    //////////////////////
    if(printLog)
        exit(0);
    
    return hasMinElem;
}




int OABusRouter::Router::lower_bound_num_segments(int m1, int m2)
{
    MultiPin* mp1 = &ckt->multipins[m1];
    MultiPin* mp2 = &ckt->multipins[m2];
    int lDiff = abs(mp1->l - mp2->l);
    int lReq = (mp1->align == mp2->align) ? 3 : 2;

    if(lDiff > lReq)
        return abs(lDiff - lReq) + lReq;
    else
        return lReq;
}

bool OABusRouter::Router::is_destination(int n, int m, int p)
{
    Bus* curBus = &ckt->buses[pin2bus[p]];
    Pin* curPin = &ckt->pins[p];
    MultiPin* curMP = &ckt->multipins[m];
    int xPin[2] = {curPin->llx, curPin->urx};
    int yPin[2] = {curPin->lly, curPin->ury};
    int lPin = curPin->l;

    RtreeNode* curNode = rtree_t.get_node(n);
    int xSeg[2] = {curNode->x1, curNode->x2};
    int ySeg[2] = {curNode->y1, curNode->y2};
    int lSeg = curNode->l;



    if(abs(lPin-lSeg) > 1)
        return false;
    
    if(curMP->needVia)
    {
        if(lPin == lSeg)
            return false;
    }
    else
    {
        if(lPin != lSeg)
            return false;
    }


    if(lPin == lSeg)
    {
        xSeg[0] -= is_vertical(lSeg) ? curBus->width[lSeg]/2 : 0;
        xSeg[1] += is_vertical(lSeg) ? curBus->width[lSeg]/2 : 0;
        ySeg[0] -= is_vertical(lSeg) ? 0 : curBus->width[lSeg]/2;
        ySeg[1] += is_vertical(lSeg) ? 0 : curBus->width[lSeg]/2;
        box g1(pt(xPin[0], yPin[0]), pt(xPin[1], yPin[1]));
        box g2(pt(xSeg[0], ySeg[0]), pt(xSeg[1], ySeg[1]));
        return bg::intersects(g1, g2);
    }
    else
    {
        box g1(pt(xPin[0], yPin[0]), pt(xPin[1], yPin[1]));
        seg g2(pt(xSeg[0], ySeg[0]), pt(xSeg[1], ySeg[1]));
        return bg::intersects(g1, g2);
    }
    ////
}


void OABusRouter::Router::get_access_point(int mp, bool last, HeapNode& curr)
{
    int x, y, numBits;
    MultiPin* curMP = &ckt->multipins[mp];
    numBits = curMP->pins.size();
    

    if(last)
    {
        curr.xLast = vector<int>(numBits);
        curr.yLast = vector<int>(numBits);
    }

    for(int i=0; i < numBits; i++)
    {
        Pin* curPin = &ckt->pins[curMP->pins[i]];
        RtreeNode* curRN = rtree_t.get_node(curr.nodes[i]);
        int xPin[] = {curPin->llx, curPin->urx};
        int yPin[] = {curPin->lly, curPin->ury};
        int lPin = curPin->l;
        int xSeg[] = {curRN->x1, curRN->x2};
        int ySeg[] = {curRN->y1, curRN->y2};
        int lSeg = curRN->l;

        pin_access_point(xPin, yPin, lPin, xSeg, ySeg, lSeg, x, y);
        if(last)
        {
            curr.xLast[i] = x;
            curr.yLast[i] = y;
        }
        else
        {
            curr.xPrev[i] = x;
            curr.yPrev[i] = y;
        }

    }
}


void OABusRouter::Router::get_access_rtree_nodes(int mp, int p, vector<int> &nodes)
{
    MultiPin* curMP = &ckt->multipins[mp];
    Pin* curPin = &ckt->pins[p];
    Bus* curBus = &ckt->buses[pin2bus[p]];
    int xPin[2] = {curPin->llx, curPin->urx};
    int yPin[2] = {curPin->lly, curPin->ury};
    int lPin = curPin->l;

    vector<int> tarLayers;
    if(curMP->needVia)
    {
        if(lPin-1 >= 0)
        {
            tarLayers.push_back(lPin-1);
        }

        if(lPin+1 < ckt->layers.size())
        {
            tarLayers.push_back(lPin+1);
        }
    }
    else
    {
        xPin[0] -= (curMP->align == VERTICAL) ? 0 : curBus->width[lPin]/2;
        xPin[1] += (curMP->align == VERTICAL) ? 0 : curBus->width[lPin]/2;
        yPin[0] -= (curMP->align == VERTICAL) ? curBus->width[lPin]/2 : 0;
        yPin[1] += (curMP->align == VERTICAL) ? curBus->width[lPin]/2 : 0;
        tarLayers.push_back(lPin);
    }

    box pinArea(pt(xPin[0], yPin[0]), pt(xPin[1], yPin[1]));

    for(auto l : tarLayers)
    {
        vector<pair<seg, int>> queries;
        vector<pair<seg, int>> tmp;
        rtree_t[l]->query(bgi::intersects(pinArea), back_inserter(queries));
        
        for(int j=0; j < queries.size(); j++)
        {
            RtreeNode* n = rtree_t.get_node(queries[j].second);
            if(n->width >= curBus->width[n->l])
            {
                tmp.push_back(queries[j]);
            }
        }

        queries = tmp;
        
        if(queries.size() > 1)
        {
            sort(queries.begin(), queries.end(), [&, xPin, yPin](const pair<seg,int> &left, const pair<seg,int> &right){
                    int xCenter = (xPin[0] + xPin[1])/2;
                    int yCenter = (yPin[0] + yPin[1])/2;
                    return bg::distance(left.first, pt(xCenter, yCenter)) < bg::distance(right.first, pt(xCenter, yCenter));
                    });
        }

        nodes.push_back(queries[0].second);
    }


}


bool OABusRouter::Router::get_drive_node(int mp, bool last, HeapNode& curr)
{
    RtreeNode* curRN = rtree_t.get_node(curr.id);
    MultiPin* curMP = &ckt->multipins[mp];
    int numBits = curMP->pins.size();
    int align = curMP->align;
    bool findAll = true;
    vector<int> nodes(numBits);
    vector<int> xPt(numBits);
    vector<int> yPt(numBits);

    dense_hash_map<int,int> bit2pin;
    bit2pin.set_empty_key(INT_MAX);
    for(int i=0; i < numBits; i++)
    {
        bit2pin[pin2bit[curMP->pins[i]]] = curMP->pins[i];
    }


    for(int i=0; i < numBits; i++)
    {
        Bus* curBus = &ckt->buses[curMP->busid];
        Pin* curPin = &ckt->pins[bit2pin[curBus->bits[i]]];
        
        int xPin[2] = {curPin->llx, curPin->urx};
        int yPin[2] = {curPin->lly, curPin->ury};
        int lPin = curPin->l;
        int lSeg = curRN->l;
        vector<pair<seg,int>> queries;

        if(lPin == lSeg)
        {
            xPin[0] -= (align == VERTICAL) ? 0 : curBus->width[lSeg]/2;
            xPin[1] += (align == VERTICAL) ? 0 : curBus->width[lSeg]/2;
            yPin[0] -= (align == VERTICAL) ? curBus->width[lSeg]/2 : 0;
            yPin[1] += (align == VERTICAL) ? curBus->width[lSeg]/2 : 0;
        }

        int x, y;
        int xSeg[2], ySeg[2];
        if(i != 0)
        {
            int reflength = manhatan_distance(curRN->x1, curRN->y1, curRN->x2, curRN->y2);

            box pinArea(pt(xPin[0], yPin[0]), pt(xPin[1], yPin[1]));

            rtree_t[lSeg]->query(bgi::intersects(pinArea), back_inserter(queries));
            vector<pair<seg,int>> tmp;
            for(int j=0; j < queries.size(); j++)
            {
                RtreeNode* n = rtree_t.get_node(queries[j].second);
                int length = manhatan_distance(n->x1, n->y1, n->x2, n->y2);
                if(1.0* abs(reflength-length) / length > 0.1)
                {
                    continue;
                }

                if(n->width >= curBus->width[lSeg])
                {
                    tmp.push_back(queries[j]);
                }
            }

            queries = tmp;

            if(queries.size() == 0)
            {
                findAll = false;
                break;
            }


            if(queries.size() > 1)
            {
                sort(queries.begin(), queries.end(), [&, xPin, yPin](const pair<seg,int> &left, const pair<seg,int> &right){
                        int xCenter = (xPin[0] + xPin[1])/2;
                        int yCenter = (yPin[0] + yPin[1])/2;
                        return bg::distance(left.first, pt(xCenter, yCenter)) < bg::distance(right.first, pt(xCenter, yCenter));
                        });
            }
            pts(queries[0].first, xSeg[0], ySeg[0], xSeg[1], ySeg[1]);
            nodes[i] = queries[0].second;
        }       
        else
        {
            nodes[i] = curr.id;
            xSeg[0] = rtree_t.get_node(curr.id)->x1;
            xSeg[1] = rtree_t.get_node(curr.id)->x2;
            ySeg[0] = rtree_t.get_node(curr.id)->y1;
            ySeg[1] = rtree_t.get_node(curr.id)->y2;
        }
            
        pin_access_point(xPin, yPin, lPin, xSeg, ySeg, lSeg, x, y);

        xPt[i] = x;
        yPt[i] = y;

    }

    if(curr.id != nodes[0])
    {
        cout << "curid : " << curr.id << " " << nodes[0] << endl;
        cout << "- - - - - - - - -" << endl;
        for(int j=0; j < numBits; j++)
        {
            cout << nodes[j] << endl;
        }
        cout << "- - - - - - - - -" << endl;
        
        cout << "12121" << endl;


        exit(0);
    }

    

    if(findAll)
    {
        curr.nodes = nodes;

        if(last)
        {
            curr.xLast = xPt;
            curr.yLast = yPt;
        }
        else
        {
            curr.xPrev = xPt;
            curr.yPrev = yPt;
        }
    }

    if(curr.id != nodes[0])
    {
        cout << "3232323" << endl;
        exit(0);
    }




#ifdef DEBUG_GET_DRIVE
    if(findAll)
    {
        printf("\n- - - -   MultiPin/Node   - - - - \n");

        for(int i=0; i < curMP->pins.size(); i++)
        {
            Pin* curPin = &ckt->pins[curMP->pins[i]];
            RtreeNode* curNode = rtree_t.get_node(curr.nodes[i]);
            if(last)
            {
                printf("(2) %s (%d %d) (%d %d) M%d -> (%d %d) (%d %d) M%d intersection (%d %d)\n", 
                        ckt->bits[pin2bit[curPin->id]].name.c_str(), curPin->llx, curPin->lly, curPin->urx, curPin->ury, curPin->l,
                        curNode->x1, curNode->y1, curNode->x2, curNode->y2, curNode->l, curr.xLast[i], curr.yLast[i]);
            }
            else
            {

                printf("(1) %s (%d %d) (%d %d) M%d -> (%d %d) (%d %d) M%d intersection (%d %d)\n", 
                        ckt->bits[pin2bit[curPin->id]].name.c_str(), curPin->llx, curPin->lly, curPin->urx, curPin->ury, curPin->l,
                        curNode->x1, curNode->y1, curNode->x2, curNode->y2, curNode->l, curr.xPrev[i], curr.yPrev[i]);
            }
        }
    }
    else
    {
        cout << "Invalid get drive node" << endl;

        printf("\n- - - -   MultiPin/Node   - - - - \n");
        for(int i=0; i < curMP->pins.size(); i++)
        {
            Pin* curPin = &ckt->pins[curMP->pins[i]];
            printf("%s (%d %d) (%d %d) M%d\n", 
                    ckt->bits[pin2bit[curPin->id]].name.c_str(), curPin->llx, curPin->lly, curPin->urx, curPin->ury, curPin->l);
        }

        for(int i=0; i < curMP->pins.size(); i++)
        {
            Pin* curPin = &ckt->pins[curMP->pins[i]];
            RtreeNode* curNode = rtree_t.get_node(curr.nodes[i]);
            printf("%s (%d %d) (%d %d) M%d\n", 
                    ckt->bits[pin2bit[curPin->id]].name.c_str(), curNode->x1, curNode->y1, curNode->x2, curNode->y2, curNode->l);
        }
        exit(0);
    }
#endif

    return findAll;
}

int OABusRouter::Router::get_routing_direction(int x1, int y1, int x2, int y2)
{
    if(x1 == x2 && y1 == y2)
        return Direction::Stack;    
    
    if(x1 == x2)
    {
        if(y1 < y2)
            return Direction::Up;
        else
            return Direction::Down;
    }
    else if(y1 == y2)
    {
        if(x1 < x2)
            return Direction::Right;
        else
            return Direction::Left;
    }
    else
    {
        cout << "Invalid points ..." << endl;
        cout << x1 << " " << y1 << " " << x2 << " " << y2 << endl;
        //exit(0);
        return -1;
    }

}

bool OABusRouter::Router::get_next_node_debug(int busid, bool fixed, double hpwl, int lbs, HeapNode& prev, HeapNode& next)
{
    Bus* curBus = &ckt->buses[busid];
    int x1, y1, x2, y2, x3, y3, l1, l2, l3;
    int bitid, rouDir, numBits, iter;
    double WL, CW, CS, CC, PS, CR, EC;
    bool valid = true;
    bool vertical1 = is_vertical(rtree_t.get_node(prev.id)->l);
    bool vertical2 = is_vertical(rtree_t.get_node(next.id)->l);

    int nPrev;

    vector<int> nodes(numBits);
    RtreeNode *n1, *n2;
    numBits = curBus->numBits;
     
    PS = 0; // prev.PS;
    CW = 0;
    CC = 0;
    CS = 0;
    EC = 0; 

  

    for(int i=0; i < numBits; i++)
    {
        bitid = curBus->bits[i];
        n1 = rtree_t.get_node(prev.nodes[i]);
        n2 = rtree_t.get_node(next.nodes[i]);
        

        if(!rtree_t.is_valid({n1->id, n2->id}))
            return false;
        
        x1 = prev.xPrev[i];
        y1 = prev.yPrev[i];
        l1 = n1->l;
        x2 = n2->intersection[n1->id].first;
        y2 = n2->intersection[n1->id].second;
        l2 = n2->l;

        rouDir = get_routing_direction(x1, y1, x2, y2);
        
        if(rouDir == -1)
        {
            #pragma omp critical(GLOBAL)
            {
                get_next_node_debug(busid, fixed, hpwl, lbs, prev, next);
                //exit(0);
            }
            return false;
        }
        
        next.xPrev[i] = x2;
        next.yPrev[i] = y2;

        // calculate PS and WL
        PS += rtree_o.num_spacing_violation(bitid, x1, y1, x2, y2, l1, curBus->width[l1], spacing[l1], is_vertical(l1));
        CW += manhatan_distance(x1, y1, x2, y2);
        
        if(fixed)
        {
            // rtree nodes fixed (drive node)
            x3 = next.xLast[i];
            y3 = next.yLast[i];
            PS += rtree_o.num_spacing_violation(bitid, x2, y2, x3, y3, l2, curBus->width[l2], spacing[l2], is_vertical(l2));
            CW += manhatan_distance(x2, y2, x3, y3);
        }
        else
        {
            // rtree nodes not fixed (non-drive node)
            if(i != numBits-1)
            {
                if(l1 == l2)
                {
                    int iter;
                    bool lCorner = (x2 == n1->x1 && y2 == n1->y1) ? true : false;

                    if(!rtree_t.get_extension(prev.nodes[i+1], iter, lCorner))
                    {
                        valid = false;
                        break;
                    }

                    next.nodes[i+1] = iter;
                    nPrev = iter;
                }
                else
                {
                    bool upperNode;
                    // get next node
                    if(rouDir == Direction::Left || rouDir == Direction::Down)
                    {
                        upperNode = false;
                    }
                    else if(rouDir == Direction::Right || rouDir == Direction::Up)
                    {
                        upperNode = true;
                    }
                    else
                    {
                        upperNode = 
                            (vertical1 && (prev.xPrev[i] < prev.xPrev[i+1])) || 
                            (!vertical1 && (prev.yPrev[i] < prev.yPrev[i+1])) ? true : false;
                    }

                    int n_iter, n_prev, n_next;
                    n_prev = prev.nodes[i+1];
                    n_iter = next.nodes[i];

                    
                    //RtreeNode* n_iter = rtree_t.get_node(next.nodes[i]);
                    //RtreeNode* n_prev = rtree_t.get_node(prev.nodes[i+1]);
                    //RtreeNode* n_next;
                    

                    int count=0;
                    while(true)
                    {
                    
                        if(count++ > 5)
                        {
                            valid = false;
                            break;
                        }

                        // get next
                        //if(!rtree_t.search_node(n_prev, n_next, n_iter, curBus->width[l2], spacing[l2], upperNode))
                        if(!rtree_t.search_node2(n_iter, n_next, n_prev, curBus->width[l2], spacing[l2], upperNode))
                        {
                            valid = false;
                            break;
                        }

                        x3 = rtree_t.get_node(n_prev)->intersection[n_next].first;
                        y3 = rtree_t.get_node(n_prev)->intersection[n_next].second;
                        l3 = rtree_t.get_node(n_prev)->l;
                        //x3 = n_prev->intersection[n_next->id].first;
                        //y3 = n_prev->intersection[n_next->id].second;
                        //l3 = n_prev->l;
                        break;

                        /*
                        if(rtree_o.spacing_violation_clean(bitid, x3, y3, x3, y3, l3, curBus->width[l3], spacing[l3], is_vertical(l3)))
                        {
                            // if current point is clean, break the loop
                            break;
                        }
                        */
                        n_iter = n_next;
                    }

                    //
                    if(!valid)
                    {
                        break;
                    }

                    //next.nodes[i+1] = n_next->id; //iter;
                    next.nodes[i+1] = n_next; //iter;
                }
            }
        }
    }
    
    
    next.backtrace = prev.id;
    next.depth = prev.depth + 1;
    // update variables
    next.PS = prev.PS + DELTA * PS; //DELTA * PS;
    next.CW = prev.CW + ALPHA * (CW/hpwl) / numBits; //ALPHA * (CW/hpwl) / numBits; 
    next.CC = 0;
    next.CS = 1.0 * BETA * (next.depth + 1) / lbs;
    next.EC = 0;

    if(valid)
    {
        printf("- - - - Current Node - - - -\n");
        for(int i=0; i < numBits; i++)
        {
            RtreeNode* n = rtree_t.get_node(prev.nodes[i]);
            printf("%5s -> n%4d (%d %d) (%d %d) M%d iter (%d %d) cost %f\n", 
                    ckt->bits[curBus->bits[i]].name.c_str(), prev.nodes[i], n->x1, n->y1, n->x2, n->y2, n->l, prev.xPrev[i], prev.yPrev[i], prev.cost());
        }

        printf("\n");
        printf("- - - -  Next Node   - - - -\n");
        for(int i=0; i < numBits; i++)
        {
            RtreeNode* n = rtree_t.get_node(next.nodes[i]);
            printf("%5s -> n%4d (%d %d) (%d %d) M%d iter (%d %d) cost %f\n", 
                    ckt->bits[curBus->bits[i]].name.c_str(), next.nodes[i], n->x1, n->y1, n->x2, n->y2, n->l, next.xPrev[i], next.yPrev[i], next.cost());
        }
        printf("\n\n");

    }
    else
    {
        printf("[INFO] invalid iterating...\n");
        //exit(0);
    }


    return valid;

}

bool OABusRouter::Router::get_next_node(int busid, bool fixed, double hpwl, int lbs, HeapNode& prev, HeapNode& next)
{
    Bus* curBus = &ckt->buses[busid];
    int x1, y1, x2, y2, x3, y3, l1, l2, l3;
    int bitid, rouDir, numBits, iter;
    double WL, CW, CS, CC, PS, CR, EC;
    bool valid = true;
    bool vertical1 = is_vertical(rtree_t.get_node(prev.id)->l);
    bool vertical2 = is_vertical(rtree_t.get_node(next.id)->l);

    int nPrev;

    vector<int> nodes(numBits);
    RtreeNode *n1, *n2;
    numBits = curBus->numBits;
     
    PS = 0; // prev.PS;
    CW = 0;
    CC = 0;
    CS = 0;
    EC = 0; 

   
    for(int i=0; i < numBits; i++)
    {
        bitid = curBus->bits[i];
        n1 = rtree_t.get_node(prev.nodes[i]);
        n2 = rtree_t.get_node(next.nodes[i]);
        
        if(!rtree_t.is_valid({n1->id, n2->id}))
            return false;
        
        x1 = prev.xPrev[i];
        y1 = prev.yPrev[i];
        l1 = n1->l;
        x2 = n2->intersection[n1->id].first;
        y2 = n2->intersection[n1->id].second;
        l2 = n2->l;

        
        rouDir = get_routing_direction(x1, y1, x2, y2);
        
        if(rouDir == -1)
        {
            #pragma omp critical(GLOBAL)
            {
                get_next_node_debug(busid, fixed, hpwl, lbs, prev, next);
                //exit(0);
            }
            return false;
        }
        
        next.xPrev[i] = x2;
        next.yPrev[i] = y2;

        // calculate PS and WL
        PS += rtree_o.num_spacing_violation(bitid, x1, y1, x2, y2, l1, curBus->width[l1], spacing[l1], is_vertical(l1));
        CW += manhatan_distance(x1, y1, x2, y2);
        
        if(fixed)
        {
            // rtree nodes fixed (drive node)
            x3 = next.xLast[i];
            y3 = next.yLast[i];
            PS += rtree_o.num_spacing_violation(bitid, x2, y2, x3, y3, l2, curBus->width[l2], spacing[l2], is_vertical(l2));
            CW += manhatan_distance(x2, y2, x3, y3);
        }
        else
        {
            // rtree nodes not fixed (non-drive node)
            if(i != numBits-1)
            {
                if(l1 == l2)
                {
                    int iter;
                    bool lCorner = (x2 == n1->x1 && y2 == n1->y1) ? true : false;

                    if(!rtree_t.get_extension(prev.nodes[i+1], iter, lCorner))
                    {
                        valid = false;
                        break;
                    }

                    next.nodes[i+1] = iter;
                    nPrev = iter;
                }
                else
                {
                    bool upperNode;
                    // get next node
                    if(rouDir == Direction::Left || rouDir == Direction::Down)
                    {
                        upperNode = false;
                    }
                    else if(rouDir == Direction::Right || rouDir == Direction::Up)
                    {
                        upperNode = true;
                    }
                    else
                    {
                        upperNode = 
                            (vertical1 && (prev.xPrev[i] < prev.xPrev[i+1])) || 
                            (!vertical1 && (prev.yPrev[i] < prev.yPrev[i+1])) ? true : false;
                    }

                    int n_iter, n_prev, n_next;
                    n_prev = prev.nodes[i+1];
                    n_iter = next.nodes[i];

                    
                    //RtreeNode* n_iter = rtree_t.get_node(next.nodes[i]);
                    //RtreeNode* n_prev = rtree_t.get_node(prev.nodes[i+1]);
                    //RtreeNode* n_next;
                    

                    int count=0;
                    while(true)
                    {
                    
                        if(count++ > 5)
                        {
                            valid = false;
                            break;
                        }

                        // get next
                        //if(!rtree_t.search_node(n_prev, n_next, n_iter, curBus->width[l2], spacing[l2], upperNode))
                        if(!rtree_t.search_node(n_iter, n_next, n_prev, curBus->width[l2], spacing[l2], upperNode))
                        {
                            valid = false;
                            break;
                        }

                        x3 = rtree_t.get_node(n_prev)->intersection[n_next].first;
                        y3 = rtree_t.get_node(n_prev)->intersection[n_next].second;
                        l3 = rtree_t.get_node(n_prev)->l;
                        //x3 = n_prev->intersection[n_next->id].first;
                        //y3 = n_prev->intersection[n_next->id].second;
                        //l3 = n_prev->l;
                        break;


                        if(rtree_o.spacing_violation_clean(bitid, x3, y3, x3, y3, l3, curBus->width[l3], spacing[l3], is_vertical(l3)))
                        {
                            // if current point is clean, break the loop
                            break;
                        }

                        n_iter = n_next;
                    }

                    //
                    if(!valid)
                    {
                        break;
                    }

                    //next.nodes[i+1] = n_next->id; //iter;
                    next.nodes[i+1] = n_next; //iter;
                }
            }
        }
    }
    
    
    next.backtrace = prev.id;
    next.depth = prev.depth + 1;
    // update variables
    next.PS = prev.PS + DELTA * PS; //DELTA * PS;
    next.CW = prev.CW + ALPHA * (CW/hpwl) / numBits; //ALPHA * (CW/hpwl) / numBits; 
    next.CC = 0;
    next.CS = 1.0 * BETA * (next.depth + 1) / lbs;
    next.EC = 0;

#ifdef DEBUG_NEXT_NODE
    if(valid)
    {
        printf("- - - - Current Node - - - -\n");
        for(int i=0; i < numBits; i++)
        {
            RtreeNode* n = rtree_t.get_node(prev.nodes[i]);
            printf("%5s -> n%4d (%d %d) (%d %d) M%d iter (%d %d) cost %f\n", 
                    ckt->bits[curBus->bits[i]].name.c_str(), prev.nodes[i], n->x1, n->y1, n->x2, n->y2, n->l, prev.xPrev[i], prev.yPrev[i], prev.cost());
        }

        printf("\n");
        printf("- - - -  Next Node   - - - -\n");
        for(int i=0; i < numBits; i++)
        {
            RtreeNode* n = rtree_t.get_node(next.nodes[i]);
            printf("%5s -> n%4d (%d %d) (%d %d) M%d iter (%d %d) cost %f\n", 
                    ckt->bits[curBus->bits[i]].name.c_str(), next.nodes[i], n->x1, n->y1, n->x2, n->y2, n->l, next.xPrev[i], next.yPrev[i], next.cost());
        }
        printf("\n\n");

    }
    else
    {
        printf("[INFO] invalid iterating...\n");
        //exit(0);
    }
#endif


    return valid;
}

double OABusRouter::Router::HPWL(int x, int y, int p)
{
    Pin* curPin = &ckt->pins[p];
    double w = min( abs(curPin->llx - x), abs(curPin->urx - x));
    double h = min( abs(curPin->lly - y), abs(curPin->ury - y));
    return w + h;
}

double OABusRouter::Router::HPWL(int p1, int p2)
{
    Pin* pin1 = &ckt->pins[p1];
    Pin* pin2 = &ckt->pins[p2];

    double w = min( abs(pin1->llx - pin2->urx), abs(pin1->urx - pin2->llx) );
    double h = min( abs(pin1->lly - pin2->ury), abs(pin1->ury - pin2->lly) );
    return w + h;
}



void OABusRouter::Router::pin_access_point(int xPin[], int yPin[], int lPin, int xSeg[], int ySeg[], int lSeg, int &x, int &y)
{
    bool vertical = (xSeg[0] == xSeg[1]) ? true : false;
    if(vertical)
    {
        x = xSeg[0];
        y = (yPin[0] + yPin[1])/2;
    }
    else
    {
        x = (xPin[0] + xPin[1])/2;
        y = ySeg[0];
    }
}

void OABusRouter::Heap::push(vector<HeapNode>& next)
{
    for(int i=0; i < next.size(); i++)
    {
        PQ.push(next[i]);
    }
}

void OABusRouter::Heap::push(HeapNode& next)
{
    PQ.push(next);
}

void OABusRouter::Heap::pop()
{
    PQ.pop();
}

OABusRouter::HeapNode OABusRouter::Heap::top()
{
    return PQ.top();
}

bool OABusRouter::Heap::empty()
{
    return PQ.size() == 0 ? true : false;
}

int OABusRouter::Heap::size()
{
    return PQ.size();
}

double OABusRouter::HeapNode::cost()
{
    if(backtrace == -1)
        return DBL_MAX;
    else
        return PS + CW + CS;
}
    /*
    if(CR == DBL_MAX && PS == DBL_MAX)
        return DBL_MAX;
    else
        return CR + PS;
    */

/*
bool OABusRouter::Router::flip(int m1, int m2)
{
    namespace bi = boost:icl;
    MultiPin* mp1 = &ckt->multipins[m1];
    MultiPin* mp2 = &ckt->multipins[m2];
    int align1, align2, l1, l2, refbit_index, refbit;
    l1 = mp1->l;
    l2 = mp2->l;
    align1 = mp1->align;
    align2 = mp2->align;

    Circuit* cir = ckt;
    sorted.clear();
    vector<int> pins = mp1->pins;
    auto cmp =  [&,cir, align1](int left, int right){
        Pin* p1 = &cir->pins[left];
        Pin* p2 = &cir->pins[right];
        return (align1 == VERTICAL) ? p1->lly < p2->lly : p1->llx < p2->llx;
    };
    
    bool increaseOrder;

    sort(pins.begin(), pins.end(), cmp);

    if(align1 == align2 && l1 == l2)
    {
        int ll1[2], ur1[2], ll2[2], ur2[2];
        ll1[0] = multipin2llx[m1];
        ll1[1] = multipin2lly[m1];
        ll2[0] = multipin2llx[m2];
        ll2[1] = multipin2lly[m2];
        ur1[0] = multipin2urx[m1];
        ur1[1] = multipin2ury[m1];
        ur2[0] = multipin2urx[m2];
        ur2[1] = multipin2ury[m2];

        int p1, p2, p3, p4;
        p1 = (align1 == VERTICAL) ? ll1[1] : ll1[0];
        p2 = (align1 == VERTICAL) ? ur1[1] : ur1[0];
        p3 = (align2 == VERTICAL) ? ll2[1] : ll2[0];
        p4 = (align2 == VERTICAL) ? ur2[1] : ur2[0];
        bool areaOverlaps = bi::intersects(bi::interval<int>::closed(p1,p2), bi::interval<int>::closed(p3,p4));
        bool below;

        if(align1 == VERTICAL)
        {
            if(ll1[0] < ll2[0])
            {
                return true; 
            }
            else
            {
                return false;
            }
        }
        else
        {
            if(ll1[1] < ll1[1])
            {

            }
        }

        bool areaOverlaps = bi::intersects(bi::interval<int>::closed(corner1[0], corner1[1]), bi::interval<int>::closed(corner2[0], corner2[1]));
        
        
        bool leftSide = 
        if(areaOverlaps)
        {
            
        }

        refbit = areaOverlaps ? pin2bit[sorted[sorted.size()-1]] : pin2bit[sorted[0]];
    }

}

int OABusRouter::Router::get_refbit_index(int m1, int m2, vector<int> &sorted)
{
    namespace bi = boost:icl;
    MultiPin* mp1 = &ckt->multipins[m1];
    MultiPin* mp2 = &ckt->multipins[m2];
    int align1, align2, l1, l2, refbit_index, refbit;
    l1 = mp1->l;
    l2 = mp2->l;
    align1 = mp1->align;
    align2 = mp2->align;

    Circuit* cir = ckt;
    sorted.clear();
    vector<int> pins = mp1->pins;
    auto cmp =  [&,cir, align1](int left, int right){
        Pin* p1 = &cir->pins[left];
        Pin* p2 = &cir->pins[right];
        return (align1 == VERTICAL) ? p1->lly < p2->lly : p1->llx < p2->llx;
    };
    
    bool increaseOrder;

    sort(pins.begin(), pins.end(), cmp);

    if(align1 == align2 && l1 == l2)
    {
        int corner1[2], corner2[2];
        corner1[0] = (align1 == VERTICAL) ? multipin2lly[m1] : multipin2llx[m1];
        corner1[1] = (align1 == VERTICAL) ? multipin2ury[m1] : multipin2urx[m1];
        corner2[0] = (align2 == VERTICAL) ? multipin2lly[m2] : multipin2llx[m2];
        corner2[1] = (align2 == VERTICAL) ? multipin2ury[m2] : multipin2urx[m2];

        bool areaOverlaps = bi::intersects(bi::interval<int>::closed(corner1[0], corner1[1]), bi::interval<int>::closed(corner2[0], corner2[1]));
        
        refbit = areaOverlaps ? pin2bit[sorted[sorted.size()-1]] : pin2bit[sorted[0]];
        increaseOrder = areaOverlaps ? false : true;

        if(areaOverlaps)
        {
            for(int i=0; i < 
        }

    }
    else
    {
        refbit = pin2bit[sorted[0]];
    }

    Bus* curBus = &ckt->buses[mp1->busid];
    for(int i=0; i < curBus->numBits; i++)
    {
        if(curBus->bits[i] == refbit)
        {
            refbit_index = i;
            break;
        }
    }

    return refbit_index;
}
*/

/*
void OABusRouter::Router::get_iterating_segment(int busid, HeapNode* current, vector<HeapNode> &nodes, vector<HeapNode> &next)
{
    Bus* curBus = &ckt->buses[busid];
    int i, i1, i2, rDir, numBits;
    int x1, y1, x2, y2, l1, l2;
    int prevOffset, currOffset;
    int PS, WL, CC, bitid;
    bool valid;
    vector<int> rn(numBits);
    RtreeNode *n1, *n2;
    RtreeNode *iter;

    numBits = ckt->buses[busid].numBits;
    n1 = rtree_t.get_node(current->id);
    x1 = current->xPrev[0];
    y1 = current->yPrev[0];
    l1 = n1->l;


    for(auto nodeid : n1->neighbor)
    {
        n2 = rtree_t.get_node(nodeid);
        x2 = n1->intersection[nodeid].first;
        y2 = n1->intersection[nodeid].second;
        l2 = n2->l;
        
        iter = n2;

        if(n2->CS + n2->PS <= n1->CS + n2->PS)
            continue;



        rDir = routing_direction(current->xPrev[0], current->yPrev[0], 
        
        valid = true;
        for(i=0; i < numBits; i++)
        {
            
            
            while(true)
            {
                // iterating node valid check
                if(iter == nullptr)
                {
                    valid = false;
                    break;
                }

                // connection valid
                i1 = iter->id;
                i2 = current->nodes[i];
                edge e(min(i1, i2), max(i1, i2));
                if(!rtree_t.is_valid(e))
                {
                    valid = false;
                    break;
                }

                // width constraint check
                if(iter->width < curBus->width[n2->l])
                    continue;


                // No spacing violation requirement with previous track
                if(i==0)
                {
                    prevOffset = iter->offset;
                }
                else
                {
                    currOffset = iter->offset;
                    if(abs(currOffset-prevOffset) < curBus->width[n2->l] + spacing[n2->l])
                        continue;

                    prevOffset = currOffset;
                }

                rn[i] = iter->id;
                iter = iter->upper;
            }
        }

        if(!valid)
            continue;


        // Create new heap node
        HeapNode newNode(nodes[n2->id]);
        newNode.backtrace = current->id;
        newNode.CR = current->CR;
        newNode.PS = current->PS;

        WL = 0;
        PS = 0;
        CC = 0;
        for(i=0; i < numBits; i++)
        {
            RtreeNode *prevNode, *currNode;
            prevNode = rtree_t.get_node(current->nodes[i]);
            currNode = rtree_t.get_node(rn[i]);
            l1 = current->l;
            x1 = current->xPrev[i];
            y1 = current->yPrev[i];
            x2 = prevNode->intersection[currNode->id].first;
            y2 = prevNode->intersection[currNode->id].second;
           
            // Update new node
            newNode.xPrev[i] = x2;
            newNode.yPrev[i] = y2;
            newNode.nodes[i] = currNode->id;
            
            PS += num_spacing_violation(curBus->bits[i], x1, y1, x2, y2, l1, curBus->width[l1], spacing[l1], is_vertical(l1));
            WL += manhatan_distance(x1, y1, x2, y2);
        }

        newNode.CR += ALPHA * WL;
        newNode.PS += DELTA * PS;

        // Compare the cost
        if(nodes[n2->id].CR + nodes[n2->id].PS > newNode.CR + newNode.PS)
        {
            next.push_back(newNode);
        }
    }
    
}

   
bool OABusRouter::Router::route_twopin_net_v8(int busid, int m1, int m2, vector<Segment> &tp)
{
    if(should_stop())
        return false;
    
    typedef tuple<int,int,int,int> ituple;

    // Global variables
    int i, wireid, bitid, trackid, l, seq, initSdir;
    int numwires, numpins, numbits, count, index, align1, align2;
    int minElem, minCost, totalSPV, numDestSPV, maxDepth, minPanelty;
    int llx, lly, urx, ury;
    int local_area_ll[2], local_area_ur[2];
    int mx1[2], mx2[2], my1[2], my2[2];
    int pin1x[2], pin1y[2], pin2x[2], pin2y[2];
    bool isRef, reverse, solution, vertical_arrange1, vertical_arrange2;

    vector<int> sorted1, sorted2;
    dense_hash_map<int,int> width;
    dense_hash_map<int,int> sequence;
    sequence.set_empty_key(INT_MAX);

    Bus* curbus;
    Pin *pin1, *pin2;
    MultiPin *mp1, *mp2;

    mp1 = &ckt->multipins[m1];
    mp2 = &ckt->multipins[m2];
   
    curbus = &ckt->buses[busid];
    numbits = curbus->numBits;
    width = curbus->width;



    // Priority Queue
    auto cmp = [](const ituple &left, const ituple &right){
        return (get<1>(left) + get<2>(left) + get<3>(left) > get<1>(right) + get<2>(right) + get<3>(right));
    };
    priority_queue<ituple , vector<ituple>, decltype(cmp)> PQ(cmp);

    // First Candidate
    queries.clear();
    local_rtree_t.query(ek;
    ueryMode::Intersects, ext1, pin1->l-1, pin1->l+1, queries);

    #pragma omp parallel for num_threads(NUM_THREADS)
    for(int j=0; j < queries.size(); j++)
    {
        // local variables
        int e1, t1, l1, x1, y1, x2, y2, c1, c2, c3;
        int maxWidth, numSpacingVio;
        int wirex[2], wirey[2], xs[2], ys[2];
        bool isDestination, vertical1;
        seg elem1;


        #pragma omp critical(GLOBAL)
        {

        }
    }

    while(PQ.size() > 0)
    {
        int e1, t1, l1, x1, y1, dep1, cost1, cost2, cost3;
        int maxWidth1;
        bool vertical1;
        seg elem1;

        queries.clear();
        local_rtree_t.query(QueryMode::Intersects, elem1, l1-1, l1+1, queries);

        #pragma omp parallel for num_threads(NUM_THREADS)
        for(int j=0; j < queries.size(); j++)
        {
            int e2, t2, l2, x2, y2, x3, y3, dep2, c1, c2, c3;
            int maxWidth2, curDir, numSpacingVio;
            int sx1, sx2, sy1, sy2, coefWL;
            int wirex[2], wirey[2], xs[2], ys[2];
            bool vertical2, isDestination;
            seg elem2;
        }


        if(hasMinElem)
        {
            int e1, e2, x1, x2, y1, y2, x3, y3, w1, w2;
            int t1, t2, l1, l2, count, index, curDir;
            bool vertical1, vertical2, pin;
            int xs[2], ys[2], wirex[2], wirey[2];

        }
        else
        {

        }
    }

    return solution;
}
*/
/*
void OABusRouter::Router::get_drive_nodes(int refBit, int mp, vector<HeapNode> &nodes)
{

    
    int i, l, x1, y1, x2, y2, align, numBits;
    int xPin[2], yPin[2], xSeg[2], ySeg[2];
    dense_hash_map<int, int> width;

    MultiPin* curMultipin = &ckt->multipins[mp];
    l = curMultipin->l;
    align = curMultipin->align;
    numBits = curMultipin->numBits;
    width = ckt->buses[curMultipin->busid].width;

    vector<int> tarLayers;
    if(!curMultipin->needVia)
    {
        tarLayers.push_back(l);

    }
    else
    {
        if(l > 0)
            tarLayers.push_back(l-1);
        if(l < ckt->layers.size()-1)
            tarLayers.push_back(l+1);
    }



    for(auto curl : tarLayers)
    {
        bool findAll = true;
        HeapNode node(numBits);

        for(i=0; i < curMultipin->pins.size(); i++)
        {
            vector<pair<seg,int>> queries;
            Pin* curPin = &ckt->pins[curMultipin->pins[i]];
            if(!curMultipin->needVia)
            {
                xPin[0] = (align == VERTICAL) ? curPxPin[0] : xPin[0] - width[l]/2;
                xPin[1] = (align == VERTICAL) ? xPin[1] : xPin[1] + width[l]/2;
                yPin[0] = (align == VERTICAL) ? yPin[0] - width[l]/2 : yPin[0];
                yPin[1] = (align == VERTICAL) ? yPin[1] + width[l]/2 : yPin[1];
            }
            
            box area(pt(xPin[0],yPin[0]), pt(xPin[1], yPin[1]));

            rtree_t[curl]->tree.query(bgi::intersects(area), back_inserter(queries));
            if(queries.size() == 0)
            {
                findAll = false;
                break;
            }
            else
            {
                if(queries.size() != 1)
                {
                    sort(queries.begin(), queries.end(), [&, xPin, yPin](const pair<seg,int> &left, const pair<seg,int> &right){
                            int xCenter = (xPin[0] + xPin[1])/2;
                            int yCenter = (yPin[0] + yPin[1])/2;
                            return bg::distance(left.first, pt(xCenter, yCenter)) < bg::distance(right.first, pt(xCenter, yCenter));
                            });
                }

                if(pin2bit[curpin->id] == refBit)
                {
                    node.id = queries[0].second;
                }

                node.nodes[i] = queries[0].second;
            }
        }

        if(findAll)
        {
            nodes.push_back(node);
        }
    }
}
*/


