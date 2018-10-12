
struct HeapNode
{
    int id;
    int depth;
    int backtrace;
    double CR;
    double PS;
    vector<int> nodes;
    vector<int> xPrev;
    vector<int> yPrev;

    HeapNode()
    {}

    HeapNode(int nodeid, int numBits) :
        depth(0),
        backtrace(-1),
        CR(0),
        PS(0)
    {
        id = nodeid;
        nodes = vector<int>(numBits, -1);
        xPrev = vector<int>(numBits, -1);
        yPrev = vector<int>(numBits, -1);
    }

    HeapNode(const HeapNode& node):
        id(node.id),
        depth(node.depth),
        backtrace(node.backtrace),
        CR(node.CR),
        PS(node.PS),
        nodes(node.nodes),
        xPrev(node.xPrev),
        yPrev(node.yPrev)
    {}

};


struct Heap
{
    static bool cmp = [](const HeapNode& n1, const HeapNode& n2)
    {
        return (n1.CR + n1.PS) > (n2.CR + n2.PS);
    };

    priority_queue<HeapNode, vector<HeapNode>, decltype(cmp)> PQ(cmp);

    // member funcs
    void push(HeapNode &node)
    {
        PQ.push(node);
    }

    void pop()
    {
        PQ.pop();
    }

    HeapNode top()
    {
        return PQ.top();
    }
};




void OABusRouter::Router::get_drive_segment(int mp, vector<HeapNode> &nodes, vector<HeapNode> &next)
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
        xPin[0] = (align == VERTICAL) ? xPin[0] : xPin[0] - width[l]/2;
        xPin[1] = (align == VERTICAL) ? xPin[1] : xPin[1] + width[l]/2;
        yPin[0] = (align == VERTICAL) ? yPin[0] - width[l]/2 : yPin[0];
        yPin[1] = (align == VERTICAL) ? yPin[1] + width[l]/2 : yPin[1];
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
        HeapNode node;

        for(i=0; i < curMultipin->pins.size(); i++)
        {
            vector<pair<seg,int>> queries;
            Pin* curpin = &ckt->pins[curMultipin->pins[i]];
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

                

                if(i=0)
                {
                    node = nodes[queries[0].second];
                    node.depth = 0;
                    node.backtrace = queries[0].second;
                }

                int x, y;
                pts(queries.first, xSeg[0], ySeg[0], xSeg[1], ySeg[1]);
                pin_access_point(xPin, yPin, l, xSeg, ySeg, curl, x, y);

                node.nodes[i] = queries[0].second;
                node.xPrev[i] = x;
                node.yPrev[i] = y;
            }
        }

        if(findAll)
        {
            nodes[node.id] = node;
            next.push_back(node);
        }
    }
}



void OABusRouter::Router::get_iterating_segment(int busid, HeapNode* current, vector<HeapNode> &nodes, vector<HeapNode> &next)
{
    Bus* curBus = &ckt->buses[busid];
    int i, i1, i2, rDir, numBits;
    int x1, y1, x2, y2, l1, l2;
    int prevOffset, currOffset;
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
                    prevOffset = iter->offset;
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



    }
    
    
    int xSeg[2] = {curNode->x1, curNode->y1};
    int ySeg[2] = {curNode->x2, curNode->y2};
    int lSeg = curNode->l;

    seg curSeg(pt(curNode->



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




/*
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
    local_rtree_t.query(QueryMode::Intersects, ext1, pin1->l-1, pin1->l+1, queries);

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

void OABusRouter::Router::get_candidates(int e1, vector<ElemInfo> &info)
{
    ElemInfo* info1 = &info[e1];
    int e2;
    
    int dep1, dep2;
    int x1, y1, x2, y2;
    int l1, l2;
    double CR1, PS1;
    CR1 = info1->CR;
    PS1 = info1->PS;
    
}

void OABusRouter::Router::get_initial_candidates(int mp, vector<ElemInfo> &info)
{

