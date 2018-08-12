
#include "circuit.h"
#include "route.h"
#include <stdlib.h>
#include <time.h>
#include <unordered_map>

#define DEBUG_MAZE


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
                    cost = e.cost + 3;
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








