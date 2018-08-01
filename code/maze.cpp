
#include "circuit.h"
#include "route.h"
#include <stdlib.h>
#include <time.h>
#include <unordered_map>


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



bool operator > (const Elem& left, const Elem& right){
    return (left.cost < right.cost) || ((left.cost == right.cost) && (left.count < right.count));
}

int random(int moduler)
{
    srand(time(NULL));
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
        //if(!curtree->assign)
        //{
            ObstacleAwareRouting(treeid);
        //}

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
    Gcell *curGC, *tarGC, *gc_iter, *gc_closest;
    
    vector<int> setN;   // stored gcell id which must be connected
    vector<int> setG;   // stored gcell id which the topology has
    vector<int> combG_PATH;
    Elem e;

    curtree = &rsmt.trees[treeid];
    deg = curtree->deg;
    //
    bw = ckt->buses[rsmt.busID[treeid]].numBits; //4;   


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
                }

                if(k==1)
                {
                    col2 = col1;
                    row2 = row1;
                    l2 = l1 + move[j];
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
                    cost =  abs(tarGC->x - gc_closest->x) + abs(tarGC->y - gc_closest->y) + abs(tarGC->l - gc_closest->l);
                    
                    combG_PATH.clear();
                    combG_PATH = setG;
                    // SetG U PATH(g_current)
                    for(g_iter = g2; g_iter != backtrace[g_iter]; g_iter = backtrace[g_iter])
                    {
                        combG_PATH.push_back(g_iter);
                    }
                
                    // Summation of manhatan distance between all target node and SetG U PATH
                    for(j=0; j<setNsize; j++)
                    {
                        g_iter = setN[j];
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
                        cost += abs(gc_iter->x - gc_closest->x) + abs(gc_iter->y - gc_closest->y) + abs(gc_iter->l - gc_closest->l);
                    }
               
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
    }
    else
    {
        //printf("No solution tree%d\n", treeid);

    }
}


//void OABusRouter::Router::MazeRouting(int p1, int p2)
//{



//}
