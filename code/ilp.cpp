#include <ilcplex/ilocplex.h>
#include <google/dense_hash_set>
#include "ilp.h"
#include "circuit.h"
#include "route.h"

//#define DEBUG_CLIP

using google::dense_hash_map;
using google::dense_hash_set;

// static parameters
static int ILP_ITER_COUNT = 0;
static int TiLimit = 600;
static int NumThread = 1;
static double EpGap = 0.05;
static double EpAGap = 0.05;

// clip container
vector<OABusRouter::Clip> clips;


// ILP formulation
void OABusRouter::Router::CreateClips()
{
    int numClips, numLayers, numCols, numRows;
    int numTrees, numSegs, numCandi;
    int numVerSegs, numHorSegs;
    int numVerLs, numHorLs;
    int clipid, busid, treeid, segid, deg;
    int i, j, k;
    int indexdivider, indexl;
    int *vars, *vals;   
    bool isVertical;
    vector<int> verls;  // vertical layer id
    vector<int> horls;  // horizontal layer id
    //vector<Clip> clips;    
    numTrees = rsmt.trees.size(); //numTrees;
    numClips = numTrees;
    numLayers = grid.numLayers;
    numCols = grid.numCols;
    numRows = grid.numRows;

    StTree* curtree;
    // 
    for(i=0; i < numLayers; i++)
    {
        isVertical = (ckt->layers[i].direction == VERTICAL)?true:false;
        if(isVertical)
            verls.push_back(ckt->layers[i].id);
        else
            horls.push_back(ckt->layers[i].id);
    }

    numHorLs = horls.size();
    numVerLs = verls.size();

    // Create clip for each bus/tree
    for(i=0; i < numTrees; i++)
    {
        curtree = rsmt[i];
        clipid = clips.size();
        treeid = curtree->id;
        busid = rsmt.GetBusID(treeid);
        numSegs = curtree->segs.size();
        numVerSegs = 0;
        numHorSegs = 0;
        deg = numSegs;

        vars = new int[numSegs];
        for(j=0; j < numSegs; j++)
        {
            segid = curtree->segs[j];
            if(segs[segid].vertical) 
                numVerSegs++;
            else
                numHorSegs++;

            vars[j] = segid;
        }
        numCandi = pow(numHorLs, numHorSegs) * pow(numVerLs, numVerSegs);
        vals = new int[numSegs * numCandi];
        indexdivider = 1;
        for(j=0; j < numSegs; j++)
        {
            isVertical = segs[curtree->segs[j]].vertical;
            //indexdivider = pow(deg,j);
            for(k=0; k < numCandi; k++)
            {
                indexl = (isVertical)? (int)(1.0*k / indexdivider) % numVerLs : (int)(1.0*k / indexdivider) % numHorLs;           
                vals[j + k*deg] = (isVertical)? verls[indexl] : horls[indexl];
            }
            indexdivider *= (isVertical)? numVerLs : numHorLs;   
        }

        Clip curClip;
        curClip.id = clipid;
        curClip.busid = busid;
        curClip.treeid = treeid;
        curClip.numSegs = numSegs;
        curClip.numCandi = numCandi;
        curClip.segs = curtree->segs;
        curClip.juncs = curtree->junctions; 


        for(j=0; j < numCandi; j++)
        {
            Candidate candi(deg, vars, (vals + j*deg));
            curClip.candi.push_back(candi);
        }

        clips.push_back(curClip);
        delete vals , vars;
    }

#ifdef DEBUG_CLIP
    numClips = clips.size();


    for(i=0; i < numClips; i++)
    {
        Clip &curClip = clips[i];
        busid = curClip.busid;
        treeid = curClip.treeid;
        numSegs = curClip.numSegs;
        numCandi = curClip.numCandi;
        printf("Current clip bus%d tree%d #seg %d #candi %d\n\n", busid, treeid, numSegs, numCandi);

        printf("vertical layers     = {");
        for(j=0; j < numVerLs; j++)
        {
            printf(" m%d", verls[j]);
        }
        printf(" }\n");

        printf("horizontal layers   = {");
        for(j=0; j < numHorLs; j++)
        {
            printf(" m%d", horls[j]);
        }
        printf(" }\n");

        printf("segment list        = {");
        for(j=0; j < numSegs; j++)
        {
            printf(" s%d", curClip.segs[j]);
        }
        printf(" }\n\n");

        for(j=0; j < numCandi; j++)
        {

            deg = curClip[j].deg; 
            printf("candidate %3d -> {",j);
            for(int k=0; k < deg; k++)
            {
                int segid = curClip.segs[k];
                printf(" s%d:m%d", segid, curClip[j][segid]);
            }
            printf(" }\n");
        }
        printf("\n\n");
    }
    printf("\n\n");

#endif

}


void OABusRouter::Router::SolveILP_v2()
{


    try{
        
        // Cplex environment setting
        IloEnv env;
        stringstream logFile;
        IloModel model(env);
        IloNumVarArray cplexVars(env);
        IloExprArray cplexExprs(env);
        IloRangeArray cplexConsts(env);
        IloCplex cplex(model);
        IloExpr objective(env);

        cplex.setParam(IloCplex::TiLim, TiLimit);
        cplex.setParam(IloCplex::Threads, NumThread);
        cplex.setParam(IloCplex::EpGap, EpGap);
        cplex.setParam(IloCplex::EpAGap, EpAGap);
        cplex.setOut(logFile);

        // Variables
        int numLayers, numCols, numRows;
        int numClips, numSegs, numJuncs,  numCandi;
        int x1, y1, x2, y2, l, bw, cap, cindex;
        int x, y, i, j, k;
        int treeid, segid, clipid, busid, juncid, varid;
        int s1, s2, l1, l2, diff;
        int varSid, varCid;
        int MAXP;
        bool isVertical;
        double alpha, beta;
        int wirelength, numsegments;
        int VIA_COST = 3;
        // 
        alpha = 1.0;
        beta = 1.0;

        Junction* curJ;
        Segment* curS;
        //vector<IloNumVar> varCs;
        vector<IloNumVar> sVars;
        vector<IloExpr> cVars;
        //vector<IloExpr> const1; // Assignment Constraint
        //vector<IloExpr> const2; // Edge Capacitance Constraint
        
        dense_hash_map<int,int> lowerid;
        dense_hash_map<int,int> upperid;
        dense_hash_map<int,bool> isSegment;            // if varS is point, junction
        dense_hash_map<int,int> varS2junc;          // varS to junction
        dense_hash_map<int,int> varS2seg;           // segment variable to segment
        dense_hash_map<int,int> varC2bus;           // candidate variable to bus
        dense_hash_map<int,int> varC2clip;          // candidate variable to clip
        dense_hash_map<int,int> varC2candi;         // candidate index


        isSegment.set_empty_key(INT_MAX);
        varS2junc.set_empty_key(INT_MAX);
        varS2seg.set_empty_key(INT_MAX);
        varC2bus.set_empty_key(INT_MAX);
        varC2clip.set_empty_key(INT_MAX);
        varC2candi.set_empty_key(INT_MAX);

        lowerid.set_empty_key(INT_MAX);
        upperid.set_empty_key(INT_MAX);
    

        numLayers = grid.numLayers;
        numCols = grid.numCols;
        numRows = grid.numRows;
        numClips = clips.size();
        MAXP = max(numCols, numRows);


        IntervalMapT imap[numLayers][MAXP];


        // for every clips, declare variables for lp formula
        for(i=0; i < numClips; i++)
        {

            IloExpr assignConst(env);

            Clip& curClip = clips[i];
            clipid = curClip.id;
            busid = curClip.busid;
            numSegs = curClip.numSegs;
            numCandi = curClip.numCandi;
            
            numJuncs = curClip.juncs.size();


            for(j=0; j < numCandi; j++)
            {
                
                // Variable C
                IloExpr varC(env);
                varCid = cVars.size();
                
                wirelength = 0;
                numsegments = 0;

                IloExpr tmp(env);
            

                
                lowerid[varCid] = sVars.size();

                for(k=0; k < numSegs; k++)
                {
                    
                    // Variable S
                    IloNumVar varS(env, 0, 1, IloNumVar::Bool);

                    // add segment variable into the vector / map
                    varSid = sVars.size();
                    segid = curClip.segs[k];
                    sVars.push_back(varS);
                    varS2seg[varSid] = segid;
                    isSegment[varSid] = true;
                    
                    //
                    curS = &segs[segid];
                    x1 = curS->x1;
                    y1 = curS->y1;
                    x2 = curS->x2;
                    y2 = curS->y2;
                    isVertical = curS->vertical;

                    l = curClip[j][segid];
                    // cost
                    wirelength = abs(x2 - x1) + abs(y2 - y1);
                    numsegments += 1;

                    // imap for conflict 
                    if(isVertical)
                    {
                        imap[l][x1] +=
                            make_pair(IntervalT::closed(min(y1,y2), max(y1,y2)), set<int>({varSid}));
                    }
                    else
                    {
                        imap[l][y1] +=
                            make_pair(IntervalT::closed(min(x1,x2), max(x1,x2)), set<int>({varSid}));
                    }

                    // add variable into the expression
                    tmp += varS;   
                }

                upperid[varCid] = sVars.size();

                // Junction
                for(k=0; k < numJuncs; k++)
                {
                    juncid = curClip.juncs[k];
                    curJ = &junctions[juncid];
                    
                    s1 = curJ->s1;
                    s2 = curJ->s2;
                    l1 = curClip[j][s1];
                    l2 = curClip[j][s2];
                    x = curJ->x;
                    y = curJ->y;
                    diff = abs(l2 - l1);
                    if(diff < 2) continue;

                    // Variable S
                    IloNumVar varS(env, 0, 1, IloNumVar::Bool);
                    varSid = sVars.size();
                    varS2junc[varSid] = juncid;
                    isSegment[varSid] = false;
                    sVars.push_back(varS);
                    // cost
                    wirelength += VIA_COST;
                    numsegments += 1;

                    if(l1 > l2)
                    {
                        swap(l1,l2);
                        swap(s1,s2);
                    }   

                    for(l= l1+1; l < l2; l++)
                    {

                        isVertical = (grid.direction[l] == VERTICAL)? true : false;  
                        if(isVertical)
                        {
                            imap[l][x] +=
                                make_pair(IntervalT::closed(y,y), set<int>({varSid}));
                        }
                        else
                        {
                            imap[l][y] +=
                                make_pair(IntervalT::closed(x,x), set<int>({varSid}));
                        }

                    }


                    // add variable into the expression
                    tmp += varS;   
                }


                // Conditianal variable C
                varC = (tmp == numsegments); // ? 1 : 0;
                //

                assignConst += varC;

                // add candidate variable into the vector / map
                cVars.push_back(varC);
                varC2clip[varCid] = clipid;
                varC2bus[varCid] = busid;
                varC2candi[varCid] = j;
                // updated objective function
                objective +=
                    1.0 / (alpha * wirelength + beta * numsegments) * varC;

            }

            // Assignment Constraint
            cplexConsts.add( assignConst <= 1 );
        }

        int start, end, col, row, numiter;
        IntervalMapT* curMap;
        IntervalMapT::iterator it;
        DiscreteIntervalT curi;


        for(i=0; i < numLayers; i++)
        {
            l = i;
            isVertical = (grid.direction[i] == VERTICAL)? true : false;
            numiter = (isVertical)? numCols : numRows;

            for(j=0; j < numiter; j++)
            {
                curMap = &imap[i][j];

                it = curMap->begin();
                while(it != curMap->end())
                {
                    curi = (*it).first;
                    set<int> &curset = (*it).second;

                    start = curi.lower();
                    end = curi.upper();
                    IloExpr tmp(env);

                    for(auto& id : curset)
                    {
                        if(isSegment[id])
                        {
                            segid = varS2seg[id];
                            busid = seg2bus[segid];
                            bw = segs[segid].bw;
                            //bitwidth[busid];
                        }
                        else
                        {
                            juncid = varS2junc[id];
                            bw = junctions[juncid].bw;
                        }
                        
                        tmp += bw*sVars[id];
                    }

                    while(start <= end)
                    {
                        col = (isVertical) ? j : start;
                        row = (isVertical) ? start : j;
                        cap = grid[grid.GetIndex(col,row,l)]->cap;
                        cplexConsts.add( tmp <= cap );
                        start++;
                    }
                    it++;
                }
            }
        }


        model.add(IloMaximize(env, objective));
        model.add(cplexConsts);
        cplex.exportModel("./lp/form.lp");

        cplex.solve();
        int numSolution=0;
        
        if(cplex.getStatus() == IloAlgorithm::Optimal)
        {
            printf("Cplex successfully solved!\n\n[Results]\n", ILP_ITER_COUNT++);
            for(i=0; i < cVars.size(); i++)
            {
                int valC = cplex.getValue(cVars[i]);

                if(valC == 1)
                {
                
                    numSolution++;

                    clipid = varC2clip[i];
                    cindex = varC2candi[i]; 
                    Clip& curClip = clips[varC2clip[i]];
                    busid = curClip.busid;
                    treeid = curClip.treeid;
                    numSegs = curClip.numSegs;
                    numJuncs = curClip.juncs.size(); 
                    
                    rsmt[treeid]->assign = true;
                    
                    printf("[Done] %s -> {", ckt->buses[busid].name.c_str());
                    for(j=0; j < numSegs; j++)
                    {   
                        segid = curClip.segs[j];
                        l = curClip[cindex][segid];

                        curS = &segs[segid];
                        curS->l = l;
                        curS->assign = true;
                        //assign[segid] = true;
                                
                        for(col=curS->x1; col <= curS->x2; col++)
                        {
                            for(row = curS->y1; row <= curS->y2; row++)
                            {
                                grid[grid.GetIndex(col,row,l)]->cap -= curS->bw;
                            }
                        }
                        printf(" s%d:m%d", segid, l);
                    }
                    printf(" }\n");
                    for(j=0; j < numJuncs; j++)
                    {
                        juncid = curClip.juncs[j];
                        curJ = &junctions[juncid];
                        s1 = curJ->s1;
                        s2 = curJ->s2;
                        curJ->l1 = segs[s1].l;
                        curJ->l2 = segs[s2].l;
                        if(curJ->l1 > curJ->l2)
                        {
                            swap(curJ->l1, curJ->l2);
                            swap(curJ->s1, curJ->s2);
                        }



                        l1 = curJ->l1+1;
                        l2 = curJ->l2;
                        col = curJ->x;
                        row = curJ->y;
                        while(l1 < l2)
                        {
                            grid[grid.GetIndex(col,row,l1)]->cap -= curJ->bw;
                            l1++;
                        }
                        
                        


                    }
                }
                else
                {
                    int varS;
                    int lower, upper;
                    lower = lowerid[i];
                    upper = upperid[i];

                    Clip& curClip = clips[varC2clip[i]];
                    busid = curClip.busid;
                    treeid = curClip.treeid;
                    //rsmt[treeid]->assign = false;

                    printf("[Fail] %s -> {", ckt->buses[busid].name.c_str());

                    while(lower < upper)
                    {
                        varS = cplex.getValue(sVars[lower]);
                        segid = varS2seg[lower];            
                        curS = &segs[segid];

                        printf(" s%d:v%d", segid, varS);
                        lower++;
                    }

                    printf(" }\n");

                }
    
            }
            
            printf("\n\n");
            printf("===================\n\n");
            printf("#Solution   : %5d\n", numSolution);
            printf("Routability : %3.2f\n\n", (float)(1.0*numSolution / numClips));
            printf("===================\n\n\n");

        }

    
    }catch(IloException& ex){
        cerr << "Error: " << ex << endl;
    }catch(...){
        cerr << "Error" << endl;
    }
}



void OABusRouter::Router::SolveILP()
{




    try{
        IloEnv env;
        stringstream logFile;
        IloModel model(env);
        IloNumVarArray cplexVars(env);
        IloExprArray cplexExprs(env);
        IloRangeArray cplexConsts(env);
        IloCplex cplex(model);
        IloExpr objective(env);

        cplex.setParam(IloCplex::TiLim, TiLimit);
        cplex.setParam(IloCplex::Threads, NumThread);
        cplex.setParam(IloCplex::EpGap, EpGap);
        cplex.setParam(IloCplex::EpAGap, EpAGap);
        cplex.setOut(logFile);


        int numLayers, numCols, numRows, numTrees, numSegs, numVars;
        int MAXP, varID, exprID, segID, busID, minval, maxval;
        int x1, y1, x2, y2, l, bw, cap, lidx, pidx;
        bool isVertical;
        StTree* tree;
        Segment* curS;
        numLayers = this->grid.numLayers;
        numCols = this->grid.numCols;
        numRows = this->grid.numRows;
        numTrees = this->rsmt.trees.size();

        //
        MAXP = max(numCols+1, numRows+1);
        IntervalMapT iMaps[numLayers + 1][MAXP];


        vector<IloNumVar> varXs;
        vector<IloExpr> exprCs;
        vector<IloExpr> exprG1s;
        vector<IloExpr> exprG2s;
        
        
        dense_hash_map<int,int> expr2seg;
        dense_hash_map<int,int> var2seg;
       
        

        var2seg.set_empty_key(INT_MAX);
        expr2seg.set_empty_key(INT_MAX);


        
        //
        numSegs = this->segs.size();
        for(int i=0; i < numSegs; i++)
        {
            
            
            IloExpr exprG1(env);
            IloExpr exprG2(env);
           
            //exprG1 = 0;
            //exprG2 += 1;
            
            exprID = exprG1s.size();
            
            curS = &this->segs[i];
            segID = curS->id;
            x1 = curS->x1;
            y1 = curS->y1;
            x2 = curS->x2;
            y2 = curS->y2;
            l = curS->l;


            isVertical = (this->grid.direction[l] == VERTICAL)? true:false;
            
            if(isVertical)
            {

                minval = min(y1,y2);
                maxval = max(y1,y2);
                numVars = maxval - minval;
                for(int k=minval ; k <= maxval; k++)
                {
                    IloNumVar varX(env, 0, 1, IloNumVar::Bool);
                    varID = varXs.size();
                    varXs.push_back(varX);
                    
                    exprG1 += varX;
                    //exprG2 = exprG2*varX;
                    var2seg[varID] = curS->id;
                }
                iMaps[l][x1] +=
                    make_pair(IntervalT::closed(min(y1,y2), max(y1,y2)), IDSetT({exprID}));
            }
            else
            {
                minval = min(x1,x2);
                maxval = max(x1,x2);
                numVars = maxval - minval;
                for(int k=minval; k <= maxval; k++)
                {
                    IloNumVar varX(env, 0, 1, IloNumVar::Bool);
                    varID = varXs.size();
                    varXs.push_back(varX);

                    exprG1 += varX;
                    //exprG2 = exprG2*varX;
                    var2seg[varID] = curS->id;
                }

                iMaps[l][y1] +=
                    make_pair(IntervalT::closed(min(x1,x2), max(x1,x2)), IDSetT({exprID}));

            }
            
       

            exprG1 -= numVars;
            objective += exprG1;
            exprG2 = (exprG1 == 0);
            
            cplexConsts.add(exprG1 >= -1);
       

            exprG1s.push_back(exprG1);
            exprG2s.push_back(exprG2);
            expr2seg[exprID] = segID;
        }


        

        int start, end, col, row, layer;
        int MAXITER = numLayers * MAXP;
        IntervalMapT* curMap;

        for(int c=0; c < MAXITER; c++)
        {
            lidx = (int)(1.0 * c / MAXP + 0.5);
            pidx = c % MAXP;
            isVertical = (this->grid.direction[lidx] == VERTICAL)? true:false;


            //printf("max (%d %d) / l, p (%d %d)\n", numLayers, MAXP, lidx, pidx);

            curMap = &iMaps[lidx][pidx];
            IntervalMapT::iterator it =  curMap->begin();
            // Conflict
            while(it != curMap->end())
            {
                DiscreteIntervalT curInterval = (*it).first;
                IDSetT &curSet = (*it).second;
                start   = curInterval.lower();
                end     = curInterval.upper();

                IloExpr tmpExpr(env);

                for(auto& eid : curSet)
                {
                    segID = expr2seg[eid];
                    busID = this->seg2bus[segID];
                    bw = segs[segID].bw;

                    //IloExpr tmpExpr2(env);
                    //tmpExpr2 = (exprG1s[eid]==0);
                    tmpExpr += exprG2s[eid]*bw;
                }

                while(start <= end)
                {
                    if(isVertical)
                    {
                        col = pidx;
                        row = start;
                        layer = lidx;
                    }
                    else
                    {
                        col = start;
                        row = pidx;
                        layer = lidx;
                    }
                    //IloExpr exprC(env);
                    //exprID = exprCs.size();

                    cap =this->grid.Capacity(col, row, layer);  
                    cplexConsts.add( tmpExpr <= cap );   

                    start++;
                }
                it++;
            }
        }


        model.add(IloMaximize(env, objective));
        model.add(cplexConsts);


        cplex.exportModel("./ilp/formulation.lp");
        if(cplex.solve())
        {
            printf("ILP(%3d) solved!\n", ILP_ITER_COUNT++);

             
        
        
        
        
        
        
        
        
        
        
        }else{
            printf("????\n");
        }

    }catch(IloException& ex){
        cerr << "Error: " << ex << endl;
    }catch(...){
        cerr << "Error" << endl;
    }
}






