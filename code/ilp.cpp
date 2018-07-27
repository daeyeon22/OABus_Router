#include <ilcplex/ilocplex.h>
#include <google/dense_hash_set>
#include "ilp.h"
#include "circuit.h"
#include "route.h"

#define DEBUG_CLIP

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
        int numLayers, numCols, numRows, numClips, numSegs, numCandi;
        int x1, y1, x2, y2, l, bw, cap;
        int i, j, k, segid, clipid, busid, varid;
        int varSid, varCid;
        int MAXP;
        bool isVertical;
        double alpha, beta;
        int wirelength, numsegments;

        // 
        alpha = 1.0;
        beta = 1.0;


        Segment* curS;
        //vector<IloNumVar> varCs;
        vector<IloNumVar> sVars;
        vector<IloExpr> cVars;
        //vector<IloExpr> const1; // Assignment Constraint
        //vector<IloExpr> const2; // Edge Capacitance Constraint
        dense_hash_map<int,int> varS2seg;           // segment variable to segment
        dense_hash_map<int,int> varC2bus;           // candidate variable to bus
        dense_hash_map<int,int> varC2clip;          // candidate variable to clip


        varS2seg.set_empty_key(0);
        varC2bus.set_empty_key(0);
        varC2clip.set_empty_key(0);

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

            for(j=0; j < numCandi; j++)
            {
                
                // Variable C
                IloExpr varC(env);
                varCid = cVars.size();
                wirelength = 0;
                numsegments = 0;

                IloExpr tmp(env);
                for(k=0; k < numSegs; k++)
                {
                    
                    // Variable S
                    IloNumVar varS(env, 0, 1, IloNumVar::Bool);

                    varSid = sVars.size();
                    segid = curClip.segs[k];
                    curS = &segs[segid];
                    x1 = curS->x1;
                    y1 = curS->y1;
                    x2 = curS->x2;
                    y2 = curS->y2;
                    isVertical = curS->vertical;

                    l = curClip[k][segid];
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
                            make_pair(IntervalT::closed(min(x1,x2), max(x1,x2)), set<int>({varCid}));
                    }


                    // add segment variable into the vector / map
                    sVars.push_back(varS);
                    varS2seg[varSid] = segid;

                    tmp += varS;   
                }

                // Conditianal variable C
                varC = (tmp == numsegments); // ? 1 : 0;
                assignConst += varC;

                // add candidate variable into the vector / map
                varC2clip[varCid] = clipid;
                varC2bus[varCid] = busid;

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

                    for(auto& sid : curset)
                    {
                        segid = varS2seg[sid];
                        busid = seg2bus[segid];
                        bw = bitwidth[busid];

                        tmp += bw*sVars[sid];
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

        if(cplex.solve())
        {
            printf("Cplex successfully solved!\n", ILP_ITER_COUNT++);

        }else{

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
       
        

        var2seg.set_empty_key(0);
        expr2seg.set_empty_key(0);


        
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
                    bw = this->bitwidth[busID];

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






