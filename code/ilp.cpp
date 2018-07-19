#include <ilcplex/ilocplex.h>
#include <google/dense_hash_set>
#include <google/dense_hash_map>
#include "circuit.h"
#include "route.h"



using google::dense_hash_map;
using google::dense_hash_set;

// static parameters
static int ILP_ITER_COUNT = 0;
static int TiLimit = 600;
static int NumThread = 1;
static double EpGap = 0.05;
static double EpAGap = 0.05;


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
        vector<IloExpr> exprGs;
        
        
        dense_hash_map<int,int> expr2seg;
        dense_hash_map<int,int> var2seg;
       
        

        var2seg.set_empty_key(0);
        expr2seg.set_empty_key(0);


        
        //
        numSegs = this->segs.size();
        for(int i=0; i < numSegs; i++)
        {
            
            
            IloExpr exprG(env);
            exprID = exprGs.size();
            
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
                    
                    exprG += varX;
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

                    exprG += varX;
                    var2seg[varID] = curS->id;
                }

                iMaps[l][y1] +=
                    make_pair(IntervalT::closed(min(x1,x2), max(x1,x2)), IDSetT({exprID}));

            }
            
       

            exprG -= numVars;
            objective += exprG;
            cplexConsts.add(exprG >= -1);
       

            exprGs.push_back(exprG);
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
                    tmpExpr += exprGs[eid]*bw;
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






