#include "typedef.h"
#include "route.h"
#include "circuit.h"
#include "func.h"
#include "heap.h"
#include <algorithm>

using OABusRouter::is_vertical;


OABusRouter::Tile* OABusRouter::Grid3D::get_tile(int col, int row, int l)
{
    return &tiles[numCols*row + numCols*numRows*l + col];
}

int OABusRouter::Grid3D::get_index(int col, int row, int l)
{
    return numCols*row + numCols*numRows*l + col;
}

int OABusRouter::Grid3D::get_colum(int index)
{
    return index % numCols;
}

int OABusRouter::Grid3D::get_row(int index)
{
    return (int)floor(index/numCols) % numRows;
}

int OABusRouter::Grid3D::get_layer(int index)
{
    return (int)floor(index/(numCols*numRows));
}


bool OABusRouter::Router::get_search_area(int busid, int m1, int m2, int margin,  SearchArea& SA)
{

    int numCols, numRows, numLayers;
    int tarWidth, tarHeight, numBits;
    int originX = ckt->originX;
    int originY = ckt->originY;
    int dieWidth = ckt->width;
    int dieHeight = ckt->height;
    int edgeCap;
    Bus* curBus = &ckt->buses[busid];
    numLayers = ckt->layers.size();
    numBits = curBus->numBits;
    tarWidth = INT_MIN;
    tarHeight = INT_MIN;

    
    
    tarWidth = max(abs(multipin2llx[m1]-multipin2urx[m1]), abs(multipin2llx[m2]-multipin2urx[m2]));
    tarHeight = max(abs(multipin2lly[m1]-multipin2ury[m1]), abs(multipin2lly[m2]-multipin2ury[m2]));


    tarWidth = max(tarWidth, tarHeight);
    tarHeight = max(tarWidth, tarHeight);

    for(int i=0; i < numLayers; i++)
    {
        if(is_vertical(i))
            tarWidth = max(tarWidth, numBits * (curBus->width[i] + spacing[i]));
        else
            tarHeight = max(tarHeight, numBits * (curBus->width[i] + spacing[i]));
    }
    
    //tarWidth*=2;
    //tarHeight*=2;

    //tarWidth = max(tarWidth, tarHeight);
    //tarHeight = max(tarWidth, tarHeight);

    
    numCols = ceil(1.0* ckt->width / tarWidth);
    numRows = ceil(1.0* ckt->height / tarHeight);

    // Contruction grid3d
    Grid3D grid;
    grid.numCols = numCols;
    grid.numRows = numRows;
    grid.numLayers = numLayers;
    grid.tileWidth = tarWidth;
    grid.tileHeight = tarHeight;
    grid.tiles = vector<Tile>(numCols * numRows * numLayers);

    
    // MultiPin
    MultiPin* mp1 = &ckt->multipins[m1];
    MultiPin* mp2 = &ckt->multipins[m2];

    dense_hash_map<int,int> bit2pin_1;
    dense_hash_map<int,int> bit2pin_2;
    bit2pin_1.set_empty_key(INT_MAX);
    bit2pin_2.set_empty_key(INT_MAX);

    for(int i=0; i < numBits; i++)
    {
        bit2pin_1[pin2bit[mp1->pins[i]]] = mp1->pins[i];
        bit2pin_2[pin2bit[mp2->pins[i]]] = mp2->pins[i];
    }
    
    // - - editted 11/24 16:10 - - //
   
    // - - - - - - - - - - - - - - //
    
    Pin* tarPin1 = &ckt->pins[bit2pin_1[curBus->bits[0]]];
    Pin* tarPin2 = &ckt->pins[bit2pin_2[curBus->bits[0]]];
    box pinShape1(pt(tarPin1->llx, tarPin1->lly), pt(tarPin1->urx, tarPin1->ury));
    box pinShape2(pt(tarPin2->llx, tarPin2->lly), pt(tarPin2->urx, tarPin2->ury));
    box mpShape1(pt(mp1->llx, mp1->lly), pt(mp1->urx, mp1->ury));
    box mpShape2(pt(mp2->llx, mp2->lly), pt(mp2->urx, mp2->ury));

    // Start initialize
    Heap<HeapNode3> PQ;
    vector<HeapNode3> nodes(numCols*numRows*numLayers);
    
    #pragma omp parallel for num_threads(NUM_THREADS)
    for(int i=0; i < numCols*numRows*numLayers; i++)
    {
        int col = grid.get_colum(i);
        int row = grid.get_row(i);
        int l = grid.get_layer(i);

        Tile* curTile = grid.get_tile(col, row, l);
        int x1 = originX + tarWidth * col;
        int x2 = originX + min(tarWidth * (col+1), dieWidth);
        int y1 = originY + tarHeight * row;
        int y2 = originY + min(tarHeight * (row+1), dieHeight);

        nodes[i].id = i;
        curTile->id = i;
        curTile->x1 = x1;
        curTile->x2 = x2;
        curTile->y1 = y1;
        curTile->y2 = y2;
        curTile->row = row;
        curTile->col = col;
        curTile->l = l;


        vector<pair<seg, int>> q1;
        vector<pair<box, int>> q2;
        vector<pair<seg, int>> q3;

        box tileArea(pt(x1, y1), pt(x2, y2));

        rtree_t.trees[l].query(bgi::intersects(tileArea), back_inserter(q1));
        rtree_o.wires[l].query(bgi::intersects(tileArea), back_inserter(q2));
        rtree_o.empties[l].query(bgi::intersects(tileArea), back_inserter(q3));
       
        int tileSize = (is_vertical(l)) ? tarWidth : tarHeight;
        curTile->edgeCap = tileSize / (curBus->width[l] + spacing[l]) - q2.size() - q3.size();
        //curTile->edgeCap = tarWidth / (curBus->width[l] + spacing[l]) -q2.size() - q3.size();
        //curTile->edgeCap = min((int)q1.size(),  numBits) - q2.size() - q3.size();

        
        //for(int j=0; j < q3.size(); j++)
        //{
        //    int bitid = q3[j].second;
        //    if(bit2bus[bitid] == busid)
        //        curTile->edgeCap++;
        //    else
        //        curTile->edgeCap--;
        //}

        if(is_vertical(l))
        {
            if(row > 0)
                curTile->neighbor.push_back(grid.get_index(col, row-1, l));

            if(row+1 < numRows)
                curTile->neighbor.push_back(grid.get_index(col, row+1, l));
        }
        else
        {
            if(col > 0)
                curTile->neighbor.push_back(grid.get_index(col-1, row, l));

            if(col+1 < numCols)
                curTile->neighbor.push_back(grid.get_index(col+1, row, l));
        }

        if(l > 0)
            curTile->neighbor.push_back(grid.get_index(col, row, l-1));
        
        if(l+1 < numLayers)
            curTile->neighbor.push_back(grid.get_index(col, row, l+1));

    
    
        int lDiff1 = abs(mp1->l - l);
        if(bg::intersects(mpShape1, tileArea))
        {
            if(mp1->needVia)
            {
                if(lDiff1 == 1)
                {
                    curTile->edgeCap += numBits;
                }
            }
            else
            {
                if(lDiff1 == 0)
                {
                    curTile->edgeCap += numBits;
                }
            }
        }

        if(bg::intersects(pinShape1, tileArea))
        {
            if(mp1->needVia)
            {
                if(lDiff1 == 1)
                {
                    //curTile->edgeCap += numBits;
                    #pragma omp critical(QUEUE)
                    {
                        if(is_vertical(l))
                        {

                        }
                        else
                        {

                        }

                        nodes[i].depth = 0;
                        nodes[i].backtrace = i;
                        nodes[i].numSegs = 0;
                        PQ.push(nodes[i]);
                    }            
                }
            }
            else
            {
                if(lDiff1 == 0)
                {
                    //curTile->edgeCap += numBits;
                    #pragma omp critical(QUEUE)
                    {
                        if(is_vertical(l))
                        {

                        }
                        else
                        {

                        }

                        nodes[i].depth = 0;
                        nodes[i].backtrace = i;
                        nodes[i].numSegs = 0;
                        PQ.push(nodes[i]);
                    } 
                }
            }

        }

        int lDiff2 = abs(mp2->l - l);
        if(bg::intersects(mpShape2, tileArea))
        {
            if(mp2->needVia)
            {
                if(lDiff2 == 1)
                {
                    curTile->edgeCap += numBits;
                }
            }
            else
            {
                if(lDiff2 == 0)
                {
                    curTile->edgeCap += numBits;
                }
            }
        }
    }





    bool hasMinElem = false;
    int minCost = INT_MAX;
    int minElem = INT_MAX;

    //cout << "start local search" << endl;
    //printf("(1) Pin1 (%d %d) (%d %d) M%d\n", tarPin1->llx, tarPin1->lly, tarPin1->urx, tarPin1->ury, tarPin1->l);
    //printf("(2) Pin2 (%d %d) (%d %d) M%d\n", tarPin2->llx, tarPin2->lly, tarPin2->urx, tarPin2->ury, tarPin2->l);

    while(!PQ.empty())
    {
        HeapNode3 curNode = PQ.top();
        PQ.pop();

        int n1 = curNode.id;
        Tile* tile1 = &grid.tiles[n1];
        int dep1 = curNode.depth;
        int l1 = tile1->l;

        //printf("(%d %d) (%d %d) M%d Cap %d\n", tile1->x1, tile1->y1, tile1->x2, tile1->y2, tile1->l, tile1->edgeCap);

        if(n1 == minElem)
            break;
        
        if(curNode.cost() != nodes[n1].cost())
            continue;

        if(curNode.cost() > minCost)
            continue;


        #pragma omp parallel for num_threads(NUM_THREADS)
        for(int i=0; i < tile1->neighbor.size(); i++)
        {
            
            int n2 = tile1->neighbor[i];
            Tile* tile2 = &grid.tiles[n2];
            int dep2 = dep1 + 1;
            int l2 = tile2->l;
            
             if(tile2->edgeCap<= 0) // - numBits < 0) // numBits)
            {
                continue;
            }

           
            //printf("(%d %d) (%d %d) M%d Cap %d\n", tile2->x1, tile2->y1, tile2->x2, tile2->y2, tile2->l, tile2->edgeCap);
           


            ////////
            int tileSize = (is_vertical(l2)) ? tarWidth : tarHeight;
            int refCap = tileSize / (curBus->width[l2] + spacing[l2]);
            double capRatio = 1.0 * refCap / tile2->edgeCap; 
            double tileWeight = pow(capRatio, 5);

            ////////////////////////////////////////////
            /*
            double tileWeight = 1;
            if(capRatio < 0.9)
                tileWeight = 2;
            else if(capRatio < 0.7)
                tileWeight = 10;
            else if(capRatio < 0.5)
                tileWeight = 20;
            else if(capRatio < 0.3)
                tileWeight = 50;
            else
                tileWeight = 100;
            */

            //(tile2->edgeCap < numBits) ? 1.0 * numBits / tile2->edgeCap : 1.0; //(1.0* tile2->edgeCap / max(1, 
            
            //double tileWeight = 1.0 * numBits / tile2->edgeCap; //(tile2->edgeCap < numBits) ? 1.0 * numBits / tile2->edgeCap : 1.0; //(1.0* tile2->edgeCap / max(1, 
            HeapNode3 nextNode = nodes[n2];
            nextNode.backtrace = n1;
            nextNode.depth = dep2;
            nextNode.numSegs = (l1 != l2) ? curNode.numSegs + 1 : curNode.numSegs;
            nextNode.PS = tileWeight + curNode.PS;
            
            
            ///////////////////////////////////////////////////////////////////////////////
            /*
            if(tile2->edgeCap < numBits)
            {
               
                if(tile2->edgeCap < numBits/2)
                {
                    nextNode.PS += 10;
                }

                int adjCap = tile2->edgeCap;
                if(is_vertical(l2))
                {
                    if(tile2->col+1 < grid.numCols)
                    {
                        adjCap += grid.get_tile(tile2->col+1, tile2->row, tile2->l)->edgeCap;
                    }

                    if(tile2->col-1 >= 0)
                    {
                        adjCap += grid.get_tile(tile2->col-1, tile2->row, tile2->l)->edgeCap;
                    }
                }
                else
                {
                    if(tile2->row+1 < grid.numRows)
                    {
                        adjCap += grid.get_tile(tile2->col, tile2->row+1, tile2->l)->edgeCap;
                    }

                    if(tile2->row-1 >= 0)
                    {
                        adjCap += grid.get_tile(tile2->col, tile2->row-1, tile2->l)->edgeCap;
                    }
                }
                if(adjCap < numBits)
                    continue;
            }
            //if(tile2->edgeCap < numBits/2)
            //    nextNode.PS += 1000;
            
            if(tile2->edgeCap < refCap/2)
                nextNode.PS += 10;
            
            if(tile2->edgeCap < refCap/4)
                nextNode.PS += 10;
            
            if(tile2->edgeCap < refCap/6)
                nextNode.PS += 10;
            */
            
            //if(tile2->edgeCap <= 0)
            //    nextNode.PS += 1000;
            /*
            if(tile2->edgeCap < numBits/4)
            {
                nextNode.PS += 10;
                
            }

            if(tile2->edgeCap < numBits/6)
            {
                nextNode.PS += 10;
            }

            if(tile2->edgeCap < numBits/8)
            {
                nextNode.PS += 10;
            }
            */
            //nextNode.PS = (tile2->edgeCap - numBits/2 <= 0) ? curNode.PS + 5 : curNode.PS;//max(0, numBits - tile2->edgeCap) : curNode.PS; //abs(
            
            //nextNode.avgEdgeCap = 


            bool arrival = false;
            box tileArea(pt(tile2->x1, tile2->y1), pt(tile2->x2, tile2->y2));
            
            if(bg::intersects(pinShape2, tileArea))
            {
                int lDiff = abs(tile2->l - tarPin2->l);
                if(mp2->needVia && (lDiff == 1))
                {
                    arrival = true;
                }
                else if(!mp2->needVia && (lDiff == 0))
                {
                    arrival = true;
                }
            }


            #pragma omp critical(QUEUE)
            {
                if(nextNode.cost() < nodes[n2].cost())
                {
                    if(arrival && (minCost > nextNode.cost()))
                    {
                        hasMinElem = true;
                        minElem = n2;
                        minCost = nextNode.cost();
                    }

                    nodes[n2] = nextNode;
                    PQ.push(nodes[n2]);
                }
            }
        }
    }
   
        
    if(hasMinElem)
    {
        printf("\n< Local Search Area #Segs %d #Bits %d minCost %2.2f>\n", nodes[minElem].numSegs, numBits, nodes[minElem].cost());


        vector<pair<int,int>> tilePair;
        int n1, n2 = minElem, n3 = minElem;
        while(nodes[n2].backtrace != n2)
        {
            int n1 = nodes[n2].backtrace;
            Tile* tile1 = &grid.tiles[n1];
            Tile* tile2 = &grid.tiles[n2];
            Tile* tile3 = &grid.tiles[n3];

            //double tileWeight = (tile2->edgeCap < numBits) ? 1.0 * numBits / tile2->edgeCap : 1.0; //(1.0* tile2->edgeCap / max(1, 
            int refCap = tarWidth / (curBus->width[tile2->l] + spacing[tile2->l]);
            //double tileWeight = 1.0 * refCap / tile2->edgeCap; //(tile2->edgeCap < numBits) ? 1.0 * numBits / tile2->edgeCap : 1.0; //(1.0* tile2->edgeCap / max(1, 
            //double tileWeight = pow(10, 1.0 * refCap / tile2->edgeCap); //(tile2->edgeCap < numBits) ? 1.0 * numBits / tile2->edgeCap : 1.0; //(1.0* tile2->edgeCap / max(1, 
            double tileWeight = pow(1.0 * refCap / tile2->edgeCap, 5); //(tile2->edgeCap < numBits) ? 1.0 * numBits / tile2->edgeCap : 1.0; //(1.0* tile2->edgeCap / max(1, 
            //double tileWeight = 1.0 * numBits / tile2->edgeCap; //(tile2->edgeCap < numBits) ? 1.0 * numBits / tile2->edgeCap : 1.0; //(1.0* tile2->edgeCap / max(1, 
            printf("Tile (col : %d  row : %d  layer : %d) Cap %d Weight %2.2f\n", tile2->col, tile2->row, tile2->l, tile2->edgeCap, tileWeight); 
            //min(1.0, 1.0* numBits /tile2->edgeCap));
            if(tile3->l != tile1->l)
            {
                int x1 = min(tile2->x1, tile3->x1);
                int x2 = max(tile2->x2, tile3->x2);
                int y1 = min(tile2->y1, tile3->y1);
                int y2 = max(tile2->y2, tile3->y2);
                int l2 = tile3->l;
                tilePair.push_back({tile3->id, tile2->id});
                //printf("(1) (%d %d) (%d %d) M%d\n", x1, y1, x2, y2, l2);
                n3 = n1;
            }
            
            if(nodes[n1].backtrace == n1)
            {
                int x1 = min(tile1->x1, tile3->x1);
                int x2 = max(tile1->x2, tile3->x2);
                int y1 = min(tile1->y1, tile3->y1);
                int y2 = max(tile1->y2, tile3->y2);
                int l2 = tile1->l;
                tilePair.push_back({tile1->id, tile3->id});
                //printf("(1) (%d %d) (%d %d) M%d\n", x1, y1, x2, y2, l2);
            }
            
            n2 = n1;
        }


        SA = SearchArea(numLayers);

        for(int i=0; i < tilePair.size(); i++)
        {
            Tile* tile1 = &grid.tiles[tilePair[i].first];
            Tile* tile2 = &grid.tiles[tilePair[i].second];

            int col1 = min(tile1->col, tile2->col);
            int col2 = max(tile1->col, tile2->col);
            int row1 = min(tile1->row, tile2->row);
            int row2 = max(tile1->row, tile2->row);
            //int l1, l2;

            col1 = max(col1 - margin, 0);
            col2 = min(col2 + margin, numCols-1);
            row1 = max(row1 - margin, 0);
            row2 = min(row2 + margin, numRows-1);
            
                
            //col1 = max(col1 - (margin-1), 0);
            //col2 = min(col2 + (margin-1), numCols-1);
            //row1 = max(row1 - (margin-1), 0);
            //row2 = min(row2 + (margin-1), numRows-1);
            //l1 = max(tile1->l - 1, 0);
            //l2 = min(tile2->l + 1, numLayers-1);
            int l = tile1->l;
            //l1 = max(tile1->l, 0);
            //l2 = min(tile2->l, numLayers-1);
            
            int x1 = grid.get_tile(col1, row1, l)->x1; //grid.tileWidth * col1;
            int x2 = grid.get_tile(col2, row2, l)->x2; //grid.tileWidth * (col2 + 1);
            int y1 = grid.get_tile(col1, row1, l)->y1; //grid.tileHeight * row1;
            int y2 = grid.get_tile(col2, row2, l)->y2; //grid.tileHeight * (row2 + 1);

            box area(pt(x1, y1), pt(x2, y2));
            SA.trees[l].insert({area, 0});
            SA.lSeq.insert(SA.lSeq.end(), l);
            printf("(2) (%d %d) (%d %d) M%d\n", x1, y1, x2, y2, l);
            //for(int l = min(l1, l2); l <= max(l1, l2); l++)
            //{
            //    SA.trees[l].insert({area, 0});
                    //bg::append(SA.area[l], area);
            //}
            
        }

        printf("lSeq { ");
        for(int i =0; i < SA.lSeq.size(); i++)
        {
            printf("M%d", SA.lSeq[i]);
            if(i != SA.lSeq.size()-1)
                printf(" -> ");
        }
        printf(" }\n");
        // Define success
        SA.defined = true;

        printf("\n\n");
    }
    else
    {
        cout << "Fail local search area..." << endl;
        //exit(0);
    }


    return SA.defined;

}

bool OABusRouter::SearchArea::check_lseq(int dep, int l)
{
    if(defined)
    {
        if(dep >= lSeq.size())
            return false;
        else
        {
            return lSeq[dep] == l ? true : false;
        }
    }
    else
    {
        return true;
    }
}

bool OABusRouter::SearchArea::inside(int x, int y, int l, bool lCheck)
{
    if(defined)
    {
        if(lCheck)
        {
            vector<pair<box,int>> queries;
            trees[l].query(bgi::intersects(pt(x,y)), back_inserter(queries));
            return queries.size() > 0 ? true : false;
        }
        else
        {
            return inside(x,y);
            for(int i=0; i < trees.size(); i++)
            {
                if(is_vertical(i) == is_vertical(l))
                {
                    vector<pair<box,int>> queries;
                    trees[l].query(bgi::intersects(pt(x,y)), back_inserter(queries));
                    return queries.size() > 0 ? true : false;
                }
            }
            return false; //inside(x, y);
        }
    }
    else
    {
        return true;
    }
}

bool OABusRouter::SearchArea::inside(int x, int y)
{
    if(defined)
    {
        for(int l=0; l < trees.size(); l++)
        {
            vector<pair<box,int>> queries;
            trees[l].query(bgi::intersects(pt(x,y)), back_inserter(queries));
            return queries.size() > 0 ? true : false;
        }
        return false;
    }
    else
    {
        return true;
    }
}

