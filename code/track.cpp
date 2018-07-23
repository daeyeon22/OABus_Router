#include "circuit.h"
#include "route.h"
#include <algorithm>
#include <vector>
#include <iostream>

//#define GCELL_WIDTH rou->grid.GCELL_WIDTH
//#define GCELL_HEIGHT rou->grid.GCELL_HEIGHT



void OABusRouter::Grid3D::print()
{
    int row, col, layer, cap;
    int idx, numGCs;
    Gcell* curGC;

    numGCs = numRows* numCols* numLayers;

    for(layer = 0; layer < numLayers; layer++)
    {
        printf("layer M%2d\n", layer);
        for(row =0; row < numRows; row++)
        {
            for(col = 0; col < numCols; col++)
            {
                printf("___");
            }
            printf("\n");
            
            for(col = 0; col < numCols; col++)
            {

                
                idx = GetIndex(col, row, layer);
                curGC = &gcells[idx];
                cap = curGC->cap;
                printf("|%2d", cap);

                
                
                //printf("Current index(%3d) (%3d %3d %3d) Cap %d\n", idx, col, row, layer, cap);
                //cout << "   -> ";
                //for(auto& it : curGC->resources)
                //{
                //    cout << it << " ";
                //}
                //cout << endl << endl;
            }
            printf("|\n");
            
            


        }
        
        for(col = 0; col < numCols; col++)
        {
            printf("___");
        }
        printf("\n");

        
        printf("\n\n");
        //printf("\n\n");
    }

    
}





void OABusRouter::Router::TrackAssign()
{

    //this->grid.print();
    //exit(0);
    
    int numLayers, numRows, numCols, numSegs;
    int i, row, col, curl, dir, GCidx, bw;
    int x1,y1, x2, y2;
    int GCx1, GCy1, GCx2, GCy2;
    int curBW, tarBW, start, end;
    int MAXSIZE=100;
    int GCELL_WIDTH, GCELL_HEIGHT; 
    bool isVertical, isValid;
    bool ver1, ver2;


    
    BoxBG qbox;
    Segment* curS;
    SegRtree* curRtree;
    vector<SegmentValT> queries;
    numSegs = segs.size();
    //rtree = &this->rtree;
    set<int> set1, set2, set3;

    //vector<int> v1;
    //vector<int> v2;
    //vector<int> v3;
    //vector<int>::iterator it;

    GCELL_WIDTH = grid.GCELL_WIDTH;
    GCELL_HEIGHT = grid.GCELL_HEIGHT;
    curRtree = &rtree.track;

    /*
    for(i=0; i < numSegs; i++)
    {
        queries.clear();

        curS = &segs[i];
        curl = curS->l;
        GCx1 = min(curS->x1, curS->x2);
        GCy1 = min(curS->y1, curS->y2);
        GCx2 = max(curS->x1, curS->x2);
        GCy2 = max(curS->y1, curS->y2);
        
        x1 = grid.GetOffset_x(GCx1);
        y1 = grid.GetOffset_y(GCy1);
        x2 = grid.GetOffset_x(GCx2) + grid.GCELL_WIDTH;
        y2 = grid.GetOffset_y(GCy2) + grid.GCELL_HEIGHT;


        qbox = BoxBG(PointBG(x1,y1), PointBG(x2, y2));
        curRtree->query(bgi::intersects(qbox) && 
                bgi::satisfies([&, curl, qbox, this](const SegmentValT& val){
                        int llx, lly, urx, ury;
                        int tarl;
                        int idx;
                        bool fallLL, fallUR, targetL;
                        idx = val.second;
                        SegmentBG curSeg = val.first;
                        tarl = this->rtree.trackNuml[idx];
                        llx = bg::get<0,0>(curSeg);
                        lly = bg::get<0,1>(curSeg);
                        urx = bg::get<1,0>(curSeg);
                        ury = bg::get<1,1>(curSeg);
                        fallLL = (bg::within(PointBG(llx,lly),qbox))?true:false;
                        fallUR = (bg::within(PointBG(urx,ury),qbox))?true:false;
                        targetL = (curl == tarl)?true:false;
                        return (!fallLL && !fallUR && targetL);
                    }), back_inserter(queries)); 

        tarBW = bitwidth[curS->id];
        curBW = queries.size();

        if(curBW < tarBW)
        {
            cout << "Cur Bitwidth : " << curBW << endl;
            cout << "Tar Bitwidth : " << tarBW << endl;
        }else{
            
            Layer* curL = &ckt->layers[curl];
            isVertical = (curL->direction == VERTICAL)?true:false;
            if(isVertical)
            {
                printf("Segment (%d %d) (%d %d) M%d VERTICAL\n", x1, y1, x2, y2, curl);
            }
            else
            {
                printf("Segment (%d %d) (%d %d) M%d HORIZONTAL\n", x1, y1, x2, y2, curl);
            }

            
            for(auto& iter : queries)
            {
                int tmpx1, tmpx2, tmpy1, tmpy2, tmpl;
                Track* curT = &ckt->tracks[iter.second];
                tmpx1 = curT->llx;
                tmpx2 = curT->urx;
                tmpy1 = curT->lly;
                tmpy2 = curT->ury;
                tmpl = ckt->layerHashMap[curT->layer];
                
                printf("Track %d (%d %d) (%d %d) M%d\n", curT->id, tmpx1, tmpy1, tmpx2, tmpy2, tmpl);
            }
        }
    }
    */


    numSegs = this->segs.size();
    
    for(i=0; i < numSegs; i++)
    {
        curS = &this->segs[i];
        curl = curS->l;
        x1 = min(curS->x1, curS->x2);
        y1 = min(curS->y1, curS->y2);
        x2 = max(curS->x1, curS->x2);
        y2 = max(curS->y1, curS->y2);
        tarBW = this->bitwidth[curS->id];

        dir = this->grid.direction[curl];
        isVertical = (dir == VERTICAL)?true:false;

        
        // Initial resources
        col = x1;
        row = y1;
        GCidx = this->grid.GetIndex(col,row,curl);
        set1 = this->grid[GCidx]->resources;

        start = x1;
        end = x2;

        if(isVertical)
        {
            start = y1;
            end = y2;
        }

        isValid = true;

        ///////////////////////////////////////////////////////////////////////////
        //if(isVertical)
        //{
        //    printf("Segment (%d %d) (%d %d) M%d VERTICAL\n", x1, y1, x2, y2, curl);
        //}else{
        //    printf("Segment (%d %d) (%d %d) M%d HORIZONTAL\n", x1, y1, x2, y2, curl);
        //}
        ///////////////////////////////////////////////////////////////////////////

        
        while(start <= end)
        {
            GCidx = (isVertical)? this->grid.GetIndex(col,start,curl) : this->grid.GetIndex(start,row,curl);
            set2 = this->grid[GCidx]->resources;
            ////////////////////////////////////////// 
            // cout << "current Gcell resources" << endl;
            //if(isVertical)
            //{
            //    printf("Current Gcell (%d %d) M%d Cap %d\n", col, start, curl, set2.size());
            //}
            //else
            //{
            //    printf("Current Gcell (%d %d) M%d Cap %d\n", start, row, curl, set2.size());
            //}
            //for(auto& iter : set2)
            //{
            //    Track* curT = &ckt->tracks[iter];
            //    printf("Track %d (%d %d) (%d %d) %s\n", iter, curT->llx, curT->lly, curT->urx, curT->ury, curT->layer.c_str());
                //cout << iter << " ";
            //}
            //cout << endl << endl;
            ///////////////////////////////////////////
            
            
            //it = set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), v3.begin());
            set_intersection(set1.begin(), set1.end(), set2.begin(), set2.end(), inserter(set3, set3.begin()));

            //cout << "V3 -> ";
            //for(auto& iter : v3) cout << iter << " ";
            //cout << endl;
            //v3.resize(it - v3.begin());
            curBW = set3.size();
            
            //////////////////////////////////////////////////////////////
            //cout << "Set1 -> ";
            //for(auto& iter : set1) cout << iter << " ";
            //cout << endl;
            //cout << "Set2 -> ";
            //for(auto& iter : set2) cout << iter << " ";
            //cout << endl;
            //cout << "Set3 -> ";
            //for(auto& iter : set3) cout << iter << " ";
            //cout << endl;
            //printf("set1 %d set2 %d set3 %d\n\n", set1.size(), set2.size(), set3.size());
            //////////////////////////////////////////////////////////////

            if(curBW < tarBW)
            {
                printf("Bitwidth (%d %d)\n", curBW, tarBW);
                cout << "Fail to assign track..." << endl;
                isValid = false;
                break;
            }
            else
            {
                set1 = set3;
                set3.clear(); // = vector<int>(MAXSIZE);
            }


            //cout << endl << endl;
            start++;
        }

        if(isValid)
        {
            //printf("Segment(%2d) (%3d %3d) (%3d %3d) m%d\n", curS->id, x1, y1, x2, y2, curl);
            //for(auto& iter : set1)
            //{
            //    printf("Available resource index(%d)\n", iter);
            //}
            //printf("\n\n");
                       
            


        }

    }

}



    /*

    cout << "4" << endl;
*/





