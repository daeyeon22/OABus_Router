#include "circuit.h"
#include "route.h"
#include <algorithm>
#include <vector>
#include <iostream>

//#define GCELL_WIDTH rou->grid.GCELL_WIDTH
//#define GCELL_HEIGHT rou->grid.GCELL_HEIGHT
//#define DEBUG_TRACK

void OABusRouter::Router::CreateVia()
{

    int numJuncs, numBW, busid;
    int i, c, s1, s2, l1, l2, x, y;
    bool assign1, assign2, ver1, ver2, isLL1, isUR1, isLL2, isUR2;
    Junction *curJ;
    Segment *seg1, *seg2;
    Wire *wire1, *wire2;

    // for intersection function
    //vector<PointBG> intersection;
    SegmentBG tmp1, tmp2;
    //PointBG pt;

    
    numJuncs = junctions.size();
#ifdef DEBUG_TRACK
    cout << "# Junctions : " << numJuncs << endl;
#endif
    
    for(i=0; i < numJuncs; i++)
    {
        curJ = &junctions[i];
        busid = junc2bus[curJ->id];
        s1 = curJ->s1;
        s2 = curJ->s2;
        l1 = curJ->l1;
        l2 = curJ->l2;
        seg1 = &segs[s1];
        seg2 = &segs[s2];
        assign1 = assign[s1];
        assign2 = assign[s2];
        numBW = curJ->bw;
        ver1 = seg1->vertical;
        ver2 = seg2->vertical;



#ifdef DEBUG_TRACK
    printf("Segment(1) id %d (%d %d) (%d %d) M%d\n",s1,  seg1->x1, seg1->y1, seg1->x2, seg1->y2, seg1->l);
    printf("Segment(2) id %d (%d %d) (%d %d) M%d\n",s2,  seg2->x1, seg2->y1, seg2->x2, seg2->y2, seg2->l);

#endif

        isLL1 = ((seg1->x1 == curJ->x) && (seg1->y1 == curJ->y))? true : false;
        isUR1 = ((seg1->x2 == curJ->x) && (seg1->y2 == curJ->y))? true : false;
        isLL2 = ((seg2->x1 == curJ->x) && (seg2->y1 == curJ->y))? true : false;
        isUR2 = ((seg2->x2 == curJ->x) && (seg2->y2 == curJ->y))? true : false;




        // Valid if both segments are assigned
        if(assign1 && assign2)
        {
            for(c=0; c<numBW; c++)
            {
                wire1 = &wires[seg1->wires[c]];
                wire2 = &wires[seg2->wires[c]];




                // via coordinate
                if(ver1 && !ver2)
                {

                    x = wire1->x1;
                    y = wire2->y1;
                }
                else if(!ver1 && ver2)
                {
                    x = wire2->x1;
                    y = wire1->y1;
                }
                else if(ver1 && ver2)
                {

                    DiscreteIntervalT i1 = DiscreteIntervalT::closed(wire1->y1, wire1->y2);
                    DiscreteIntervalT i2 = DiscreteIntervalT::closed(wire2->y1, wire2->y2);
                    DiscreteIntervalT overlap = i1 & i2;
                    cout << i1 << endl;
                    cout << i2 << endl;
                    cout << overlap << endl;
                }
                else if(!ver1 && !ver2)
                {
                    DiscreteIntervalT i1 = DiscreteIntervalT::closed(wire1->x1, wire1->x2);
                    DiscreteIntervalT i2 = DiscreteIntervalT::closed(wire2->x1, wire2->x2);
                    DiscreteIntervalT overlap = i1 & i2;
                    cout << i1 << endl;
                    cout << i2 << endl;
                    cout << overlap << endl;
                }
                
                tmp1 = SegmentBG(PointBG(wire1->x1, wire1->y1), PointBG(wire1->x2, wire1->y2));
                tmp2 = SegmentBG(PointBG(wire2->x1, wire2->y1), PointBG(wire2->x2, wire2->y2));
#ifdef DEBUG_TRACK
                cout << "(1) " << bg::dsv(tmp1) << endl;
                cout << "(2) " << bg::dsv(tmp2) << endl;
#endif

                if(!bg::intersects(tmp1, tmp2))
                {
                    cout << "No Intersection point..." << endl;
                    //exit(0);
                }

                
                
                //pt = *intersection.begin();
                //x = (int)(bg::get<0>(intersection[0]) + 0.5);
                //x = (int)(bg::get<0>(pt) + 0.5);
                //y = (int)(bg::get<1>(intersection[0]) + 0.5);
                //y = (int)(bg::get<1>(pt) + 0.5);
#ifdef DEBUG_TRACK
                printf("x = %4d\ny = %4d\n", x, y);
#endif


                Via via;
                via.id = vias.size();
                via.x = x;
                via.y = y;
                via.l1 = l1;
                via.l2 = l2;
                via.w1 = wire1->id;
                via.w2 = wire2->id;

                vias.push_back(via);
                via2bus[via.id] = busid;


                // cut
                if(isLL1)
                {
                    wire1->x1 = x;
                    wire1->y1 = y;
                }

                if(isUR1)
                {
                    wire1->x2 = x;
                    wire1->y2 = y;

                }

                if(isLL2)
                {
                    wire2->x1 = x;
                    wire2->y1 = y;

                }

                if(isUR2)
                {
                    wire2->x2 = x;
                    wire2->y2 = y;

                }
            
            
            
            }
        }else{
            cout << "Invalid .." << endl;


        }

        // 
    }



}


void OABusRouter::Router::TrackAssign()
{
    //this->grid.print();
    //exit(0);
    
    int numLayers, numRows, numCols, numSegs;
    int i, row, col, curl, dir, GCidx, bw;
    int x1,y1, x2, y2, segid, busid, bitid, trackid, width, seq;
    int GCx1, GCy1, GCx2, GCy2;
    int curBW, tarBW, start, end;
    //int MAXSIZE=100;
    int GCELL_WIDTH, GCELL_HEIGHT; 
    int absx1, absx2, absy1, absy2;
    set<int>::iterator it;
    bool isVertical, isValid;
    bool ver1, ver2;
    
    BoxBG qbox;
    Segment* curS;
    Bus* curBus;
    Bit* curBit;
    Gcell* curGC;
    SegRtree* curRtree;
    vector<SegmentValT> queries;
    numSegs = segs.size();
    set<int> set1, set2, set3;


    GCELL_WIDTH = grid.GCELL_WIDTH;
    GCELL_HEIGHT = grid.GCELL_HEIGHT;
    curRtree = &rtree.track;
    numSegs = this->segs.size();
  
#ifdef DEBUG_TRACK
    printf("Total Number of Segments : %d\n", numSegs);
#endif

    
    // Iterating segments
    for(i=0; i < numSegs; i++)
    {
#ifdef DEBUG_TRACK
        printf("\n\n");
#endif 
        curS = &this->segs[i];
        segid = curS->id;
        busid = this->seg2bus[segid];

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

#ifdef DEBUG_TRACK
        if(isVertical)  printf("Segment (%d %d) (%d %d) M%d VERTICAL\n", x1, y1, x2, y2, curl);
        else            printf("Segment (%d %d) (%d %d) M%d HORIZONTAL\n", x1, y1, x2, y2, curl);
        printf("Start %d End %d\n", start, end);
#endif

        while(start <= end)
        {
            GCidx = (isVertical)? this->grid.GetIndex(col,start,curl) : this->grid.GetIndex(start,row,curl);
            set2 = this->grid[GCidx]->resources;
            //it = set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), v3.begin());
            set_intersection(set1.begin(), set1.end(), set2.begin(), set2.end(), inserter(set3, set3.begin()));
            curBW = set3.size();
 
#ifdef DEBUG_TRACK
            if(isVertical) printf("Current Gcell (%d %d) M%d Cap %d\n", col, start, curl, set2.size());
            else           printf("Current Gcell (%d %d) M%d Cap %d\n", start, row, curl, set2.size());
            for(auto& iter : set2)
            {
                Track* curT = &ckt->tracks[iter];
                printf("Track %d (%d %d) (%d %d) M%d\n", iter, curT->llx, curT->lly, curT->urx, curT->ury, curT->l);
            }
            printf("\n\nSet1 -> ");
            for(auto& iter : set1) cout << iter << " ";
            printf("\n\nSet2 -> ");
            for(auto& iter : set2) cout << iter << " ";
            printf("\n\nSet3 -> ");
            for(auto& iter : set3) cout << iter << " ";
            printf("\n\n\nset1 %d set2 %d set3 %d\n\n", set1.size(), set2.size(), set3.size());
#endif

           
            if(curBW < tarBW)
            {
                printf("Bitwidth (%d %d)\n", curBW, tarBW);
                cout << "Fail to assign track..." << endl;
                isValid = false;
                //
                set1 = set3;
                break;
            }
            else
            {
                set1 = set3;
                set3.clear(); // = vector<int>(MAXSIZE);
            }



            start++;
        }

        // Create wire
        if(isValid)
        {
            
            
            // Assign track for all bits
            assign[segid] = true;
            curBus = &ckt->buses[busid];
            it = set1.begin();
            tarBW = bitwidth[segid];
#ifdef DEBUG_TRACK
            printf("Segment(%2d) (%3d %3d) (%3d %3d) m%d Bitwidth %d\n", curS->id, x1, y1, x2, y2, curl, tarBW);
            //for(auto& iter : set1) printf("Available resource index(%d)\n", iter);
            printf("\n\n");
#endif


            for(int s=0; s < tarBW; s++)
            {
                seq = s;
                width = curBus->width[curl];
                bitid = curBus->bits[s];
                trackid = *it++;
               
                if(isVertical)
                {
                    start = y1;
                    end = y2;
                }
                else
                {
                    start = x1;
                    end = y2;
                }


                // Erase available resources
                while(start <= end)
                {
                    curGC = (isVertical)? grid[grid.GetIndex(col,start,curl)] : grid[grid.GetIndex(start,row,curl)];
                    curGC->resources.erase(trackid);
                    start++;
                }

                Wire curWire;
                curWire.id = wires.size();
                curWire.width = width;
                curWire.l = curl;
                curWire.busid = busid;
                curWire.bitid = bitid;
                curWire.trackid = trackid;
                curWire.seq = seq;
                curWire.vertical = isVertical;
                if(isVertical)
                {
                    curWire.x1 = ckt->tracks[trackid].offset;
                    curWire.x2 = ckt->tracks[trackid].offset;
                    curWire.y1 = grid.GetOffset_y(y1);
                    curWire.y2 = min(grid.GetOffset_y(y2) + GCELL_HEIGHT, grid.yoffset + grid.height);
                }
                else
                {
                    curWire.y1 = ckt->tracks[trackid].offset;
                    curWire.y2 = ckt->tracks[trackid].offset;
                    curWire.x1 = grid.GetOffset_x(x1);
                    curWire.x2 = min(grid.GetOffset_x(x2) + GCELL_WIDTH, grid.xoffset + grid.width);
                }

#ifdef DEBUG_TRACK
                printf("Create wire(%-4d) (%4d %-4d) (%4d %-4d) M%d --> %s\n",s,curWire.x1, curWire.y1, curWire.x2, curWire.y2, curWire.l, ckt->buses[curWire.busid].name.c_str());  
#endif
                wires.push_back(curWire);
                curS->wires.push_back(curWire.id);
            }




        }else{
#ifdef DEBUG_TRACK
            printf("#required tracks : %d\n", tarBW);
            printf("failed... current remained tracks: %d\n\n\n", set1.size());

#endif

        }
        ///
    }

#ifdef DEBUG_TRACK
    Wire* curW;
    
    printf("\n\n======================================================\n\n");
    for(i=0; i < wires.size(); i++)
    {
        curW = &wires[i];
        printf("Create wire(%-4d) (%4d %-4d) (%4d %-4d) M%d --> %s\n",i,curW->x1, curW->y1, curW->x2, curW->y2, curW->l, ckt->buses[curW->busid].name.c_str());  
    }

    printf("\n\n======================================================\n\n\n\n");
#endif

}



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


    /*

    cout << "4" << endl;
*/





