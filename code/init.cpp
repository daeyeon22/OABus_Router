#include "func.h"
#include "circuit.h"
#include "route.h"
#include <tuple>

using namespace OABusRouter;




void OABusRouter::Router::initialize()
{
    for(auto& mp : ckt->multipins)
    {
        int llx, lly, urx, ury;
        llx = INT_MAX;
        lly = INT_MAX;
        urx = INT_MIN;
        ury = INT_MIN;
        for(auto& pinid : mp.pins)
        {
            Pin* curpin = &ckt->pins[pinid];
            llx = min(llx, curpin->llx);
            lly = min(lly, curpin->lly);
            urx = max(urx, curpin->urx);
            ury = max(ury, curpin->ury);
        }


        multipin2llx[mp.id] = llx;
        multipin2lly[mp.id] = lly;
        multipin2urx[mp.id] = urx;
        multipin2ury[mp.id] = ury;
    }


    initialize_rtree();
    initialize_grid3d();
}

void OABusRouter::Router::initialize_rtree()
{
    int numTracks, numObstacles, numLayers, numPins, tid, curl;
    int trackid, l; 
    int x1, y1, x2, y2, i, dir;
    int offset;
    int& elemindex = rtree.elemindex;
    bool isVertical;
    SegmentBG curS;
    Track* curT;
    Pin* curpin;
    Obstacle* curobs;
    SegRtree* curRtree;
    Container* curct;

    typedef SegmentBG seg;
    typedef PointBG pt;
    typedef BoxBG box;


    curRtree = &rtree.track;
    numTracks = ckt->tracks.size(); //interval.numtracks;
    numObstacles = ckt->obstacles.size();
    numLayers = ckt->layers.size();
    numPins = ckt->pins.size();
    elemindex=0;

#ifdef DEBUG_RTREE
    if(numTracks > ckt->tracks.size())
    {
        printf("Invalid #tracks %d %d\n", numTracks, ckt->tracks.size());
        exit(0);
    }
#endif

    // Initialize Intervals.
    
    vector<SegRtree> trackrtree(numLayers);
    rtree.containers = vector<Container>(numTracks);
    //
    rtree.wire = vector<BoxRtree>(numLayers);
    rtree.obstacle = vector<BoxRtree>(numLayers);

    //
    for(i=0; i < numTracks; i++)
    {
        curT = &ckt->tracks[i];
        trackid = curT->id;
        curct = &rtree.containers[trackid];
        curct->trackid = trackid;
        curct->offset = curT->offset;
        curct->l = curT->l;
        curct->vertical = ckt->is_vertical(curT->l);
        curct->width = curT->width;


        x1 = curT->llx;
        y1 = curT->lly;
        x2 = curT->urx;
        y2 = curT->ury;
        l = curT->l;

        seg s(pt(x1, y1), pt(x2, y2));
        trackrtree[l].insert({s, trackid});

        curct->empty += (curct->vertical) ?
            IntervalT::open(y1, y2) : IntervalT::open(x1, x2);
    }
    
    // obstacle
    for(i=0; i < numObstacles; i++)
    {
        curobs = &ckt->obstacles[i];
        x1 = curobs->llx;
        y1 = curobs->lly;
        x2 = curobs->urx;
        y2 = curobs->ury;
        l = curobs->l;

        box b(pt(x1,y1), pt(x2,y2));
        // add into the obstacle tree
        rtree.obstacle[l].insert({b, OBSTACLE});
        vector<pair<seg, int>> queries;
        trackrtree[l].query(bgi::intersects(b), back_inserter(queries));
        for(auto& it : queries)
        {
            trackid = it.second;
            curct = &rtree.containers[trackid];

            curct->empty -= (curct->vertical)?
                IntervalT::open(y1, y2) : IntervalT::open(x1, x2);
        }
    }

    for(i=0; i < numPins; i++)
    {
        curpin = &ckt->pins[i];
        x1 = curpin->llx;
        y1 = curpin->lly;
        x2 = curpin->urx;
        y2 = curpin->ury;
        l = curpin->l;

        box b(pt(x1,y1), pt(x2,y2));
        vector<pair<seg, int>> queries;
        trackrtree[l].query(bgi::intersects(b), back_inserter(queries));
        for(auto& it : queries)
        {
            trackid = it.second;
            curct = &rtree.containers[trackid];

            curct->empty -= (curct->vertical)?
                IntervalT::open(y1, y2) : IntervalT::open(x1, x2);
        }
    }


    
    
    
    for(i=0; i < numTracks; i++)
    {
        curct = &rtree.containers[i];
        IntervalSetT::iterator it = curct->empty.begin();
        while(it != curct->empty.end())
        {
            x1 = curct->vertical ? curct->offset : it->lower();
            x2 = curct->vertical ? curct->offset : it->upper();
            y1 = curct->vertical ? it->lower() : curct->offset;
            y2 = curct->vertical ? it->upper() : curct->offset;
            seg s(pt(x1,y1), pt(x2,y2));
            curct->segs.push_back(s);
            curct->elems.push_back(elemindex);

            curRtree->insert({s, elemindex});

            rtree.elem2track[elemindex] = curct->trackid;
            elemindex++;
            it++;
        }
    }
}





// Initialize Grid3D
void OABusRouter::Router::initialize_grid3d()
{
    int minV, minH, minNumBit, maxNumBit;
    int numlayers, numbuses;
    int l_ver, l_hor;
    vector<int> xoffsets;
    vector<int> yoffsets;

    minV = INT_MAX;
    minH = INT_MAX;
    minNumBit = INT_MAX;
    maxNumBit = INT_MIN;
    numlayers = ckt->layers.size();
    numbuses = ckt->buses.size();

    for(int i=0; i < numlayers; i++)
    {
        Layer* curl = &ckt->layers[i];
        spacing[curl->id] = curl->spacing;
        
        if(curl->is_vertical())
        {

            if(minV > curl->offsets.size())
            {
                xoffsets.clear();
                minV = curl->offsets.size();
                xoffsets.insert(xoffsets.end(), curl->offsets.begin(), curl->offsets.end()); // = curl->offsets;
                l_ver = curl->id;
            }
        }
        else
        {

            if(minH > curl->offsets.size())
            {
                yoffsets.clear();
                minH = curl->offsets.size();
                yoffsets.insert(yoffsets.end(), curl->offsets.begin(), curl->offsets.end());
                //hoffsets = curl->offsets;
                l_hor = curl->id;
            }
        }
    }

    sort(xoffsets.begin(), xoffsets.end());
    sort(yoffsets.begin(), yoffsets.end());
    for(int i=0; i < numbuses; i++)
    {
        Bus* curB = &ckt->buses[i];
        minNumBit = min(curB->numBits, minNumBit);
        maxNumBit = max(curB->numBits, maxNumBit);
    }




    int numcols = (int)(1.0* minV / maxNumBit);
    int numrows = (int)(1.0* minH / maxNumBit);
    //int numcols = (int)(1.0* minH / maxNumBit);
    //int numrows = (int)(1.0* minV / maxNumBit);
    int originX = ckt->originX;
    int originY = ckt->originY;

#ifdef DEBUG_GRID
    for(auto& it : xoffsets) printf("x offset %d\n", it);
    for(auto& it : yoffsets) printf("y offset %d\n", it);
    printf("max num bits %d\nmin num bits %d\nmin vertical offsets %d\nmin horizontal offsets %d\n",
            maxNumBit, minNumBit, minV, minH);
    printf("#cols %d #rows %d\n", numcols, numrows);
#endif


    for(int i=0; i < numcols; i++)
    {
        int lower = maxNumBit*i;
        int lx = (lower==0)? originX : (int)(1.0*(xoffsets[lower] + xoffsets[lower-1])/2);
        grid.offsetxs.push_back(lx);
        //printf("x offset %d\n",lx);
    }

    for(int i=0; i < numrows; i++)
    {
        int lower = maxNumBit*i;
        int ly = (lower==0)? originY : (int)(1.0*(yoffsets[lower] + yoffsets[lower-1])/2);
        grid.offsetys.push_back(ly);
        //printf("y offset %d\n", ly);
    }

    grid.offsetxs.push_back(originX + ckt->width);
    grid.offsetys.push_back(originY + ckt->height);

    grid.numCols = numcols;
    grid.numRows = numrows;
    grid.numLayers = numlayers;
    grid.xoffset = originX;
    grid.yoffset = originY;
    grid.width = ckt->width;
    grid.height = ckt->height;
    grid.gcells = vector<Gcell>(numcols*numrows*numlayers, Gcell());

#ifdef DEBUG
    ////////////////////////////////////
    printf("GCell (%4d %4d) col: %3d row: %3d\n", GCELL_WIDTH, GCELL_HEIGHT, numCols, numRows);
    for(auto& os : this->grid.offsetxs)
    {
        printf("OffsetX %4d\n", os);
    }

    for(auto& os : this->grid.offsetys)
    {
        printf("OffsetY %4d\n", os);
    }
    printf("\n\n");
    
#endif


    // Initialize Gcell Capacitance
    for(int l=0; l < numlayers; l++)
    {
        Layer* curL = &ckt->layers[l];
        grid.initialize_gcell_cap(curL->id, curL->direction, curL->trackOffsets);
    }

    //exit(0);

}

void OABusRouter::Grid3D::initialize_gcell_cap(int layer, int dir, vector<int> &offsets)
{

    int GCllx, GClly, GCurx, GCury;
    //int originX, originY, width, height; //, lower;
    int cap, tid, curl, idx;
    int x1, x2, y1, y2;
    bool adj;
    vector<SegmentValT> queries;
    vector<int> resources;
    SegRtree* trackrtree;
    BoxBG queryBox;
    SegmentBG tmpseg;
    Rtree* rtree;
    rtree = &rou->rtree;
    trackrtree = &rtree->track;
    //dense_hash_map<int,int> &trackNuml= rou->rtree.trackNuml;
    //dense_hash_map<int,int> &trackID = rou->rtree.trackID;
    curl = layer;



    // HashMap preffered direction
    this->direction[layer] = dir;

    for(int col=0; col < numCols; col++)
    {
        for(int row=0; row < numRows; row++)
        {

            cap = 0;
            queries.clear();
            resources.clear();

            GCllx = GetOffset_x(col);
            GClly = GetOffset_y(row);
            GCurx = (col == numCols-1) ? xoffset + width : GetOffset_x(col+1);// + GCELL_WIDTH, xoffset + width) ;
            GCury = (row == numRows-1) ? yoffset + height : GetOffset_y(row+1);// + GCELL_HEIGHT, yoffset + height);

            //printf("box (%d %d) (%d %d)\n", GCllx, GClly, GCurx, GCury);

            queryBox = BoxBG(PointBG(GCllx, GClly), PointBG(GCurx, GCury));
            trackrtree->query(
                    bgi::intersects(queryBox) && // && !bgi::overlaps(queryBox), 
                    bgi::satisfies([&, curl, queryBox, rtree](const SegmentValT& val){
                        float llx, lly, urx, ury;
                        int tarl;
                        int idx;
                        bool fallLL, fallUR, targetL;
                        idx = val.second;
                        SegmentBG curSeg = val.first;
                        tarl = rtree->layer(idx); //trackNuml[idx];
                        llx = bg::get<0,0>(curSeg);
                        lly = bg::get<0,1>(curSeg);
                        urx = bg::get<1,0>(curSeg);
                        ury = bg::get<1,1>(curSeg);
                        fallLL = (bg::within(PointBG(llx,lly),queryBox))?true:false;
                        fallUR = (bg::within(PointBG(urx,ury),queryBox))?true:false;
#ifdef DEBUG_GRID
                        if(fallLL || fallUR)
                        {
                            cout << "Box -> " << bg::dsv(queryBox) << endl;
                            cout << "LL  -> " << bg::dsv(PointBG(llx,lly)) << endl;
                            cout << "UR  -> " << bg::dsv(PointBG(urx,ury)) << endl;
                        }
#endif
                        
                        targetL = (curl == tarl)?true:false;
                        return (!fallLL && !fallUR && targetL);
                    }), back_inserter(queries));

            //cout << "Queries size : " << queries.size() << endl;
            cap = queries.size();
                /*
            for(int i=0; i < queries.size(); i++)
            {
             
                tmpseg = queries[i].first;
                idx = queries[i].second;
                curl = trackNuml[idx];
                tid = trackID[idx];
                adj = (layer == curl)? true:false;

                x1 = (int)(bg::get<0,0>(tmpseg) + 0.5);
                y1 = (int)(bg::get<0,1>(tmpseg) + 0.5);
                x2 = (int)(bg::get<1,0>(tmpseg) + 0.5);
                y2 = (int)(bg::get<1,1>(tmpseg) + 0.5);

                PointBG pt1(x1,y1);
                PointBG pt2(x2,y2);

                if(bg::within(pt1,queryBox) || bg::within(pt2,queryBox))
                {
                    continue;
                }

                if(adj)
                {
                    resources.push_back(tid);
                    cap++;
                }
                
            }
                */
            
            //sort(resources.begin(), resources.end());

            Gcell* curGC = &gcells[GetIndex(col,row,layer)];
            for(int i=0; i < cap; i++)
            {
                int idx = queries[i].second;
                tid = rtree->trackid(idx); //trackID[idx];
                curGC->resources.insert(tid);   
            }
            //sort(curGC->resources.begin(), curGC->resources.end());

            curGC->id = GetIndex(col,row,layer);
            curGC->x = col;
            curGC->y = row;
            curGC->l = layer;
            curGC->cap = cap;
            curGC->direction = dir;
            //curGC->resources = resources;
            
#ifdef DEBUG_GRID
            printf("g%d (%d %d %d) cap %d\n", curGC->id, curGC->x, curGC->y, curGC->l, curGC->cap);
#endif
        }
    }


}


/*
void OABusRouter::Circuit::Init()
{
    //cout << "Init track" << endl;
    //InitTrack();
    //cout << "Get pitch" << endl;
    //Getpitch();
}

void OABusRouter::Circuit::Getpitch()
{
    int i, j;
    int minpitch, maxpitch;
    int numl, numos;
    int diff;

    numl = layers.size();
    Layer* curL;
    for(i=0; i < numl; i++)
    {
        curL = &layers[i];
        minpitch = INT_MAX;
        maxpitch = INT_MIN;

        numos = curL->trackOffsets.size() -1;
        for(j=0; j < numos; j++)
        {
            diff = abs(curL->trackOffsets[j] - curL->trackOffsets[j+1]);
            if(diff == 0) continue;

            minpitch = min(minpitch, diff);
            maxpitch = max(maxpitch, diff);
        }

        curL->minpitch = minpitch;
        curL->maxpitch = maxpitch;
    }

}

*/

/*
void OABusRouter::Router::InitInterval()
{
    int numtracks, numobstacles, numresults;
    int trackid, trackl, i, j, offset, maxwidth;
    int x1, y1, x2, y2, l;
    Track* curT;
    Layer* curL;
    Obstacle* curObs;
    numtracks = ckt->tracks.size();
    numobstacles = ckt->obstacles.size();
    
    // Initialize
    interval = Interval();
    interval.numtracks = numtracks;
    interval.empty = vector<IntervalSetT>(numtracks);
    interval.assign = vector<IntervalMapT>(numtracks);
    


    //vector<IntervalSetT> iset(numtracks);
    vector<SegmentValT> queries;

    SegRtree trackrtree;

    for(i=0; i < numtracks; i++)
    {
        curT = &ckt->tracks[i];
        trackid = curT->id;
        x1 = curT->llx;
        y1 = curT->lly;
        x2 = curT->urx;
        y2 = curT->ury;
        l = curT->l;

        
        SegmentBG seg(PointBG(x1, y1), PointBG(x2, y2));
        trackrtree.insert( { seg, trackid } );
             
        interval.empty[trackid] +=
            //iset[trackid] += 
            (ckt->is_vertical(l)) ? IntervalT::open(y1, y2) : IntervalT::open(x1, x2);  
        interval.is_vertical[trackid] = ckt->is_vertical(l);
        //cout << "offset : " << curT->offset << endl;
        interval.offset[trackid] = curT->offset;
        interval.layer[trackid] = l;
    }

    for(i=0; i < numobstacles; i++)
    {
        queries.clear();
        curObs = &ckt->obstacles[i];
        x1 = curObs->llx;
        x2 = curObs->urx;
        y1 = curObs->lly;
        y2 = curObs->ury;
        l = curObs->l;

        BoxBG box(PointBG(x1,y1), PointBG(x2, y2));
        trackrtree.query(bgi::intersects(box), back_inserter(queries)); 
        // && 
        //        bgi::satisfies([&,l,this](const pair<SegmentBG,int> &val){
        //            int trackid = val.second;
        //            int trackl = this->tracks[trackid].l;
        //            return (trackl == l);
        //            }), back_inserter(queries));

        numresults = queries.size();
        for(j=0; j < numresults; j++)
        {
            trackid = queries[j].second;
            trackl = ckt->tracks[trackid].l;       
            if(trackl == l)
            {
                interval.empty[trackid] -= 
                    interval.is_vertical[trackid] ? IntervalT::closed(y1, y2) : IntervalT::closed(x1, x2);
            }
        }
    }


    CreateTrackRtree();
    */
    /*
    for(i=0; i < numtracks; i++)
    {
        IntervalSetT::iterator it = interval.empty[i].begin();
        curT = &ckt->tracks[i];
        curL = &ckt->layers[curT->l];
        l = curL->id;
        offset = curT->offset;
        maxwidth = curT->width;

        while(it != iset[i].end())
        {
            DiscreteIntervalT intv = (*it++);
            x1 = is_vertical(l) ? offset : intv.lower();
            x2 = is_vertical(l) ? offset : intv.upper();
            y1 = is_vertical(l) ? intv.lower() : offset;
            y2 = is_vertical(l) ? intv.upper() : offset;
            Track t;
            t.id = tmp.size();
            t.width = maxwidth;
            t.offset = offset;
            t.llx = x1;
            t.lly = y1;
            t.urx = x2;
            t.ury = y2;
            t.l = l;

            tmp.push_back(t);
            curL->trackOffsets.push_back(t.offset);
            if(curL->offsets.find(offset) == curL->offsets.end())
                curL->offsets.insert(offset);

        }
    }

    tracks.clear();
    tracks = tmp;
    */   
//}

