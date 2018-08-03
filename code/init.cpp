#include "func.h"
#include "circuit.h"
#include "route.h"
#include <tuple>

using namespace OABusRouter;



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
        cout << "offset : " << curT->offset << endl;
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
}

