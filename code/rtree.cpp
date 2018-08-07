#include "route.h"
#include "circuit.h"

#define DEBUG_RTREE

SegRtree* OABusRouter::Router::GetTrackRtree()
{
    return &rtree.track;
}





void OABusRouter::Router::CreateTrackRtree()
{
    int numTracks, tid, curl;
    int x1, y1, x2, y2, i, dir;
    int offset;
    int& elemindex = rtree.elemindex;
    bool isVertical;
    SegmentBG curS;
    Track* curT;
    SegRtree* curRtree;


    curRtree = GetTrackRtree();
    numTracks = interval.numtracks;
    elemindex=0;

#ifdef DEBUG_RTREE
    if(numTracks > ckt->tracks.size())
    {
        printf("Invalid #tracks %d %d\n", numTracks, ckt->tracks.size());
        exit(0);
    }
#endif

    for(i=0; i < numTracks; i++)
    {
        IntervalSetT::iterator it_begin = interval.empty[i].begin();
        IntervalSetT::iterator it_end = interval.empty[i].end();
        isVertical = interval.is_vertical[i];
        offset = interval.offset[i];
        curl = interval.layer[i];

    
#ifdef DEBUG_RTREE
        if(ckt->tracks[i].offset != offset)
        {
            printf("Invalid offset number...\n");
            exit(0);
        }
#endif


        while(it_begin != it_end)
        {
            DiscreteIntervalT intv = (*it_begin++);
            tid = i;
            x1 = isVertical ? offset : intv.lower();
            x2 = isVertical ? offset : intv.upper();
            y1 = isVertical ? intv.lower() : offset;
            y2 = isVertical ? intv.upper() : offset;
            curS = SegmentBG(PointBG(x1,y1), PointBG(x2,y2));
            rtree.trackNuml[elemindex] = curl;
            rtree.trackID[elemindex] = tid;
            rtree.trackDir[elemindex] = isVertical ? VERTICAL : HORIZONTAL;
            curRtree->insert( { curS, elemindex++ } );


        }
    }
#ifdef DEBUG_RTREE
    for(i=0; i < elemindex; i++)
    {
        printf("elem %d -> track %d\n", i, rtree.trackID[i]);
    }
#endif
}





