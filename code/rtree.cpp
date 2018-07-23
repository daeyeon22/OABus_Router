#include "route.h"
#include "circuit.h"

SegRtree* OABusRouter::Router::GetTrackRtree()
{
    return &rtree.track;
}

void OABusRouter::Router::CreateTrackRtree()
{
    int numTracks, tid, curl;
    int x1, y1, x2, y2, i, dir;
    int elemindex;
    SegmentBG curS;
    Track* curT;
    SegRtree* curRtree;


    curRtree = GetTrackRtree();
    numTracks = ckt->tracks.size();
    elemindex=0;

    for(i=0; i < numTracks; i++)
    {
        curT = &ckt->tracks[i];
        tid = curT->id;
        x1 = curT->llx;
        y1 = curT->lly;
        x2 = curT->urx;
        y2 = curT->ury;
        curl = ckt->layerHashMap[curT->layer];
        dir = ckt->layers[curl].direction;

        curS = SegmentBG(PointBG(x1,y1), PointBG(x2,y2));
        
        this->rtree.trackNuml[elemindex] = curl;
        this->rtree.trackID[elemindex] = tid;
        this->rtree.trackDir[elemindex] = dir;

        curRtree->insert(make_pair(curS, elemindex++));
    }
}





